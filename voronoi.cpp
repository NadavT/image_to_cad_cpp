#include "voronoi.h"

#include <boost/polygon/polygon.hpp>

#include "math.h"
#include "utils.h"
#include "voronoi_visual_utils.hpp"

namespace boost
{
namespace polygon
{
template <> struct geometry_concept<cv::Point>
{
    typedef point_concept type;
};

template <> struct point_traits<cv::Point>
{
    typedef int coordinate_type;

    static inline coordinate_type get(const cv::Point &point, orientation_2d orient)
    {
        return (orient == HORIZONTAL) ? point.x : point.y;
    }
};

template <> struct geometry_concept<std::vector<cv::Point>>
{
    typedef segment_concept type;
};

template <> struct segment_traits<std::vector<cv::Point>>
{
    typedef int coordinate_type;
    typedef cv::Point point_type;

    static inline point_type get(const std::vector<cv::Point> &segment, direction_1d dir)
    {
        return dir.to_int() ? segment[1] : segment[0];
    }
};

template <> struct segment_mutable_traits<std::vector<cv::Point>>
{
    typedef typename segment_traits<std::vector<cv::Point>>::coordinate_type coordinate_type;
    typedef typename segment_traits<std::vector<cv::Point>>::point_type point_type;

    static inline void set(std::vector<cv::Point> &segment, direction_1d dir, const point_type &point)
    {
        if (dir.to_int())
        {
            segment[1] = point;
        }
        else
        {
            segment[0] = point;
        }
    }

    static inline std::vector<cv::Point> construct(const point_type &low, const point_type &high)
    {
        return {low, high};
    }
};

} // namespace polygon
} // namespace boost

VoronoiCalculator::VoronoiCalculator(Image &image, const Image &grayscale_image)
    : m_image(image)
    , m_grayscale_image(grayscale_image)
{
    TIMED_INNER_FUNCTION(find_segments(), "Finding segments");
    TIMED_INNER_FUNCTION(calculate(), "Calculating voronoi");
    TIMED_INNER_FUNCTION(draw_graph(), "Drawing voronoi graph");
}

void VoronoiCalculator::find_segments()
{
    cv::Mat edged;
    std::vector<std::vector<cv::Point>> contours;

    cv::Canny(m_grayscale_image, edged, 30, 200);

    cv::findContours(edged, contours, m_hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    contours.reserve(contours.size() + 4);

    std::vector<cv::Point> line1(m_grayscale_image.rows);
    int i = 0;
    std::generate(line1.begin(), line1.end(), [&]() { return cv::Point(0, i++); });
    contours.push_back(line1);

    std::vector<cv::Point> line2(m_grayscale_image.cols);
    i = 0;
    std::generate(line2.begin(), line2.end(), [&]() { return cv::Point(i++, 0); });
    contours.push_back(line2);

    std::vector<cv::Point> line3(m_grayscale_image.rows);
    i = 0;
    std::generate(line3.begin(), line3.end(), [&]() { return cv::Point(m_grayscale_image.cols - 1, i++); });
    contours.push_back(line3);

    std::vector<cv::Point> line4(m_grayscale_image.cols);
    i = 0;
    std::generate(line4.begin(), line4.end(), [&]() { return cv::Point(i++, m_grayscale_image.rows - 1); });
    contours.push_back(line4);

    for (const auto &contour : contours)
    {
        const cv::Point *prev = nullptr;
        for (const cv::Point &point : contour)
        {
            if (prev)
            {
                m_segments.push_back({*prev, point});
            }
            prev = &point;
        }
    }

    m_segments.shrink_to_fit();
    cv::drawContours(m_image, m_segments, -1, {0xff, 0, 0}, 1);
    cv::imwrite("contours.png", m_image);
}

VoronoiDiagram &VoronoiCalculator::get_diagram()
{
    return m_diagram;
}

Graph &VoronoiCalculator::get_graph()
{
    return m_graph;
}

std::unordered_map<cv::Point, VertexDescriptor> &VoronoiCalculator::get_vertex_descriptor_map()
{
    return m_vertex_descriptor_map;
}

std::unordered_set<Segment> &VoronoiCalculator::get_added_edges()
{
    return m_added_edges;
}

void VoronoiCalculator::calculate()
{
    std::vector<cv::Point> points;
    boost::polygon::construct_voronoi(points.begin(), points.end(), m_segments.begin(), m_segments.end(), &m_diagram);
}

bool VoronoiCalculator::check_mask(int x, int y)
{
    if (x < 0 || y < 0 || x >= m_image.cols || y >= m_image.rows)
    {
        return false;
    }
    return m_image.at<cv::Vec3b>({x, y})[2] > 120;
}

void VoronoiCalculator::draw_graph()
{
    int width = m_image.cols;
    int height = m_image.rows;
    cv::Mat image2(m_image.clone());
    cv::Mat image3(m_image.clone());
    cv::Mat image_vor(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    int line_width = 1;
    for (const auto &segment : m_segments)
    {
        cv::polylines(image_vor, segment, false, cv::Scalar(0, 0, 0), line_width);
    }
    for (const auto &cell : m_diagram.cells())
    {
        auto *edge = cell.incident_edge();
        segment_type incident_segment = m_segments[cell.source_index()];
        if (!edge)
        {
            continue;
        }
        do
        {
            auto *start = edge->vertex0();
            auto *end = edge->vertex1();
            if (start && end && edge->is_primary() && edge->is_finite())
            {
                cv::Point start_p(start->x(), start->y());
                cv::Point end_p(end->x(), end->y());
                cv::line(image3, start_p, end_p, cv::Scalar(0, 0, 255), line_width);
                if (start_p != end_p && m_added_edges.count({start_p, end_p}) == 0 &&
                    m_added_edges.count({end_p, start_p}) == 0)
                {
                    if (edge->is_linear())
                    {
                        if (start != end && check_mask(start->x(), start->y()) && check_mask(end->x(), end->y()))
                        {
                            Vertex u = {start_p, incident_segment,
                                        distance_to_edge(start_p, incident_segment[0], incident_segment[1])};
                            Vertex v = {end_p, incident_segment,
                                        distance_to_edge(end_p, incident_segment[0], incident_segment[1])};
                            cv::line(image_vor, start_p, end_p, cv::Scalar(0, 0, 255), line_width);
                            cv::line(image2, start_p, end_p, cv::Scalar(0, 0, 255), line_width);
                            Graph::vertex_descriptor u_desc;
                            Graph::vertex_descriptor v_desc;
                            if (m_vertex_descriptor_map.count(start_p) == 0)
                            {
                                u_desc = boost::add_vertex(u, m_graph);
                                m_vertex_descriptor_map[start_p] = u_desc;
                            }
                            else
                            {
                                u_desc = m_vertex_descriptor_map[start_p];
                                m_graph[u_desc].distance_to_source =
                                    std::max(m_graph[u_desc].distance_to_source, u.distance_to_source);
                            }
                            if (m_vertex_descriptor_map.count(end_p) == 0)
                            {
                                v_desc = boost::add_vertex(v, m_graph);
                                m_vertex_descriptor_map[end_p] = v_desc;
                            }
                            else
                            {
                                v_desc = m_vertex_descriptor_map[end_p];
                                m_graph[v_desc].distance_to_source =
                                    std::max(m_graph[v_desc].distance_to_source, v.distance_to_source);
                            }
                            assert(u_desc != v_desc);
                            boost::add_edge(u_desc, v_desc, distance(start_p, end_p), m_graph);
                            m_added_edges.insert({start_p, end_p});
                        }
                    }
                    else
                    {
                        if (check_mask(start->x(), start->y()) && check_mask(end->x(), end->y()))
                        {
                            Vertex u = {start_p, incident_segment,
                                        distance_to_edge(start_p, incident_segment[0], incident_segment[1])};
                            Vertex v = {end_p, incident_segment,
                                        distance_to_edge(end_p, incident_segment[0], incident_segment[1])};
                            cv::line(image_vor, start_p, end_p, cv::Scalar(0, 0, 255), line_width);
                            cv::line(image2, start_p, end_p, cv::Scalar(0, 0, 255), line_width);
                            Graph::vertex_descriptor u_desc;
                            Graph::vertex_descriptor v_desc;
                            if (m_vertex_descriptor_map.count(start_p) == 0)
                            {
                                u_desc = boost::add_vertex(u, m_graph);
                                m_vertex_descriptor_map[start_p] = u_desc;
                            }
                            else
                            {
                                u_desc = m_vertex_descriptor_map[start_p];
                                m_graph[u_desc].distance_to_source =
                                    std::max(m_graph[u_desc].distance_to_source, u.distance_to_source);
                            }
                            if (m_vertex_descriptor_map.count(end_p) == 0)
                            {
                                v_desc = boost::add_vertex(v, m_graph);
                                m_vertex_descriptor_map[end_p] = v_desc;
                            }
                            else
                            {
                                v_desc = m_vertex_descriptor_map[end_p];
                                m_graph[v_desc].distance_to_source =
                                    std::max(m_graph[v_desc].distance_to_source, v.distance_to_source);
                            }
                            assert(u_desc != v_desc);
                            boost::add_edge(u_desc, v_desc, distance(start_p, end_p), m_graph);
                            m_added_edges.insert({start_p, end_p});
                        }
                        // TODO: Fixed parabolic solver
                        // if (check_mask(start->x(), start->y()) && check_mask(end->x(), end->y()))
                        // {
                        //     std::vector<cv::Point> samples(
                        //         {cv::Point(start->x(), start->y()), cv::Point(end->x(), end->y())});
                        //     sample_curved_edge(*edge, &samples);
                        //     for (std::size_t i = 1; i < samples.size(); ++i)
                        //     {
                        //         point_type vertex0 = samples[i - 1];
                        //         point_type vertex1 = samples[i];
                        //         if (vertex0.x >= 0 && vertex0.x <= width && vertex0.y >= 0 && vertex0.y <= height &&
                        //             vertex1.x >= 0 && vertex1.x <= width && vertex1.y >= 0 && vertex1.y <= height)
                        //         {
                        //             if (check_mask(vertex0.x, vertex0.y) && check_mask(vertex1.x, vertex1.y))
                        //             {
                        //                 cv::line(image_vor, vertex0, vertex1, cv::Scalar(0, 0, 255), line_width);
                        //                 cv::line(image2, vertex0, vertex1, cv::Scalar(0, 0, 255), line_width);
                        //             }
                        //         }
                        //     }
                        // }
                        assert(edge->cell()->source_category() != boost::polygon::SOURCE_CATEGORY_SINGLE_POINT);
                    }
                }
            }
            edge = edge->next();
        } while (edge != cell.incident_edge());
    }
    cv::imwrite("voronoi.png", image_vor);
    cv::imwrite("voronoi2.png", image2);
    cv::imwrite("voronoi4.png", image3);

    cv::Mat image_graph(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    auto edges = boost::edges(m_graph);

    for (auto it = edges.first; it != edges.second; ++it)
    {
        const Vertex &u = m_graph[boost::source(*it, m_graph)];
        const Vertex &v = m_graph[boost::target(*it, m_graph)];
        cv::line(image_graph, u.p, v.p, cv::Scalar(0, 0, 255), line_width);
    }

    cv::imwrite("voronoi3.png", image_graph);
}

point_type VoronoiCalculator::retrieve_point(const cell_type &cell)
{
    source_index_type index = cell.source_index();
    source_category_type category = cell.source_category();
    assert(category != boost::polygon::SOURCE_CATEGORY_SINGLE_POINT);
    if (category == boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT)
    {
        return boost::polygon::low(m_segments[index]);
    }
    else
    {
        return boost::polygon::high(m_segments[index]);
    }
}

segment_type VoronoiCalculator::retrieve_segment(const cell_type &cell)
{
    source_index_type index = cell.source_index();
    return m_segments[index];
}

using namespace boost::polygon;

void VoronoiCalculator::sample_curved_edge(const edge_type &edge, std::vector<point_type> *sampled_edge)
{
    coordinate_type max_dist = 0;
    point_type point =
        edge.cell()->contains_point() ? retrieve_point(*edge.cell()) : retrieve_point(*edge.twin()->cell());
    segment_type segment =
        edge.cell()->contains_point() ? retrieve_segment(*edge.twin()->cell()) : retrieve_segment(*edge.cell());
    segment_data<coordinate_type> segment_a(segment[0], segment[1]);
    voronoi_visual_utils<coordinate_type>::discretize(point, segment_a, max_dist, sampled_edge);
}
