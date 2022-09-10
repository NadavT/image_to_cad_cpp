#include "voronoi.h"

#include "utils.h"

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
} // namespace polygon
} // namespace boost

VoronoiCalculator::VoronoiCalculator(const Image &image, const Segments &segments)
    : m_image(image)
    , m_segments(segments)
{
    TIMED_INNER_FUNCTION(calculate(), "Calculating voronoi");
    TIMED_INNER_FUNCTION(draw_graph(), "Drawing voronoi graph");
}

VoronoiDiagram &VoronoiCalculator::get_diagram()
{
    return m_diagram;
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
    return m_image.at<cv::Vec3b>({x, y})[2] == 255;
}

double VoronoiCalculator::distance_to_edge(cv::Point point, cv::Point edge_start, cv::Point edge_end)
{
    double x1 = edge_start.x;
    double y1 = edge_start.y;
    double x2 = edge_end.x;
    double y2 = edge_end.y;
    double x0 = point.x;
    double y0 = point.y;

    double numerator = std::abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1));
    double denominator = std::sqrt(std::pow(y2 - y1, 2) + std::pow(x2 - x1, 2));

    return numerator / denominator;
}

void VoronoiCalculator::draw_graph()
{
    int width = m_image.cols;
    int height = m_image.rows;
    cv::Mat image2(m_image);
    cv::Mat image_vor(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    int line_width = 1;
    for (const auto &segment : m_segments)
    {
        cv::polylines(image_vor, segment, false, cv::Scalar(0, 0, 0), line_width);
    }
    for (const auto &cell : m_diagram.cells())
    {
        auto *edge = cell.incident_edge();
        do
        {
            auto *start = edge->vertex0();
            auto *end = edge->vertex1();
            if (start && end && edge->is_primary() && edge->is_finite())
            {
                if (edge->is_linear())
                {
                    if (check_mask(start->x(), start->y()) && check_mask(end->x(), end->y()))
                    {
                        cv::line(image_vor, cv::Point(start->x(), start->y()), cv::Point(end->x(), end->y()),
                                 cv::Scalar(0, 0, 255), line_width);
                        cv::line(image2, cv::Point(start->x(), start->y()), cv::Point(end->x(), end->y()),
                                 cv::Scalar(0, 0, 255), line_width);
                    }
                }
                else
                {
                    if (check_mask(start->x(), start->y()) && check_mask(end->x(), end->y()))
                    {
                        cv::line(image_vor, cv::Point(start->x(), start->y()), cv::Point(end->x(), end->y()),
                                 cv::Scalar(0, 0, 255), line_width);
                        cv::line(image2, cv::Point(start->x(), start->y()), cv::Point(end->x(), end->y()),
                                 cv::Scalar(0, 0, 255), line_width);
                    }
                    assert(edge->cell()->source_category() != SOURCE_CATEGORY_SINGLE_POINT);
                    // point_type point =
                }
            }
            edge = edge->next();
        } while (edge != cell.incident_edge());
    }
    cv::imwrite("C:/technion/image_to_cad_cpp/results/voronoi.png", image_vor);
    cv::imwrite("C:/technion/image_to_cad_cpp/results/voronoi2.png", image2);
}
