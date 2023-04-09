#include "curves_generator.h"

#include "math.h"
#include "process_graph.h"
#include "utils.h"

#include <cmath>
#include <execution>
#include <inc_irit/allocate.h>
#include <inc_irit/cagd_lib.h>
#include <inc_irit/geom_lib.h>
#include <inc_irit/ip_cnvrt.h>
#include <inc_irit/iritprsr.h>

#include <boost/graph/dijkstra_shortest_paths.hpp>

#define INTERSECTION_PROXIMITY_THRESHOLD 0.00001

static inline unsigned int point_to_index(const cv::Point &point, const int width)
{
    return point.y * width + point.x;
}

CurvesGenerator::CurvesGenerator(Graph &graph, int max_order, int target_order, double extrusion_amount,
                                 bool filter_curves, const Image &reference_image, int distance_to_boundary_samples,
                                 int distance_to_boundary_threshold, double distance_in_boundary_backoff,
                                 double distance_in_boundary_factor, double curve_density, int min_curve_length,
                                 double junction_radius_adder)
    : m_graph(graph)
    , m_curves()
    , m_max_order(max_order)
    , m_target_order(target_order)
    , m_offset_curves()
    , m_filter_offset_curves(filter_curves)
    , m_extrusion_amount(extrusion_amount)
    , m_reference_image(reference_image)
    , m_distance_to_boundary_samples(distance_to_boundary_samples)
    , m_distance_to_boundary_threshold(distance_to_boundary_threshold)
    , m_distance_in_boundary_backoff(distance_in_boundary_backoff)
    , m_distance_in_boundary_factor(distance_in_boundary_factor)
    , m_curve_density(curve_density)
    , m_min_curve_length(min_curve_length)
    , m_image_graph(m_reference_image.cols * m_reference_image.rows)
    , m_junction_radius_adder(junction_radius_adder)
{
    if (max_order == -1)
    {
        m_max_order = std::numeric_limits<int>::max();
    }
    if (target_order == -1)
    {
        m_target_order = m_max_order;
    }

    if (m_filter_offset_curves)
    {
        TIMED_INNER_FUNCTION(generate_image_graph(), "Generating image graph");
    }
    TIMED_INNER_FUNCTION(generate_curves(), "Generating curves");
    TIMED_INNER_FUNCTION(decrease_curves_order(), "Decrease curves order");
    TIMED_INNER_FUNCTION(generate_offset_curves(), "Generating offset curves");
    TIMED_INNER_FUNCTION(sort_junction_curves(), "Sorting junction curves");
    TIMED_INNER_FUNCTION(generate_surfaces_from_junctions(), "Generating surfaces from junctions");
    TIMED_INNER_FUNCTION(find_neighborhoods_intersections(), "Finding neighborhoods intersections");
    TIMED_INNER_FUNCTION(fill_holes(), "Filling holes");
    TIMED_INNER_FUNCTION(generate_surfaces_from_curves(), "Generating surfaces from curves");
    TIMED_INNER_FUNCTION(extrude_surfaces(), "Extruding surfaces");
}

void CurvesGenerator::write_curves(const std::string &filename)
{
    int handler = IPOpenDataFile(filename.c_str(), FALSE, TRUE);
    for (auto &curve : m_height_curves)
    {
        char *error;
        BzrCrvWriteToFile2(curve.get(), handler, 4, nullptr, &error);
    }
    IPCloseStream(handler, TRUE);
}

void CurvesGenerator::write_offset_curves_before_trim(const std::string &filename)
{
    int handler = IPOpenDataFile(filename.c_str(), FALSE, TRUE);
    for (auto &curve : m_offset_curves_before_trim)
    {
        char *error;
        BzrCrvWriteToFile2(curve.get(), handler, 4, nullptr, &error);
    }
    IPCloseStream(handler, TRUE);
}

void CurvesGenerator::write_offset_curves(const std::string &filename)
{
    int handler = IPOpenDataFile(filename.c_str(), FALSE, TRUE);
    for (auto &curve : m_offset_curves)
    {
        char *error;
        BzrCrvWriteToFile2(std::get<0>(curve).get(), handler, 4, nullptr, &error);
    }
    IPCloseStream(handler, TRUE);
}

void CurvesGenerator::write_filtered_offset_curves(const std::string &filename)
{
    int handler = IPOpenDataFile(filename.c_str(), FALSE, TRUE);
    for (auto &curve : m_filtered_offset_curves)
    {
        char *error;
        BzrCrvWriteToFile2(std::get<0>(curve).get(), handler, 4, nullptr, &error);
    }
    IPCloseStream(handler, TRUE);
}

void CurvesGenerator::write_surfaces(const std::string &filename)
{
    int handler = IPOpenDataFile(filename.c_str(), FALSE, TRUE);
    for (auto &surface : m_surfaces)
    {
        char *error;
        BzrSrfWriteToFile2(surface.get(), handler, 4, nullptr, &error);
    }
    IPCloseStream(handler, TRUE);
}
void CurvesGenerator::write_extrusions(const std::string &filename)
{
    int handler = IPOpenDataFile(filename.c_str(), FALSE, TRUE);
    for (auto &extrusion : m_extrusions)
    {
        char *error;
        TrivTVWriteToFile2(extrusion.get(), handler, 4, nullptr, &error);
    }
    IPCloseStream(handler, TRUE);
}

void CurvesGenerator::generate_image_graph()
{
    std::cout << "\t\tFinished " << 0 << " colums out of " << m_reference_image.cols << std::endl;
    m_image_graph.m_vertices.reserve(m_reference_image.cols * m_reference_image.rows);
    for (int i = 0; i < m_reference_image.cols; i++)
    {
        for (int j = 0; j < m_reference_image.rows; j++)
        {
            if (m_reference_image.at<cv::Vec3b>({i, j})[2] >= 120)
            {
                continue;
            }
            cv::Point p(i, j);
            for (int inner_i = std::max(i - 1, 0); inner_i <= std::min(i + 1, m_reference_image.cols - 1); inner_i++)
            {
                for (int inner_j = std::max(j - 1, 0); inner_j <= std::min(j + 1, m_reference_image.rows - 1);
                     inner_j++)
                {
                    if (m_reference_image.at<cv::Vec3b>({i, j})[2] >= 120 || (inner_i == i && inner_j == j))
                    {
                        continue;
                    }
                    cv::Point inner_p(inner_i, inner_j);
                    ImageGraph::vertex_descriptor v1 = point_to_index(p, m_reference_image.cols);
                    ImageGraph::vertex_descriptor v2 = point_to_index(inner_p, m_reference_image.cols);

                    if (inner_i != i && inner_j != j)
                    {
                        boost::add_edge(v1, v2, std::sqrt(2), m_image_graph);
                    }
                    else
                    {
                        boost::add_edge(v1, v2, 1, m_image_graph);
                    }
                }
            }
        }
        std::cout << "\x1b[A";
        std::cout << "\t\tFinished " << i << " columns out of " << m_reference_image.cols << std::endl;
    }
    std::cout << "Image graph has " << boost::num_vertices(m_image_graph) << " vertices and "
              << boost::num_edges(m_image_graph) << " edges" << std::endl;
}

void CurvesGenerator::generate_curves()
{
    // Get candidates
    std::vector<std::tuple<EdgeDescriptor, VertexDescriptor>> candidates;
    for (const auto &vertex : boost::make_iterator_range(boost::vertices(m_graph)))
    {
        if (boost::degree(vertex, m_graph) > 0 && boost::degree(vertex, m_graph) != 2)
        {
            for (const auto &edge : boost::make_iterator_range(boost::out_edges(vertex, m_graph)))
            {
                candidates.push_back({edge, vertex});
            }
        }
    }

    // Populate edges to be processed
    std::set<EdgeDescriptor> remaining;
    for (const auto &edge : boost::make_iterator_range(boost::edges(m_graph)))
    {
        remaining.insert(edge);
    }

    // Generate curves
    while (remaining.size() > 0)
    {
        EdgeDescriptor edge;
        // Take candidate if available
        if (candidates.size() > 0)
        {
            edge = std::get<0>(candidates.back());
            candidates.pop_back();
        }
        else
        {
            // Otherwise take any remaining edge
            edge = *remaining.begin();
        }
        if (remaining.count(edge) == 0)
        {
            continue;
        }
        remaining.erase(edge);
        std::vector<VertexDescriptor> route;
        std::vector<EdgeDescriptor> route_edges;
        std::tie(std::ignore, route, route_edges) = ProcessGraph::walk_to_next_junction(
            boost::source(edge, m_graph), boost::target(edge, m_graph), m_graph, true);
        for (auto &item : split_junction_walk({route, route_edges}))
        {
            generate_curve(item.first, item.second);
        }

        // Clean route_edges
        for (const auto &edge : route_edges)
        {
            remaining.erase(edge);
        }
    }
}

void CurvesGenerator::decrease_curves_order()
{
    if (m_target_order == m_max_order)
    {
        return;
    }

    decltype(m_curves) new_curves;
    decltype(m_height_curves) new_height_curves;
    int i = 0;
    std::mutex lock;
    std::cout << "\t\tFinished " << i << " curves out of " << m_curves.size() << std::endl;
    std::for_each(std::execution::par, std::begin(m_curves), std::end(m_curves), [&](auto &item) {
        Curve &original_curve = std::get<0>(item);
        Curve &original_width_curve = std::get<1>(item);
        Curve &original_opposite_width_curve = std::get<2>(item);
        VertexDescriptor junction1 = std::get<3>(item);
        VertexDescriptor junction2 = std::get<4>(item);
        int length = original_curve->Length;
        int order = (original_curve->Length > m_target_order - 1) ? m_target_order : original_curve->Length;
        double arc_len = CagdCrvArcLenPoly(original_curve.get());
        Curve final_curve = Curve(nullptr, CagdCrvFree);
        Curve final_width_curve = Curve(nullptr, CagdCrvFree);
        Curve final_opposite_width_curve = Curve(nullptr, CagdCrvFree);
        Curve final_height_curve = Curve(nullptr, CagdCrvFree);
        if (m_target_order != m_max_order || length / arc_len > m_curve_density)
        {
            int final_length = std::ceil(arc_len * m_curve_density);
            if (final_length < m_min_curve_length)
            {
                final_length = m_min_curve_length;
            }
            int final_order = (final_length > m_target_order - 1) ? m_target_order : final_length;
            Curve curve = Curve(BspCrvNew(final_length, final_order, CAGD_PT_E2_TYPE), CagdCrvFree);
            Curve width_curve = Curve(BspCrvNew(final_length, final_order, CAGD_PT_E1_TYPE), CagdCrvFree);
            Curve opposite_width_curve = Curve(BspCrvNew(final_length, final_order, CAGD_PT_E1_TYPE), CagdCrvFree);
            Curve height_curve = Curve(BspCrvNew(final_length, final_order, CAGD_PT_E3_TYPE), CagdCrvFree);
            BspKnotUniformOpen(final_length, final_order, curve->KnotVector);
            BspKnotUniformOpen(final_length, final_order, width_curve->KnotVector);
            BspKnotUniformOpen(final_length, final_order, opposite_width_curve->KnotVector);
            BspKnotUniformOpen(final_length, final_order, height_curve->KnotVector);
            for (int j = 0; j < final_length; ++j)
            {
                CagdPtStruct *point = CagdPtNew();
                CAGD_CRV_EVAL_E2(original_curve.get(), static_cast<CagdRType>(j) / (final_length - 1), &point->Pt[0]);
                curve->Points[1][j] = point->Pt[0];
                curve->Points[2][j] = point->Pt[1];
                CAGD_CRV_EVAL_SCALAR(original_width_curve.get(), static_cast<CagdRType>(j) / (final_length - 1),
                                     width_curve->Points[1][j]);
                CAGD_CRV_EVAL_SCALAR(original_opposite_width_curve.get(),
                                     static_cast<CagdRType>(j) / (final_length - 1),
                                     opposite_width_curve->Points[1][j]);

                height_curve->Points[1][j] = curve->Points[1][j];
                height_curve->Points[2][j] = curve->Points[2][j];
                height_curve->Points[3][j] = width_curve->Points[1][j];
                CagdPtFree(point);
            }
            final_curve = std::move(curve);
            final_width_curve = std::move(width_curve);
            final_opposite_width_curve = std::move(opposite_width_curve);
            final_height_curve = std::move(height_curve);
        }
        else
        {
            final_curve = std::move(original_curve);
            final_width_curve = std::move(original_width_curve);
            final_opposite_width_curve = std::move(original_opposite_width_curve);
            Curve height_curve =
                Curve(BspCrvNew(final_curve->Length, final_curve->Order, CAGD_PT_E3_TYPE), CagdCrvFree);
            for (int j = 0; j < final_curve->Length; ++j)
            {
                height_curve->Points[1][j] = final_curve->Points[1][j];
                height_curve->Points[2][j] = final_curve->Points[2][j];
                height_curve->Points[3][j] = final_width_curve->Points[1][j];
            }
            final_height_curve = std::move(height_curve);
        }
        lock.lock();
        new_curves.push_back({std::move(final_curve), std::move(final_width_curve),
                              std::move(final_opposite_width_curve), junction1, junction2});
        new_height_curves.push_back(std::move(final_height_curve));
        i++;
        std::cout << "\x1b[A";
        std::cout << "\t\tFinished " << i << " curves out of " << m_curves.size() << std::endl;
        lock.unlock();
    });
    m_curves = std::move(new_curves);
    m_height_curves = std::move(new_height_curves);
}

void CurvesGenerator::generate_offset_curves()
{
    int i = 0;
    std::mutex junctions_to_curves_lock;
    std::mutex curves_before_trim_lock;
    std::mutex update_lock;
    std::mutex marked_lock;
    std::mutex progress_lock;
    std::cout << "\t\tFinished " << i << " curves out of " << m_curves.size() << std::endl;
    std::for_each(std::execution::par, std::begin(m_curves), std::end(m_curves), [&](auto &item) {
        const Curve &curve = std::get<0>(item);
        const Curve &width_curve = std::get<1>(item);
        const Curve &opposite_width_curve = std::get<2>(item);
        std::vector<VertexDescriptor> junctions;
        if (std::get<3>(item) != -1)
        {
            junctions.push_back(std::get<3>(item));
            junctions_to_curves_lock.lock();
            if (m_junction_to_curves.count(std::get<3>(item)) == 0)
            {
                m_junction_to_curves[std::get<3>(item)] = std::vector<CagdCrvStruct *>();
            }
            junctions_to_curves_lock.unlock();
        }
        if (std::get<4>(item) != -1)
        {
            junctions.push_back(std::get<4>(item));
            junctions_to_curves_lock.lock();
            if (m_junction_to_curves.count(std::get<4>(item)) == 0)
            {
                m_junction_to_curves[std::get<4>(item)] = std::vector<CagdCrvStruct *>();
            }
            junctions_to_curves_lock.unlock();
        }
        junctions_to_curves_lock.lock();
        m_curve_to_junctions[curve.get()] = junctions;
        for (const auto &junction : junctions)
        {
            m_junction_to_curves[junction].push_back(curve.get());
        }
        junctions_to_curves_lock.unlock();

        Curve offset_curve = Curve(SymbCrvVarOffset(curve.get(), width_curve.get(), FALSE), CagdCrvFree);
        Curve opposite_offset_curve =
            Curve(SymbCrvVarOffset(curve.get(), opposite_width_curve.get(), FALSE), CagdCrvFree);
        Curve new_offset_curve = trim_curve_to_fit_boundary(curve, width_curve, offset_curve);
        Curve new_opposite_offset_curve =
            trim_curve_to_fit_boundary(curve, opposite_width_curve, opposite_offset_curve);
        fix_offset_curves_surface_self_intersection(new_offset_curve, new_opposite_offset_curve);
        curves_before_trim_lock.lock();
        m_offset_curves_before_trim.push_back(std::move(offset_curve));
        m_offset_curves_before_trim.push_back(std::move(opposite_offset_curve));
        curves_before_trim_lock.unlock();
        if (new_offset_curve != nullptr && new_opposite_offset_curve != nullptr)
        {
            bool should_filter = false;
            if (m_filter_offset_curves)
            {
                CagdPtStruct *point = CagdPtNew();
                double max_distance_offset_curve = 0;
                double min_distance_offset_curve = std::numeric_limits<double>::max();
                double max_distance_opposite_curve = 0;
                double min_distance_opposite_curve = std::numeric_limits<double>::max();
                for (int i = 0; i < m_distance_to_boundary_samples; i++)
                {
                    double sample = m_distance_in_boundary_backoff +
                                    (static_cast<double>(i) / (m_distance_to_boundary_samples - 1)) *
                                        (1 - 2 * m_distance_in_boundary_backoff);
                    CAGD_CRV_EVAL_E2(new_offset_curve.get(), sample, &point->Pt[0]);
                    cv::Point p(point->Pt[0], point->Pt[1]);
                    double distance = distance_to_boundary(p, m_distance_to_boundary_threshold * 2);
                    if (distance > max_distance_offset_curve)
                    {
                        max_distance_offset_curve = distance;
                    }
                    if (distance < min_distance_opposite_curve)
                    {
                        min_distance_opposite_curve = distance;
                    }
                    CAGD_CRV_EVAL_E2(new_opposite_offset_curve.get(),
                                     static_cast<double>(i) / (m_distance_to_boundary_samples - 1), &point->Pt[0]);
                    cv::Point p2(point->Pt[0], point->Pt[1]);
                    distance = distance_to_boundary(p2, m_distance_to_boundary_threshold * 2);
                    if (distance > max_distance_opposite_curve)
                    {
                        max_distance_opposite_curve = distance;
                    }
                    if (distance < min_distance_opposite_curve)
                    {
                        min_distance_opposite_curve = distance;
                    }
                }

                CAGD_CRV_EVAL_E2(new_offset_curve.get(), m_distance_in_boundary_backoff, &point->Pt[0]);
                cv::Point p0(point->Pt[0], point->Pt[1]);
                CAGD_CRV_EVAL_E2(new_offset_curve.get(), 1 - m_distance_in_boundary_backoff, &point->Pt[0]);
                cv::Point p1(point->Pt[0], point->Pt[1]);
                CAGD_CRV_EVAL_E2(new_opposite_offset_curve.get(), m_distance_in_boundary_backoff, &point->Pt[0]);
                cv::Point p2(point->Pt[0], point->Pt[1]);
                CAGD_CRV_EVAL_E2(new_opposite_offset_curve.get(), 1 - m_distance_in_boundary_backoff, &point->Pt[0]);
                cv::Point p3(point->Pt[0], point->Pt[1]);
                double max_diff = std::max({(max_distance_offset_curve - min_distance_offset_curve) /
                                                CagdCrvArcLenPoly(new_offset_curve.get()),
                                            (max_distance_opposite_curve - min_distance_opposite_curve) /
                                                CagdCrvArcLenPoly(new_opposite_offset_curve.get())});
                CagdPtFree(point);
                double offset_curve_length = CagdCrvArcLenPoly(new_offset_curve.get());
                double opposite_offset_curve_length = CagdCrvArcLenPoly(new_opposite_offset_curve.get());
                should_filter =
                    (max_distance_offset_curve >= m_distance_to_boundary_threshold ||
                     max_distance_opposite_curve >= m_distance_to_boundary_threshold || max_diff >= 0.1 ||
                     (offset_curve_length * (1 - 2 * m_distance_in_boundary_backoff)) * 0.5 >=
                         distance_in_boundary(closest_point_on_boundary(p0, m_distance_to_boundary_threshold),
                                              closest_point_on_boundary(p1, m_distance_to_boundary_threshold)) ||
                     distance_in_boundary(closest_point_on_boundary(p0, m_distance_to_boundary_threshold),
                                          closest_point_on_boundary(p1, m_distance_to_boundary_threshold)) >=
                         m_distance_in_boundary_factor * offset_curve_length ||
                     (opposite_offset_curve_length * (1 - 2 * m_distance_in_boundary_backoff)) * 0.5 >=
                         distance_in_boundary(closest_point_on_boundary(p2, m_distance_to_boundary_threshold),
                                              closest_point_on_boundary(p3, m_distance_to_boundary_threshold)) ||
                     distance_in_boundary(closest_point_on_boundary(p2, m_distance_to_boundary_threshold),
                                          closest_point_on_boundary(p3, m_distance_to_boundary_threshold)) >=
                         m_distance_in_boundary_factor * opposite_offset_curve_length);
            }

            if (junctions.size() < 2 || !should_filter)
            {
                update_lock.lock();
                m_offset_curves.push_back({std::move(new_offset_curve), junctions});
                m_offset_curves.push_back({std::move(new_opposite_offset_curve), junctions});
                m_curve_to_offset_curves[curve.get()] = {std::get<0>(m_offset_curves.back()).get(),
                                                         std::get<0>(*std::prev(m_offset_curves.end(), 2)).get()};
                if (junctions.size() == 2)
                {
                    m_connections.insert({junctions[0], junctions[1]});
                    m_connections.insert({junctions[1], junctions[0]});
                }
                update_lock.unlock();
            }
            else
            {
                marked_lock.lock();
                for (const auto &junction : junctions)
                {
                    if (m_marked_junctions.count(junction) > 0)
                    {
                        m_marked_junctions[junction]++;
                    }
                    else
                    {
                        m_marked_junctions[junction] = 1;
                    }
                }
                m_filtered_offset_curves.push_back({std::move(new_offset_curve), junctions});
                m_filtered_offset_curves.push_back({std::move(new_opposite_offset_curve), junctions});
                marked_lock.unlock();
            }
        }
        else
        {
            marked_lock.lock();
            for (const auto &junction : junctions)
            {
                if (m_marked_junctions.count(junction) > 0)
                {
                    m_marked_junctions[junction]++;
                }
                else
                {
                    m_marked_junctions[junction] = 1;
                }
            }
            marked_lock.unlock();
        }
        progress_lock.lock();
        i++;
        std::cout << "\x1b[A";
        std::cout << "\t\tFinished " << i << " curves out of " << m_curves.size() << std::endl;
        progress_lock.unlock();
    });
}

void CurvesGenerator::sort_junction_curves()
{
    for (auto &item : m_junction_to_curves)
    {
        const cv::Point &junction_point = m_graph[item.first].p;
        std::sort(item.second.begin(), item.second.end(),
                  [junction_point](const CagdCrvStruct *a, const CagdCrvStruct *b) {
                      cv::Point a_point = cv::Point(a->Points[1][1], a->Points[2][1]);
                      if (distance(a_point, junction_point) >
                          distance(cv::Point(a->Points[1][a->Length - 2], a->Points[2][a->Length - 2]), junction_point))
                      {
                          a_point = cv::Point(a->Points[1][a->Length - 2], a->Points[2][a->Length - 2]);
                      }
                      cv::Point b_point = cv::Point(b->Points[1][1], b->Points[2][1]);
                      if (distance(b_point, junction_point) >
                          distance(cv::Point(b->Points[1][b->Length - 2], b->Points[2][b->Length - 2]), junction_point))
                      {
                          b_point = cv::Point(b->Points[1][b->Length - 2], b->Points[2][b->Length - 2]);
                      }
                      double a_cartesian = std::atan2(a_point.y - junction_point.y, a_point.x - junction_point.x);
                      double b_cartesian = std::atan2(b_point.y - junction_point.y, b_point.x - junction_point.x);
                      return a_cartesian < b_cartesian;
                  });
    }
}

void CurvesGenerator::generate_surfaces_from_junctions()
{
    for (const auto &junction_matcher : m_junction_to_curves)
    {
        if (junction_matcher.second.size() < 2)
        {
            continue;
        }
        if (m_marked_junctions.count(junction_matcher.first) > 0) // && m_marked_junctions[junction_matcher.first] > 1)
        {
            continue;
        }
        m_point_to_originating_curve.clear();
        std::vector<IritPoint> points = get_intersection_points(junction_matcher);
        if (points.size() == 3)
        {
            IritSurface surface = IritSurface(
                CagdBilinearSrf(points[0].get(), points[1].get(), points[2].get(), points[1].get(), CAGD_PT_E2_TYPE),
                CagdSrfFree);
            fix_surface_orientation(surface);
            if (surface != nullptr)
            {
                m_surfaces.push_back(std::move(surface));
            }
        }
        else if (points.size() == 4)
        {
            add_surface_from_4_points(points[0], points[1], points[2], points[3]);
        }
        else
        {
            const cv::Point &junction_point = m_graph[junction_matcher.first].p;
            std::sort(points.begin(), points.end(), [this, junction_matcher](const IritPoint &a, const IritPoint &b) {
                return compare_two_points_in_junction(a, b, junction_matcher);
            });
            IritPoint pivot = IritPoint(CagdPtNew(), CagdPtFree);
            pivot->Pt[0] = junction_point.x;
            pivot->Pt[1] = junction_point.y;
            for (int i = 0; i < points.size(); i++)
            {
                int j = (i + 1) % points.size();
                if (m_junctions_radius.count(junction_point) == 0)
                {
                    std::cerr << "ERROR: Failed to find junction radius" << junction_point << std::endl;
                    throw std::runtime_error("Failed to find junction radius");
                }
                IritSurface surface = generate_surface_from_pivot_and_points(pivot, points[i], points[j],
                                                                             m_junctions_radius[junction_point]);
                fix_surface_orientation(surface, false);
                if (surface != nullptr)
                {
                    m_surfaces.push_back(std::move(surface));
                }
                else
                {
                    surface = IritSurface(
                        CagdBilinearSrf(points[i].get(), pivot.get(), points[j].get(), pivot.get(), CAGD_PT_E2_TYPE),
                        CagdSrfFree);
                    fix_surface_orientation(surface);
                    if (surface != nullptr)
                    {
                        m_surfaces.push_back(std::move(surface));
                    }
                }
            }
        }
    }
}

void CurvesGenerator::find_neighborhoods_intersections()
{
    std::unordered_set<VertexDescriptor> passed_junctions;
    for (const auto &item : m_marked_junctions)
    {
        if (passed_junctions.count(item.first) > 0)
        {
            continue;
        }
        const auto &junction = item.first;
        std::vector<IritPoint> points;
        std::unordered_set<VertexDescriptor> neighborhood = get_marked_neighborhood(junction);
        std::vector<std::tuple<VertexDescriptor, CagdCrvStruct *, CagdCrvStruct *>> neighborhood_curves;
        std::vector<CagdCrvStruct *> neighborhood_offset_curves;
        for (const auto &neighbor : neighborhood)
        {
            assert(passed_junctions.count(neighbor) == 0);
            if (passed_junctions.count(neighbor) > 0)
            {
                std::cerr << "ERROR: Neighbor was already processed" << std::endl;
                throw std::runtime_error("Neighbor was already processed");
            }
            passed_junctions.insert(neighbor);
            for (const auto &curve : m_junction_to_curves[neighbor])
            {
                if (m_curve_to_offset_curves[curve].size() == 2)
                {
                    neighborhood_curves.push_back(std::make_tuple(neighbor, curve, m_curve_to_offset_curves[curve][0]));
                    neighborhood_curves.push_back(std::make_tuple(neighbor, curve, m_curve_to_offset_curves[curve][1]));
                    neighborhood_offset_curves.push_back(m_curve_to_offset_curves[curve][0]);
                    neighborhood_offset_curves.push_back(m_curve_to_offset_curves[curve][1]);
                }
            }
        }
        for (const auto &curve_item : neighborhood_curves)
        {
            VertexDescriptor junction = std::get<0>(curve_item);
            CagdCrvStruct *curve = std::get<1>(curve_item);
            CagdCrvStruct *offset_curve = std::get<2>(curve_item);
            double distance_0 =
                distance(cv::Point2d(curve->Points[1][0], curve->Points[2][0]), cv::Point2d(m_graph[junction].p));
            double distance_1 =
                distance(cv::Point2d(curve->Points[1][curve->Length - 1], curve->Points[2][curve->Length - 1]),
                         cv::Point2d(m_graph[junction].p));
            CagdRType choosen_t = (distance_0 <= distance_1) ? 0 : 1;
            bool t_is_tail = (choosen_t == 1);
            for (const auto &second_offset_curve : neighborhood_offset_curves)
            {
                if (offset_curve == second_offset_curve)
                {
                    continue;
                }
                CagdPtStruct *intersections =
                    CagdCrvCrvInter(offset_curve, second_offset_curve, INTERSECTION_PROXIMITY_THRESHOLD);
                for (CagdPtStruct *t = intersections; t != nullptr; t = t->Pnext)
                {
                    if (t->Pt[0] > 0 && t->Pt[0] < 1)
                    {
                        CagdRType sub1 = (t_is_tail) ? 1 - t->Pt[0] : t->Pt[0];
                        CagdRType sub2 = (t_is_tail) ? 1 - choosen_t : choosen_t;
                        if ((sub1 < 0.5 || (t_is_tail && sub1 == 0.5)) && sub1 > sub2)
                        {
                            choosen_t = t->Pt[0];
                        }
                    }
                }
            }
            if (m_offset_curve_subdivision_params.count(offset_curve) > 0 &&
                m_offset_curve_subdivision_params[offset_curve].count(junction) > 0)
            {

                CagdRType sub1 = (t_is_tail) ? 1 - choosen_t : choosen_t;
                CagdRType sub2 = (t_is_tail) ? 1 - m_offset_curve_subdivision_params[offset_curve][junction]
                                             : m_offset_curve_subdivision_params[offset_curve][junction];
                if ((sub1 < 0.5 || (t_is_tail && sub1 == 0.5)) && sub1 > sub2)
                {
                    m_offset_curve_subdivision_params[offset_curve][junction] = choosen_t;
                }
            }
            else
            {
                m_offset_curve_subdivision_params[offset_curve][junction] = choosen_t;
            }
        }
    }
}

void CurvesGenerator::generate_surfaces_from_curves()
{
    for (const auto &curve_info : m_curves)
    {
        const Curve &curve = std::get<0>(curve_info);
        VertexDescriptor junction1 = std::get<3>(curve_info);
        VertexDescriptor junction2 = std::get<4>(curve_info);
        if (m_curve_to_offset_curves.count(curve.get()) == 0 || m_curve_to_offset_curves[curve.get()].size() != 2)
        {
            continue;
        }
        CagdCrvStruct *offset_curve1 = m_curve_to_offset_curves[curve.get()][0];
        CagdCrvStruct *offset_curve2 = m_curve_to_offset_curves[curve.get()][1];
        CagdRType offset_curve1_junction1_subdiv = 0;
        CagdRType offset_curve2_junction1_subdiv = 0;
        CagdRType offset_curve1_junction2_subdiv = 1;
        CagdRType offset_curve2_junction2_subdiv = 1;
        if (m_offset_curve_subdivision_params.count(offset_curve1) > 0)
        {
            if (m_offset_curve_subdivision_params[offset_curve1].count(junction1) > 0)
            {
                offset_curve1_junction1_subdiv = m_offset_curve_subdivision_params[offset_curve1][junction1];
            }
            if (m_offset_curve_subdivision_params[offset_curve1].count(junction2) > 0)
            {
                offset_curve1_junction2_subdiv = m_offset_curve_subdivision_params[offset_curve1][junction2];
            }
        }
        if (m_offset_curve_subdivision_params.count(offset_curve2) > 0)
        {
            if (m_offset_curve_subdivision_params[offset_curve2].count(junction1) > 0)
            {
                offset_curve2_junction1_subdiv = m_offset_curve_subdivision_params[offset_curve2][junction1];
            }
            if (m_offset_curve_subdivision_params[offset_curve2].count(junction2) > 0)
            {
                offset_curve2_junction2_subdiv = m_offset_curve_subdivision_params[offset_curve2][junction2];
            }
        }
        int proximity;
        CagdRType params1[2] = {offset_curve1_junction1_subdiv, offset_curve1_junction2_subdiv};
        CagdRType params2[2] = {offset_curve2_junction1_subdiv, offset_curve2_junction2_subdiv};
        CagdCrvStruct *sliced_offset_curve1 = CagdCrvSubdivAtParams3(offset_curve1, params1, 2, 0, FALSE, &proximity);
        CagdCrvStruct *sliced_offset_curve2 = CagdCrvSubdivAtParams3(offset_curve2, params2, 2, 0, FALSE, &proximity);
        CagdCrvStruct *wanted_curve1 =
            (offset_curve1_junction1_subdiv > 0) ? sliced_offset_curve1->Pnext : sliced_offset_curve1;
        CagdCrvStruct *wanted_curve2 =
            (offset_curve2_junction1_subdiv > 0) ? sliced_offset_curve2->Pnext : sliced_offset_curve2;
        IritSurface surface = IritSurface(CagdRuledSrf(wanted_curve1, wanted_curve2, 2, 2), CagdSrfFree);
        fix_surface_orientation(surface);
        if (surface != nullptr)
        {
            m_surfaces.push_back(std::move(surface));
        }
        CagdCrvFreeList(sliced_offset_curve1);
        CagdCrvFreeList(sliced_offset_curve2);
    }
}

void CurvesGenerator::fill_holes()
{
    std::unordered_set<VertexDescriptor> passed_junctions;
    for (const auto &item : m_marked_junctions)
    {
        if (passed_junctions.count(item.first) > 0)
        {
            continue;
        }
        const auto &junction = item.first;
        std::unordered_set<VertexDescriptor> neighborhood = get_marked_neighborhood(junction);
        std::unordered_set<cv::Point> unique_points;
#ifdef DEBUG_NEIGHBORHOOD
        std::cout << "\t\tNeighborhood of junction " << m_graph[junction].p << " has " << neighborhood.size()
                  << " elements" << std::endl;
#endif
        for (const auto &neighbor : neighborhood)
        {
            assert(passed_junctions.count(neighbor) == 0);
            if (passed_junctions.count(neighbor) > 0)
            {
                std::cerr << "ERROR: Neighbor was already processed" << std::endl;
                throw std::runtime_error("Neighbor was already processed");
            }
            passed_junctions.insert(neighbor);
#ifdef DEBUG_NEIGHBORHOOD
            std::cout << "\t\t\tNeighbor: " << m_graph[neighbor].p << std::endl;
#endif
        }

        std::vector<CagdCrvStruct *> neighborhood_offset_curves;
        std::vector<IritPoint> points = get_neighborhood_points(neighborhood, neighborhood_offset_curves);

        if (points.size() == 0)
        {
            continue;
        }
#ifdef DEBUG_NEIGHBORHOOD
        std::cout << "\t\tFilling hole with " << points.size() << " points:" << std::endl;
        for (const auto &point : points)
        {
            std::cout << "\t\t\t(" << point.get()->Pt[0] << ", " << point.get()->Pt[1] << ")" << std::endl;
        }
#endif
        IPVertexStruct *first_vertex = nullptr;
        IPVertexStruct *last_vertex = nullptr;
        for (const auto &point : points)
        {
            IPVertexStruct *vertex = IPAllocVertex(0, nullptr, first_vertex);
            vertex->Coord[0] = point.get()->Pt[0];
            vertex->Coord[1] = point.get()->Pt[1];
            vertex->Coord[2] = 0;
            if (last_vertex == nullptr)
            {
                last_vertex = vertex;
            }
            first_vertex = vertex;
        }
        last_vertex->Pnext = first_vertex;
        IPPolygonStruct *polygon = IPAllocPolygon(0, first_vertex, nullptr);
        IPUpdatePolyPlane(polygon);
        IPPolygonStruct *polygons = GMSplitNonConvexPoly(polygon, FALSE);
        IPFreePolygon(polygon);
        for (IPPolygonStruct *current_polygon = polygons; current_polygon != nullptr;
             current_polygon = current_polygon->Pnext)
        {
            IritPoint pivot = IritPoint(CagdPtNew(), CagdPtFree);
            GMComputeAverageVertex(current_polygon->PVertex, pivot->Pt, 1);
            IPVertexStruct *first_vertex = current_polygon->PVertex;
            IPVertexStruct *prev_vertex = first_vertex;
            IPVertexStruct *current_vertex = first_vertex->Pnext;
            do
            {
                IritPoint prev_point = IritPoint(CagdPtNew(), CagdPtFree);
                prev_point.get()->Pt[0] = prev_vertex->Coord[0];
                prev_point.get()->Pt[1] = prev_vertex->Coord[1];
                prev_point.get()->Pt[2] = 0;
                IritPoint current_point = IritPoint(CagdPtNew(), CagdPtFree);
                current_point.get()->Pt[0] = current_vertex->Coord[0];
                current_point.get()->Pt[1] = current_vertex->Coord[1];
                current_point.get()->Pt[2] = 0;
                CagdCrvStruct *relative_curve;
                CagdRType t0;
                CagdRType t1;
                std::tie(relative_curve, t0, t1) =
                    get_points_matching_curve(neighborhood_offset_curves, prev_point, current_point);
                IritSurface surface = IritSurface(nullptr, CagdSrfFree);
                if (relative_curve != nullptr)
                {
                    if ((t0 < 0.01 || 1 - t0 < 0.01) && (t1 < 0.01 || 1 - t1 < 0.01))
                    {
                        prev_point->Pt[0] = relative_curve->Points[1][0];
                        prev_point->Pt[1] = relative_curve->Points[2][0];
                        current_point->Pt[0] = relative_curve->Points[1][relative_curve->Length - 1];
                        current_point->Pt[1] = relative_curve->Points[2][relative_curve->Length - 1];
                        Curve a = Curve(CagdMergePtPtLen(prev_point.get(), pivot.get(), 2), CagdCrvFree);
                        Curve b = Curve(CagdMergePtPtLen(pivot.get(), current_point.get(), 2), CagdCrvFree);
                        Curve pivot_curve = Curve(CagdMergeCrvCrv(a.get(), b.get(), TRUE, 0.5), CagdCrvFree);
                        surface.reset(CagdRuledSrf(relative_curve, pivot_curve.get(), 2, 2));
                    }
                    else
                    {
                        if (t0 > t1)
                        {
                            std::swap(t0, t1);
                        }
                        int proximity;
                        CagdRType params[2] = {t0, t1};
                        CagdCrvStruct *sliced_curve =
                            CagdCrvSubdivAtParams3(CagdCrvCopy(relative_curve), params, 0, 0, FALSE, &proximity);
                        CagdCrvStruct *wanted_curve = (t0 > 0 && t1 > 0) ? sliced_curve->Pnext : sliced_curve;
                        prev_point->Pt[0] = wanted_curve->Points[1][0];
                        prev_point->Pt[1] = wanted_curve->Points[2][0];
                        current_point->Pt[0] = wanted_curve->Points[1][wanted_curve->Length - 1];
                        current_point->Pt[1] = wanted_curve->Points[2][wanted_curve->Length - 1];
                        Curve a = Curve(CagdMergePtPtLen(prev_point.get(), pivot.get(), 2), CagdCrvFree);
                        Curve b = Curve(CagdMergePtPtLen(pivot.get(), current_point.get(), 2), CagdCrvFree);
                        Curve pivot_curve = Curve(CagdMergeCrvCrv(a.get(), b.get(), TRUE, 0.5), CagdCrvFree);
                        surface.reset(CagdRuledSrf(wanted_curve, pivot_curve.get(), 2, 2));
                        CagdCrvFreeList(sliced_curve);
                    }
                }
                else
                {
                    surface.reset(CagdBilinearSrf(prev_point.get(), pivot.get(), current_point.get(), pivot.get(),
                                                  CAGD_PT_E2_TYPE));
                }
                fix_surface_orientation(surface);
                if (surface != nullptr)
                {
                    m_surfaces.push_back(std::move(surface));
                }
                prev_vertex = current_vertex;
                current_vertex = current_vertex->Pnext;
            } while (prev_vertex != first_vertex);
        }
        IPFreePolygonList(polygons);
    }
}

void CurvesGenerator::extrude_surfaces()
{
    CagdVecStruct extrusion_vector = {0};
    extrusion_vector.Vec[0] = 0;
    extrusion_vector.Vec[1] = 0;
    extrusion_vector.Vec[2] = m_extrusion_amount;
    for (auto &surface : m_surfaces)
    {
        m_extrusions.push_back(IritTV(TrivExtrudeTV(surface.get(), &extrusion_vector), TrivTVFree));
    }
}

std::vector<std::pair<std::vector<VertexDescriptor>, std::vector<EdgeDescriptor>>> CurvesGenerator::split_junction_walk(
    const std::pair<std::vector<VertexDescriptor>, std::vector<EdgeDescriptor>> &junction_walk)
{
    if (junction_walk.first.size() < 4)
    {
        return {junction_walk};
    }
    VertexDescriptor prev = junction_walk.first[0];
    VertexDescriptor current = junction_walk.first[1];
    VertexDescriptor next = junction_walk.first[2];
    double prev_angle = angle_between(m_graph[prev].p, m_graph[current].p, m_graph[next].p);
    while (prev_angle < 0)
    {
        prev_angle += M_PI / 2;
    }
    while (prev_angle > M_PI / 2)
    {
        prev_angle -= M_PI / 2;
    }
    for (size_t i = 1; i < junction_walk.first.size() - 1; ++i)
    {
        VertexDescriptor prev = junction_walk.first[i - 1];
        VertexDescriptor current = junction_walk.first[i];
        VertexDescriptor next = junction_walk.first[i + 1];
        double current_angle = angle_between(m_graph[prev].p, m_graph[current].p, m_graph[next].p);
        if (false && std::abs(current_angle - prev_angle) > degrees_to_radians(70))
        {
            std::vector<VertexDescriptor> first_half(junction_walk.first.begin(), junction_walk.first.begin() + i + 1);
            std::vector<VertexDescriptor> second_half(junction_walk.first.begin() + i, junction_walk.first.end());
            std::vector<EdgeDescriptor> first_half_edges(junction_walk.second.begin(),
                                                         junction_walk.second.begin() + i);
            std::vector<EdgeDescriptor> second_half_edges(junction_walk.second.begin() + i - 1,
                                                          junction_walk.second.end());
            std::vector<std::pair<std::vector<VertexDescriptor>, std::vector<EdgeDescriptor>>> splitted =
                split_junction_walk({second_half, second_half_edges});
            splitted.insert(splitted.begin(), {first_half, first_half_edges});
            return splitted;
        }
    }
    return {junction_walk};
}

void CurvesGenerator::generate_curve(const std::vector<VertexDescriptor> &route,
                                     const std::vector<EdgeDescriptor> &route_edges)
{
    VertexDescriptor junction = route.front();
    VertexDescriptor junction2 = (route.back() != route.front()) ? route.back() : -1;

    // Add curve
    Curve curve =
        Curve(BspCrvNew(route.size(), (route.size() > m_max_order - 1) ? m_max_order : route.size(), CAGD_PT_E2_TYPE),
              CagdCrvFree);
    Curve width_curve =
        Curve(BspCrvNew(route.size(), (route.size() > m_max_order - 1) ? m_max_order : route.size(), CAGD_PT_E1_TYPE),
              CagdCrvFree);
    Curve opposite_width_curve =
        Curve(BspCrvNew(route.size(), (route.size() > m_max_order - 1) ? m_max_order : route.size(), CAGD_PT_E1_TYPE),
              CagdCrvFree);
    Curve height_curve =
        Curve(BspCrvNew(route.size(), (route.size() > m_max_order - 1) ? m_max_order : route.size(), CAGD_PT_E3_TYPE),
              CagdCrvFree);
    BspKnotUniformOpen(route.size(), (route.size() > m_max_order - 1) ? m_max_order : route.size(), curve->KnotVector);
    BspKnotUniformOpen(route.size(), (route.size() > m_max_order - 1) ? m_max_order : route.size(),
                       width_curve->KnotVector);
    BspKnotUniformOpen(route.size(), (route.size() > m_max_order - 1) ? m_max_order : route.size(),
                       opposite_width_curve->KnotVector);
    BspKnotUniformOpen(route.size(), (route.size() > m_max_order - 1) ? m_max_order : route.size(),
                       height_curve->KnotVector);
    for (int i = 0; i < route.size(); ++i)
    {
        double distance = m_graph[route[i]].distance_to_source;
        // if (i == 0)
        // if (i == 1 && route.size() > 3)
        // {
        //     distance = m_graph[route[i + 1]].distance_to_source;
        // }
        // if (i == route.size() - 1)
        // if (i == route.size() - 2 && route.size() > 3)
        // {
        //     distance = m_graph[route[i - 1]].distance_to_source;
        // }
        curve->Points[1][i] = m_graph[route[i]].p.x;
        curve->Points[2][i] = m_graph[route[i]].p.y;
        width_curve->Points[1][i] = distance;
        opposite_width_curve->Points[1][i] = -distance;

        height_curve->Points[1][i] = m_graph[route[i]].p.x;
        height_curve->Points[2][i] = m_graph[route[i]].p.y;
        height_curve->Points[3][i] = distance;
    }
    m_curves.push_back(
        {std::move(curve), std::move(width_curve), std::move(opposite_width_curve), junction, junction2});
    m_height_curves.push_back(std::move(height_curve));
}

std::vector<IritPoint> CurvesGenerator::get_intersection_points(
    const std::pair<const VertexDescriptor, std::vector<CagdCrvStruct *>> &junction_matcher)
{
    std::vector<IritPoint> points;
    std::unordered_map<CagdCrvStruct *, CagdRType> curve_to_subdiv;
    std::unordered_map<CagdCrvStruct *, CagdCrvStruct *> curve_to_curve;
    std::unordered_map<CagdCrvStruct *, CagdCrvStruct *> curve_to_originating_curve;
    for (size_t i = 0; i < junction_matcher.second.size(); ++i)
    {
        CagdCrvStruct *curve = junction_matcher.second[i];
        CagdCrvStruct *next_curve = junction_matcher.second[(i + 1) % junction_matcher.second.size()];
        bool found = false;
        CagdCrvStruct *choosen_curve = nullptr;
        CagdCrvStruct *choosen_next_curve = nullptr;
        CagdPtStruct *point = CagdPtNew();
        double distance_0 = distance(cv::Point2d(curve->Points[1][0], curve->Points[2][0]),
                                     cv::Point2d(m_graph[junction_matcher.first].p));
        double distance_1 =
            distance(cv::Point2d(curve->Points[1][curve->Length - 1], curve->Points[2][curve->Length - 1]),
                     cv::Point2d(m_graph[junction_matcher.first].p));
        CagdRType choosen_t1 = (distance_0 <= distance_1) ? 0 : 1;
        distance_0 = distance(cv::Point2d(next_curve->Points[1][0], next_curve->Points[2][0]),
                              cv::Point2d(m_graph[junction_matcher.first].p));
        distance_1 = distance(
            cv::Point2d(next_curve->Points[1][next_curve->Length - 1], next_curve->Points[2][next_curve->Length - 1]),
            cv::Point2d(m_graph[junction_matcher.first].p));
        CagdRType choosen_t2 = (distance_0 <= distance_1) ? 0 : 1;
        CagdPtFree(point);
        bool t1_is_tail = (choosen_t1 == 1);
        for (auto offset_curve : m_curve_to_offset_curves[curve])
        {
            for (auto next_offset_curve : m_curve_to_offset_curves[next_curve])
            {
                CagdPtStruct *intersections =
                    CagdCrvCrvInter(offset_curve, next_offset_curve, INTERSECTION_PROXIMITY_THRESHOLD);
                CagdPtStruct *intersections2 =
                    CagdCrvCrvInter(next_offset_curve, offset_curve, INTERSECTION_PROXIMITY_THRESHOLD);
                for (CagdPtStruct *t = intersections; t != nullptr; t = t->Pnext)
                {
                    assert(intersections2 != nullptr);
                    CagdPtStruct *point = CagdPtNew();
                    CAGD_CRV_EVAL_E2(offset_curve, t->Pt[0], &point->Pt[0]);
                    double intersection_distance_to_junction = distance(cv::Point2d(point->Pt[0], point->Pt[1]),
                                                                        cv::Point2d(m_graph[junction_matcher.first].p));
                    double curve_distance_to_junction_1 =
                        distance(cv::Point2d(curve->Points[1][0], curve->Points[2][0]),
                                 cv::Point2d(m_graph[junction_matcher.first].p));
                    double curve_distance_to_junction_2 =
                        distance(cv::Point2d(curve->Points[1][curve->Length - 1], curve->Points[2][curve->Length - 1]),
                                 cv::Point2d(m_graph[junction_matcher.first].p));
                    if (t->Pt[0] > 0 && t->Pt[0] < 1 &&
                        intersection_distance_to_junction <
                            (curve_distance_to_junction_1 + curve_distance_to_junction_2) * 0.75)
                    {
                        if (found)
                        {
                            // std::cerr << "Found more than one intersection in junction" << std::endl;
                        }
                        CagdRType sub1 = (t1_is_tail) ? 1 - t->Pt[0] : t->Pt[0];
                        CagdRType sub2 = (t1_is_tail) ? 1 - choosen_t1 : choosen_t1;
                        if ((sub1 < 0.5 || (t1_is_tail && sub1 == 0.5)) && sub1 > sub2)
                        {
                            if (curve_to_subdiv.count(offset_curve) > 0)
                            {
                                CagdRType sub3 =
                                    std::min(curve_to_subdiv[offset_curve], 1 - curve_to_subdiv[offset_curve]);
                                if (sub1 < sub3 || curve_to_curve[offset_curve] == next_offset_curve)
                                {
                                    continue;
                                }
                            }
                            CagdPtStruct *t2 = intersections2;
                            CagdPtStruct *point = CagdPtNew();
                            CagdPtStruct *best = CagdPtNew();
                            CagdPtStruct *reference = CagdPtNew();
                            CAGD_CRV_EVAL_E2(offset_curve, t->Pt[0], &point->Pt[0]);
                            CAGD_CRV_EVAL_E2(next_offset_curve, t2->Pt[0], &best->Pt[0]);
                            for (CagdPtStruct *search = intersections2->Pnext; search != nullptr;
                                 search = search->Pnext)
                            {
                                CAGD_CRV_EVAL_E2(next_offset_curve, search->Pt[0], &reference->Pt[0]);
                                if (distance(cv::Point2d(point->Pt[0], point->Pt[1]),
                                             cv::Point2d(reference->Pt[0], reference->Pt[1])) <
                                    distance(cv::Point2d(point->Pt[0], point->Pt[1]),
                                             cv::Point2d(best->Pt[0], best->Pt[1])))
                                {
                                    t2 = search;
                                }
                            }
                            CagdPtFree(point);
                            CagdPtFree(best);
                            CagdPtFree(reference);
                            choosen_curve = offset_curve;
                            choosen_next_curve = next_offset_curve;
                            choosen_t1 = t->Pt[0];
                            choosen_t2 = t2->Pt[0];
                            found = true;
                        }
                    }
                }
                CagdPtFreeList(intersections);
                CagdPtFreeList(intersections2);
            }
        }
        if (found)
        {
            curve_to_subdiv[choosen_curve] = choosen_t1;
            curve_to_subdiv[choosen_next_curve] = choosen_t2;
            curve_to_curve[choosen_curve] = choosen_next_curve;
            curve_to_curve[choosen_next_curve] = choosen_curve;
            curve_to_originating_curve[choosen_curve] = curve;
            curve_to_originating_curve[choosen_next_curve] = next_curve;
        }
    }
    std::unordered_set<CagdCrvStruct *> used;
    for (const auto &item : curve_to_subdiv)
    {
        if (used.count(item.first) > 0)
        {
            continue;
        }
        CagdCrvStruct *twin = curve_to_curve[item.first];
        CagdPtStruct *point = CagdPtNew();
        CAGD_CRV_EVAL_E2(item.first, item.second, &point->Pt[0]);
        m_offset_curve_subdivision_params[item.first][junction_matcher.first] = item.second;
        if (twin)
        {
            m_offset_curve_subdivision_params[twin][junction_matcher.first] = curve_to_subdiv[twin];
        }
        else
        {
            std::cerr << "No twin" << std::endl;
        }
        points.push_back(IritPoint(point, CagdPtFree));
        if (m_point_to_originating_curve.count(point) == 0)
        {
            m_point_to_originating_curve[point] = {};
        }
        m_point_to_originating_curve[point].push_back(
            std::make_tuple(curve_to_originating_curve[item.first], nullptr, 0));
        m_point_to_originating_curve[point].push_back(std::make_tuple(curve_to_originating_curve[twin], nullptr, 0));
        used.insert(item.first);
        if (twin)
        {
            used.insert(twin);
        }
    }
    for (const auto &curve : junction_matcher.second)
    {
        for (auto offset_curve : m_curve_to_offset_curves[curve])
        {
            if (used.count(offset_curve) == 0)
            {
                CagdPtStruct *point = CagdPtNew();
                if (std::abs(curve->Points[1][0] - m_graph[junction_matcher.first].p.x) < 1 &&
                    std::abs(curve->Points[2][0] - m_graph[junction_matcher.first].p.y) < 1)
                {
                    CAGD_CRV_EVAL_E2(offset_curve, 0, &point->Pt[0]);
                    m_offset_curve_subdivision_params[offset_curve][junction_matcher.first] = 0;
                }
                else
                {
                    CAGD_CRV_EVAL_E2(offset_curve, 1, &point->Pt[0]);
                    m_offset_curve_subdivision_params[offset_curve][junction_matcher.first] = 1;
                }
                points.push_back(IritPoint(point, CagdPtFree));
                if (m_point_to_originating_curve.count(point) == 0)
                {
                    m_point_to_originating_curve[point] = {};
                }
                else
                {
                    std::cerr << "Point already exists" << std::endl;
                }
                m_point_to_originating_curve[point].push_back(std::make_tuple(
                    curve, offset_curve, m_offset_curve_subdivision_params[offset_curve][junction_matcher.first]));
            }
        }
    }
    return points;
}

void CurvesGenerator::add_surface_from_4_points(const IritPoint &p0, const IritPoint &p1, const IritPoint &p2,
                                                const IritPoint &p3)
{
    std::vector<CagdPtStruct *> points = {p0.get(), p1.get(), p2.get(), p3.get()};
    do
    {
        IritSurface surface(CagdBilinearSrf(points[0], points[1], points[2], points[3], CAGD_PT_E2_TYPE), CagdSrfFree);
        fix_surface_orientation(surface, false);
        if (surface != nullptr)
        {
            m_surfaces.push_back(std::move(surface));
            return;
        }
    } while (std::next_permutation(points.begin(), points.end()));
    std::cerr << "Failed to create surface" << std::endl;
    return;
}

void CurvesGenerator::add_surface_from_2_lines(const IritPoint &line0_p0, const IritPoint &line0_p1,
                                               const IritPoint &line1_p0, const IritPoint &line1_p1)
{
    Curve curve1 = Curve(BzrCrvNew(2, CAGD_PT_E2_TYPE), CagdCrvFree);
    curve1->Points[1][0] = line0_p0->Pt[0];
    curve1->Points[2][0] = line0_p0->Pt[1];
    curve1->Points[1][1] = line1_p0->Pt[0];
    curve1->Points[2][1] = line1_p0->Pt[1];
    Curve curve2 = Curve(BzrCrvNew(2, CAGD_PT_E2_TYPE), CagdCrvFree);
    curve2->Points[1][0] = line0_p1->Pt[0];
    curve2->Points[2][0] = line0_p1->Pt[1];
    curve2->Points[1][1] = line1_p1->Pt[0];
    curve2->Points[2][1] = line1_p1->Pt[1];
    CagdPtStruct *intersections = CagdCrvCrvInter(curve1.get(), curve2.get(), INTERSECTION_PROXIMITY_THRESHOLD);
    if (intersections == nullptr)
    {
        IritSurface surface(
            CagdBilinearSrf(line0_p0.get(), line0_p1.get(), line1_p0.get(), line1_p1.get(), CAGD_PT_E2_TYPE),
            CagdSrfFree);
        fix_surface_orientation(surface);
        if (surface != nullptr)
        {
            m_surfaces.push_back(std::move(surface));
        }
    }
    else
    {
        IritSurface surface(
            CagdBilinearSrf(line0_p0.get(), line0_p1.get(), line1_p1.get(), line1_p0.get(), CAGD_PT_E2_TYPE),
            CagdSrfFree);
        fix_surface_orientation(surface);
        if (surface != nullptr)
        {
            m_surfaces.push_back(std::move(surface));
        }
    }
    CagdPtFreeList(intersections);
}

int CurvesGenerator::distance_to_boundary(const cv::Point &point, int maximum_distance)
{
    for (int radius = 0; radius < maximum_distance; radius++)
    {
        for (int i = std::max(point.x - radius, 0); i <= std::min(point.x + radius, m_reference_image.cols - 1); i++)
        {
            for (int j = std::max(point.y - radius, 0); j <= std::min(point.y + radius, m_reference_image.rows - 1);
                 j++)
            {
                if (m_reference_image.at<cv::Vec3b>({i, j})[2] < 120)
                {
                    return radius;
                }
            }
        }
    }

    return maximum_distance;
}

cv::Point CurvesGenerator::closest_point_on_boundary(const cv::Point &point, int maximum_distance)
{
    for (int radius = 0; radius < maximum_distance; radius++)
    {
        for (int i = std::max(point.x - radius, 0); i <= std::min(point.x + radius, m_reference_image.cols - 1); i++)
        {
            for (int j = std::max(point.y - radius, 0); j <= std::min(point.y + radius, m_reference_image.rows - 1);
                 j++)
            {
                if (m_reference_image.at<cv::Vec3b>({i, j})[2] < 120)
                {
                    return {i, j};
                }
            }
        }
    }

    return {-1, -1};
}

int CurvesGenerator::distance_in_boundary(const cv::Point &p0, const cv::Point &p1)
{
    if (p0 == p1)
    {
        return 0;
    }

    auto source = point_to_index(p0, m_reference_image.cols);
    auto target = point_to_index(p1, m_reference_image.cols);

    std::vector<int> distances(boost::num_vertices(m_image_graph));

    std::vector<VertexDescriptor> p(boost::num_vertices(m_image_graph));
    std::vector<double> d(boost::num_vertices(m_image_graph));

    boost::dijkstra_shortest_paths(m_image_graph, source, boost::predecessor_map(&p[0]).distance_map(&d[0]));

    return d[target] != 0 ? d[target] : std::numeric_limits<int>::max();
}

Curve CurvesGenerator::trim_curve_to_fit_boundary(const Curve &curve, const Curve &width_curve,
                                                  const Curve &curve_to_trim)
{
    double start_point = -1;
    double end_point = -1;
    cv::Point2d junction0(curve->Points[1][0], curve->Points[2][0]);
    double junction0_radius = std::abs(width_curve->Points[1][0]) + m_junction_radius_adder;
    cv::Point2d junction1(curve->Points[1][curve->Length - 1], curve->Points[2][curve->Length - 1]);
    double junction1_radius = std::abs(width_curve->Points[1][width_curve->Length - 1]) + m_junction_radius_adder;

    m_junction_radius_lock.lock();
    if (m_junctions_radius.count(junction0) == 0)
    {
        m_junctions_radius[junction0] = junction0_radius;
    }
    else
    {
        m_junctions_radius[junction0] = std::min(m_junctions_radius[junction0], junction0_radius);
    }

    if (m_junctions_radius.count(junction1) == 0)
    {
        m_junctions_radius[junction1] = junction1_radius;
    }
    else
    {
        m_junctions_radius[junction1] = std::min(m_junctions_radius[junction1], junction1_radius);
    }
    m_junction_radius_lock.unlock();

    double curve_length = CagdCrvArcLenPoly(curve.get());
    for (double i = 0; i < 1; i += 1 / curve_length)
    {
        CagdPtStruct *point = CagdPtNew();
        CAGD_CRV_EVAL_E2(curve.get(), i, &point->Pt[0]);
        cv::Point2d p(point->Pt[0], point->Pt[1]);
        CagdPtFree(point);

        // if (distance_to_boundary(p, m_distance_to_boundary_threshold) == m_distance_to_boundary_threshold)
        // {
        //     start_point = (i > 0) ? i + 0.05 : 0;
        //     break;
        // }
        if (distance(p, junction0) > junction0_radius)
        {
            start_point = i;
            break;
        }
    }
    for (double i = 1; i > 0; i -= 1 / curve_length)
    {
        CagdPtStruct *point = CagdPtNew();
        CAGD_CRV_EVAL_E2(curve.get(), i, &point->Pt[0]);
        cv::Point2d p(point->Pt[0], point->Pt[1]);

        // if (distance_to_boundary(p, m_distance_to_boundary_threshold) == m_distance_to_boundary_threshold)
        // {
        //     end_point = (i < 1) ? i - 0.05 : 1;
        //     break;
        if (distance(p, junction1) > junction1_radius)
        {
            end_point = i;
            break;
        }
    }
    if (start_point == -1 || end_point == -1 || start_point >= end_point)
    {
        return Curve(nullptr, CagdCrvFree);
    }
    int proximity;
    CagdRType params[2] = {start_point, end_point};
    CagdCrvStruct *sliced_curved = CagdCrvSubdivAtParams3(curve_to_trim.get(), params, 2, 0, FALSE, &proximity);
    CagdCrvStruct *wanted_curve = (start_point > 0) ? sliced_curved->Pnext : sliced_curved;
    wanted_curve->Pnext = nullptr;
    CagdCrvSetDomain(wanted_curve, 0, 1);

    // CagdCrvStruct *CrvTemp;
    // while (sliced_curved)
    // {
    //     CrvTemp = sliced_curved->Pnext;
    //     if (CrvTemp != wanted_curve && CrvTemp != curve.get())
    //     {
    //         CagdCrvFree(sliced_curved);
    //     }
    //     sliced_curved = CrvTemp;
    // }
    return Curve(wanted_curve, CagdCrvFree);
}

void CurvesGenerator::fix_offset_curves_surface_self_intersection(Curve &offset_curve, Curve &opposite_offset_curve)
{
    if (offset_curve == nullptr || opposite_offset_curve == nullptr)
    {
        return;
    }
    IritPoint offset_curve_p0 = IritPoint(CagdPtNew(), CagdPtFree);
    IritPoint offset_curve_p1 = IritPoint(CagdPtNew(), CagdPtFree);
    IritPoint opposite_offset_curve_p0 = IritPoint(CagdPtNew(), CagdPtFree);
    IritPoint opposite_offset_curve_p1 = IritPoint(CagdPtNew(), CagdPtFree);
    CAGD_CRV_EVAL_E2(offset_curve.get(), 0, &offset_curve_p0.get()->Pt[0]);
    CAGD_CRV_EVAL_E2(offset_curve.get(), 1, &offset_curve_p1.get()->Pt[0]);
    CAGD_CRV_EVAL_E2(opposite_offset_curve.get(), 0, &opposite_offset_curve_p0.get()->Pt[0]);
    CAGD_CRV_EVAL_E2(opposite_offset_curve.get(), 1, &opposite_offset_curve_p1.get()->Pt[0]);

    Curve connection_0_curve = Curve(BzrCrvNew(2, CAGD_PT_E2_TYPE), CagdCrvFree);
    connection_0_curve->Points[1][0] = offset_curve_p0->Pt[0];
    connection_0_curve->Points[2][0] = offset_curve_p0->Pt[1];
    connection_0_curve->Points[1][1] = opposite_offset_curve_p0->Pt[0];
    connection_0_curve->Points[2][1] = opposite_offset_curve_p0->Pt[1];
    Curve connection_1_curve = Curve(BzrCrvNew(2, CAGD_PT_E2_TYPE), CagdCrvFree);
    connection_1_curve->Points[1][0] = offset_curve_p1->Pt[0];
    connection_1_curve->Points[2][0] = offset_curve_p1->Pt[1];
    connection_1_curve->Points[1][1] = opposite_offset_curve_p1->Pt[0];
    connection_1_curve->Points[2][1] = opposite_offset_curve_p1->Pt[1];
    CagdPtStruct *intersections =
        CagdCrvCrvInter(connection_0_curve.get(), connection_1_curve.get(), INTERSECTION_PROXIMITY_THRESHOLD);
    if (intersections == nullptr)
    {
        fix_offset_curves_surface_self_intersection(offset_curve, connection_0_curve, connection_1_curve);
        fix_offset_curves_surface_self_intersection(opposite_offset_curve, connection_0_curve, connection_1_curve);
    }
    else
    {
        CagdPtFreeList(intersections);
        connection_0_curve = Curve(BzrCrvNew(2, CAGD_PT_E2_TYPE), CagdCrvFree);
        connection_0_curve->Points[1][0] = offset_curve_p0->Pt[0];
        connection_0_curve->Points[2][0] = offset_curve_p0->Pt[1];
        connection_0_curve->Points[1][1] = opposite_offset_curve_p1->Pt[0];
        connection_0_curve->Points[2][1] = opposite_offset_curve_p1->Pt[1];
        connection_1_curve = Curve(BzrCrvNew(2, CAGD_PT_E2_TYPE), CagdCrvFree);
        connection_1_curve->Points[1][0] = offset_curve_p1->Pt[0];
        connection_1_curve->Points[2][0] = offset_curve_p1->Pt[1];
        connection_1_curve->Points[1][1] = opposite_offset_curve_p0->Pt[0];
        connection_1_curve->Points[2][1] = opposite_offset_curve_p0->Pt[1];
        intersections =
            CagdCrvCrvInter(connection_0_curve.get(), connection_1_curve.get(), INTERSECTION_PROXIMITY_THRESHOLD);
        if (intersections == nullptr)
        {
            fix_offset_curves_surface_self_intersection(offset_curve, connection_0_curve, connection_1_curve);
            fix_offset_curves_surface_self_intersection(opposite_offset_curve, connection_1_curve, connection_0_curve);
        }
        else
        {
            std::cerr << "Error: Offset curves are self intersecting" << std::endl;
            throw std::runtime_error("Offset curves are self intersecting");
        }
    }
}

void CurvesGenerator::fix_offset_curves_surface_self_intersection(Curve &offset_curve, Curve &connection_0_curve,
                                                                  Curve &connection_1_curve)
{
    CagdRType tail0 = 0;
    CagdRType tail1 = 1;
    CagdPtStruct *intersections =
        CagdCrvCrvInter(offset_curve.get(), connection_0_curve.get(), INTERSECTION_PROXIMITY_THRESHOLD);
    if (intersections != nullptr)
    {
        for (CagdPtStruct *t = intersections; t != nullptr; t = t->Pnext)
        {
            tail0 = std::max(t->Pt[0], tail0);
        }
        CagdPtFreeList(intersections);
    }
    intersections = CagdCrvCrvInter(offset_curve.get(), connection_1_curve.get(), INTERSECTION_PROXIMITY_THRESHOLD);
    if (intersections != nullptr)
    {
        for (CagdPtStruct *t = intersections; t != nullptr; t = t->Pnext)
        {
            tail1 = std::min(t->Pt[0], tail1);
        }

        CagdPtFreeList(intersections);
    }
    if (tail0 < 1e-10)
    {
        tail0 = 0;
    }
    if (tail1 > 1 - 1e-10)
    {
        tail1 = 1;
    }
    CagdRType params[2] = {tail0, tail1};
    int proximity;
    CagdCrvStruct *sliced_curved = CagdCrvSubdivAtParams3(offset_curve.get(), params, 2, 0, FALSE, &proximity);
    CagdCrvStruct *wanted_curve = (tail0 > 0) ? sliced_curved->Pnext : sliced_curved;
    sliced_curved->Pnext = nullptr;
    wanted_curve->Pnext = nullptr;
    if (tail0 > 0)
    {
        CagdCrvFree(sliced_curved);
    }
    else
    {
        CagdCrvFree(wanted_curve->Pnext);
    }
    CagdCrvSetDomain(wanted_curve, 0, 1);
    offset_curve.reset(wanted_curve);
}

void CurvesGenerator::fix_surface_orientation(IritSurface &surface, bool print_error)
{
    if (surface.get() == nullptr)
    {
        return;
    }
    double min, max;
    CagdGetPlnrSrfJacobianMinMax(surface.get(), &min, &max, TRUE);
    if (min * max < 0)
    {
        if (print_error)
        {
            std::cerr << "Warning: surface is self intersecting: min: " << min << ", max: " << max << "Point0: ("
                      << surface->Points[1][0] << ", " << surface->Points[2][0] << ")" << std::endl;
        }
        // throw std::runtime_error("Surface is self intersecting");
        surface.reset();
        return;
    }
    if (max <= 0)
    {
        surface.reset(CagdSrfReverse2(surface.get()));
    }
}

IritSurface CurvesGenerator::generate_surface_from_pivot_and_points(const IritPoint &pivot, const IritPoint &p0,
                                                                    const IritPoint &p1, double radius)
{
    if (m_point_to_originating_curve.count(p0.get()) == 0 || m_point_to_originating_curve.count(p1.get()) == 0 ||
        m_point_to_originating_curve[p0.get()].size() > 1 || m_point_to_originating_curve[p1.get()].size() > 1 ||
        std::get<0>(m_point_to_originating_curve[p0.get()][0]) ==
            std::get<0>(m_point_to_originating_curve[p1.get()][0]))
    {
        return IritSurface(CagdBilinearSrf(p0.get(), pivot.get(), p1.get(), pivot.get(), CAGD_PT_E2_TYPE), CagdSrfFree);
    }
    else
    {
        auto &item = m_point_to_originating_curve[p0.get()][0];
        Curve a = Curve(CagdMergePtPtLen(p0.get(), pivot.get(), 2), CagdCrvFree);
        Curve b = Curve(CagdMergePtPtLen(pivot.get(), p1.get(), 2), CagdCrvFree);
        Curve curve1 = Curve(CagdMergeCrvCrv(a.get(), b.get(), TRUE, 0.5), CagdCrvFree);
        Curve curve2 = Curve(BzrCrvNew(4, CAGD_PT_E2_TYPE), CagdCrvFree);

        curve2->Points[1][0] = p0->Pt[0];
        curve2->Points[2][0] = p0->Pt[1];

        CagdRType t = std::get<2>(item) - 0.1;
        if (std::get<2>(item) < 0.5)
        {
            t = std::get<2>(item) + 0.1;
        }
        CagdPtStruct *point = CagdPtNew();
        CAGD_CRV_EVAL_E2(std::get<1>(item), t, &point->Pt[0]);
        cv::Point2d line_p0 = cv::Point2d(point->Pt[0], point->Pt[1]);
        cv::Point2d line_p1 = cv::Point2d(p0->Pt[0], p0->Pt[1]);
        double diff_x = line_p1.x - line_p0.x;
        double diff_y = line_p1.y - line_p0.y;
        // Normalize diff
        double norm = std::sqrt(diff_x * diff_x + diff_y * diff_y);
        if (norm == 0)
        {
            return IritSurface(nullptr, CagdSrfFree);
        }
        diff_x /= norm;
        diff_y /= norm;
        // Get point which is the continuation of the line
        curve2->Points[1][1] = line_p1.x + diff_x * radius * 0.1;
        curve2->Points[2][1] = line_p1.y + diff_y * radius * 0.1;

        auto &item2 = m_point_to_originating_curve[p1.get()][0];
        t = std::get<2>(item2) - 0.1;
        if (std::get<2>(item2) < 0.5)
        {
            t = std::get<2>(item2) + 0.1;
        }
        CAGD_CRV_EVAL_E2(std::get<1>(item2), t, &point->Pt[0]);
        line_p0 = cv::Point2d(point->Pt[0], point->Pt[1]);
        line_p1 = cv::Point2d(p1->Pt[0], p1->Pt[1]);
        diff_x = line_p1.x - line_p0.x;
        diff_y = line_p1.y - line_p0.y;
        // Normalize diff
        norm = std::sqrt(diff_x * diff_x + diff_y * diff_y);
        if (norm <= 0)
        {
            return IritSurface(nullptr, CagdSrfFree);
        }
        diff_x /= norm;
        diff_y /= norm;
        // Get point which is the continuation of the line
        curve2->Points[1][2] = line_p1.x + diff_x * radius * 0.1;
        curve2->Points[2][2] = line_p1.y + diff_y * radius * 0.1;
        CagdPtFree(point);

        curve2->Points[1][3] = p1->Pt[0];
        curve2->Points[2][3] = p1->Pt[1];

        return IritSurface(CagdRuledSrf(curve1.get(), curve2.get(), 2, 2), CagdSrfFree);
    }
}

std::unordered_set<VertexDescriptor> CurvesGenerator::get_marked_neighborhood(const VertexDescriptor &junction)
{
    std::unordered_set<VertexDescriptor> visited;
    return get_marked_neighborhood(junction, visited);
}

std::unordered_set<VertexDescriptor> CurvesGenerator::get_marked_neighborhood(
    const VertexDescriptor &junction, std::unordered_set<VertexDescriptor> &visited)
{
    if (visited.count(junction) > 0)
    {
        return std::unordered_set<VertexDescriptor>();
    }
    std::unordered_set<VertexDescriptor> neighborhood;
    neighborhood.insert(junction);
    visited.insert(junction);
    auto out_edges = boost::out_edges(junction, m_graph);
    for (auto it = out_edges.first; it != out_edges.second; ++it)
    {
        std::vector<VertexDescriptor> route;
        std::tie(std::ignore, route, std::ignore) =
            ProcessGraph::walk_to_next_junction(junction, boost::target(*it, m_graph), m_graph);
        if (m_marked_junctions.count(route.back()) > 0 && m_connections.count({junction, route.back()}) == 0)
        {
            auto marked_neighborhood = get_marked_neighborhood(route.back(), visited);
            neighborhood.insert(marked_neighborhood.begin(), marked_neighborhood.end());
        }
    }
    return neighborhood;
}

bool CurvesGenerator::compare_two_points_in_junction(
    const IritPoint &a, const IritPoint &b,
    const std::pair<const VertexDescriptor, std::vector<CagdCrvStruct *>> &junction_matcher)
{
    assert(m_point_to_originating_curve.count(a.get()) > 0);
    assert(m_point_to_originating_curve.count(b.get()) > 0);
    const cv::Point &junction_point = m_graph[junction_matcher.first].p;
    CagdCrvStruct *a_curve = nullptr;
    auto prev_iter = junction_matcher.second.end();
    for (const auto &item : m_point_to_originating_curve[a.get()])
    {
        auto current_iter =
            std::find(junction_matcher.second.begin(), junction_matcher.second.end(), std::get<0>(item));
        if (std::distance(current_iter, prev_iter) > 0)
        {
            a_curve = std::get<0>(item);
            prev_iter = current_iter;
        }
    }
    if (a_curve == nullptr)
    {
        std::cerr << "Could not find curve for point " << a << std::endl;
        throw std::runtime_error("Could not find curve for point");
    }
    CagdCrvStruct *b_curve = nullptr;
    prev_iter = junction_matcher.second.end();
    for (const auto &item : m_point_to_originating_curve[b.get()])
    {
        auto current_iter =
            std::find(junction_matcher.second.begin(), junction_matcher.second.end(), std::get<0>(item));
        if (std::distance(current_iter, prev_iter) > 0)
        {
            b_curve = std::get<0>(item);
            prev_iter = current_iter;
        }
    }
    if (b_curve == nullptr)
    {
        std::cerr << "Could not find curve for point " << b << std::endl;
        throw std::runtime_error("Could not find curve for point");
    }

    if (a_curve == b_curve)
    {
        double a_cartesian = std::atan2(a->Pt[1] - junction_point.y, a->Pt[0] - junction_point.x);
        double b_cartesian = std::atan2(b->Pt[1] - junction_point.y, b->Pt[0] - junction_point.x);
        if (a_cartesian > M_PI / 2 && b_cartesian < -M_PI / 2)
        {
            return true;
        }
        if (a_cartesian < -M_PI / 2 && b_cartesian > M_PI / 2)
        {
            return false;
        }
        return a_cartesian < b_cartesian;
    }

    auto a_iter = std::find(junction_matcher.second.begin(), junction_matcher.second.end(), a_curve);
    auto b_iter = std::find(junction_matcher.second.begin(), junction_matcher.second.end(), b_curve);
    if (std::distance(a_iter, b_iter) == 0)
    {
        std::cerr << "ERROR: Failed to find curve order" << std::endl;
        throw std::runtime_error("Failed to find curve order");
    }
    return std::distance(a_iter, b_iter) > 0;
}

bool CurvesGenerator::compare_two_curves_in_junction(
    const CagdCrvStruct *a, const CagdCrvStruct *b,
    const std::pair<const VertexDescriptor, std::vector<CagdCrvStruct *>> &junction_matcher)
{
    auto a_iter = std::find(junction_matcher.second.begin(), junction_matcher.second.end(), a);
    auto b_iter = std::find(junction_matcher.second.begin(), junction_matcher.second.end(), b);
    return std::distance(a_iter, b_iter) > 0;
}

bool CurvesGenerator::compare_point_and_curve_in_junction(
    const IritPoint &a, const CagdCrvStruct *b,
    const std::pair<const VertexDescriptor, std::vector<CagdCrvStruct *>> &junction_matcher)
{
    assert(m_point_to_originating_curve.count(a.get()) > 0);
    CagdCrvStruct *a_curve = nullptr;
    auto prev_iter = junction_matcher.second.end();
    for (const auto &item : m_point_to_originating_curve[a.get()])
    {
        auto current_iter =
            std::find(junction_matcher.second.begin(), junction_matcher.second.end(), std::get<0>(item));
        if (std::distance(current_iter, prev_iter) > 0)
        {
            a_curve = std::get<0>(item);
            prev_iter = current_iter;
        }
    }
    if (a_curve == nullptr)
    {
        std::cerr << "Could not find curve for point " << a << std::endl;
        throw std::runtime_error("Could not find curve for point");
    }
    if (a_curve == b)
    {
        cv::Point2d junction_point = m_graph[junction_matcher.first].p;
        double a_cartesian = std::atan2(a->Pt[1] - junction_point.y, a->Pt[0] - junction_point.x);

        cv::Point2d b_point = cv::Point2d(b->Points[1][1], b->Points[2][1]);
        if (distance(b_point, junction_point) >
            distance(cv::Point2d(b->Points[1][b->Length - 2], b->Points[2][b->Length - 2]), junction_point))
        {
            b_point = cv::Point2d(b->Points[1][b->Length - 2], b->Points[2][b->Length - 2]);
        }
        double b_cartesian = std::atan2(b_point.y - junction_point.y, b_point.x - junction_point.x);
        if (a_cartesian > M_PI / 2 && b_cartesian < -M_PI / 2)
        {
            return true;
        }
        if (a_cartesian < -M_PI / 2 && b_cartesian > M_PI / 2)
        {
            return false;
        }
        return a_cartesian < b_cartesian;
    }
    auto a_iter = std::find(junction_matcher.second.begin(), junction_matcher.second.end(), a_curve);
    auto b_iter = std::find(junction_matcher.second.begin(), junction_matcher.second.end(), b);
    return std::distance(a_iter, b_iter) > 0;
}

CagdRType CurvesGenerator::get_offset_curve_subdivision_param(CagdCrvStruct *offset_curve, VertexDescriptor junction)
{
    if (m_offset_curve_subdivision_params.count(offset_curve) > 0 &&
        m_offset_curve_subdivision_params[offset_curve].count(junction) > 0)
    {
        return m_offset_curve_subdivision_params[offset_curve][junction];
    }
    IritPoint p0 = IritPoint(CagdPtNew(), CagdPtFree);
    IritPoint p1 = IritPoint(CagdPtNew(), CagdPtFree);
    CAGD_CRV_EVAL_E2(offset_curve, 0, &p0.get()->Pt[0]);
    CAGD_CRV_EVAL_E2(offset_curve, 1, &p1.get()->Pt[0]);
    return (distance(cv::Point2d(p0.get()->Pt[0], p0.get()->Pt[1]), cv::Point2d(m_graph[junction].p)) >
            distance(cv::Point2d(p1.get()->Pt[0], p1.get()->Pt[1]), cv::Point2d(m_graph[junction].p)))
               ? 1
               : 0;
}

std::vector<IritPoint> CurvesGenerator::get_neighborhood_points(
    const std::unordered_set<VertexDescriptor> &neighborhood, std::vector<CagdCrvStruct *> &neighborhood_offset_curves)
{
    std::unordered_map<VertexDescriptor, std::vector<std::variant<IritPoint, CagdCrvStruct *>>> neighborhood_points;
    m_point_to_originating_curve.clear();
    for (const auto &junction : neighborhood)
    {
        std::vector<std::variant<IritPoint, CagdCrvStruct *>> junction_reference;
        std::unordered_map<cv::Point, CagdPtStruct *> unique_points;
        for (const auto &curve : m_junction_to_curves[junction])
        {
            if (m_curve_to_offset_curves.count(curve) == 0 || m_curve_to_offset_curves[curve].empty())
            {
                junction_reference.push_back(curve);
            }
            else
            {
                for (const auto &offset_curve : m_curve_to_offset_curves[curve])
                {
                    neighborhood_offset_curves.push_back(offset_curve);
                    IritPoint p = IritPoint(CagdPtNew(), CagdPtFree);
                    CAGD_CRV_EVAL_E2(offset_curve, get_offset_curve_subdivision_param(offset_curve, junction),
                                     &p.get()->Pt[0]);
                    if (unique_points.count(cv::Point(p.get()->Pt[0], p.get()->Pt[1])) == 0)
                    {
                        unique_points[cv::Point(p.get()->Pt[0], p.get()->Pt[1])] = p.get();
                        m_point_to_originating_curve[p.get()] = {std::make_tuple(
                            curve, offset_curve, get_offset_curve_subdivision_param(offset_curve, junction))};
                        junction_reference.push_back(std::move(p));
                    }
                    else
                    {
                        CagdPtStruct *point = unique_points[cv::Point(p.get()->Pt[0], p.get()->Pt[1])];
                        m_point_to_originating_curve[point].push_back(std::make_tuple(
                            curve, offset_curve, get_offset_curve_subdivision_param(offset_curve, junction)));
                    }
                }
            }
        }
        std::sort(junction_reference.begin(), junction_reference.end(), [this, junction](const auto &a, const auto &b) {
            if (std::holds_alternative<IritPoint>(a) && std::holds_alternative<IritPoint>(b))
            {
                return compare_two_points_in_junction(std::get<IritPoint>(a), std::get<IritPoint>(b),
                                                      *m_junction_to_curves.find(junction));
            }
            if (std::holds_alternative<IritPoint>(a) && std::holds_alternative<CagdCrvStruct *>(b))
            {
                return compare_point_and_curve_in_junction(std::get<IritPoint>(a), std::get<CagdCrvStruct *>(b),
                                                           *m_junction_to_curves.find(junction));
            }
            if (std::holds_alternative<CagdCrvStruct *>(a) && std::holds_alternative<IritPoint>(b))
            {
                return !compare_point_and_curve_in_junction(std::get<IritPoint>(b), std::get<CagdCrvStruct *>(a),
                                                            *m_junction_to_curves.find(junction));
            }
            if (std::holds_alternative<CagdCrvStruct *>(a) && std::holds_alternative<CagdCrvStruct *>(b))
            {
                return compare_two_curves_in_junction(std::get<CagdCrvStruct *>(a), std::get<CagdCrvStruct *>(b),
                                                      *m_junction_to_curves.find(junction));
            }
            std::cerr << "ERROR: Invalid variant type" << std::endl;
            throw std::runtime_error("Invalid variant type");
        });
        neighborhood_points[junction] = std::move(junction_reference);
#ifdef DEBUG_NEIGHBORHOOD_JUNCTION_SORT
        std::cout << "\t\t\tJunction " << m_graph[junction].p << " order:" << std::endl;
        for (const auto &point : neighborhood_points[junction])
        {
            if (std::holds_alternative<IritPoint>(point))
            {
                std::cout << "\t\t\t\tPoint: " << std::get<IritPoint>(point).get()->Pt[0] << ", "
                          << std::get<IritPoint>(point).get()->Pt[1] << std::endl;
            }
            else
            {
                CagdCrvStruct *curve = std::get<CagdCrvStruct *>(point);
                std::cout << "\t\t\t\tCurve: " << curve->Points[1][0] << ", " << curve->Points[2][0] << " - "
                          << curve->Points[1][curve->Length - 1] << ", " << curve->Points[2][curve->Length - 1]
                          << std::endl;
            }
        }
#endif // DEBUG_NEIGHBORHOOD_JUNCTION_SORT
    }

    std::vector<IritPoint> neighborhood_points_vector;
    const std::variant<IritPoint, CagdCrvStruct *> *first_point = nullptr;
    VertexDescriptor first_junction;
    for (const auto &junction : neighborhood)
    {
        for (const auto &point : neighborhood_points[junction])
        {
            if (std::holds_alternative<IritPoint>(point))
            {
                first_point = &point;
                first_junction = junction;
            }
        }
    }
    if (first_point == nullptr)
    {
        std::cerr << "ERROR: No points in neighborhood" << std::endl;
        return std::vector<IritPoint>();
    }
    neighborhood_points_vector.push_back(IritPoint(CagdPtCopy(std::get<IritPoint>(*first_point).get()), CagdPtFree));
    auto first_iter =
        std::find(neighborhood_points[first_junction].begin(), neighborhood_points[first_junction].end(), *first_point);
    assert(first_iter != neighborhood_points[first_junction].end());
    VertexDescriptor current_junction = first_junction;
    auto current_iter = ++first_iter;
    const std::variant<IritPoint, CagdCrvStruct *> *last_point = first_point;
    std::unordered_set<const std::variant<IritPoint, CagdCrvStruct *> *> visited_points = {first_point};

#ifdef DEBUG_NEIGHBORHOOD_SORT
    std::cout << "\t\tStart of sorting neighborhood points" << std::endl;
    std::cout << "\t\t\tPoint: " << std::get<IritPoint>(*first_point).get()->Pt[0] << ", "
              << std::get<IritPoint>(*first_point).get()->Pt[1] << std::endl;
#endif // DEBUG_NEIGHBORHOOD_SORT

    if (current_iter == neighborhood_points[current_junction].end())
    {
        current_iter = neighborhood_points[current_junction].begin();
    }
    do
    {
        if (std::holds_alternative<IritPoint>(*current_iter))
        {
            if (&(*current_iter) != last_point)
            {
#ifdef DEBUG_NEIGHBORHOOD_SORT
                std::cout << "\t\t\tPoint: " << std::get<IritPoint>(*current_iter).get()->Pt[0] << ", "
                          << std::get<IritPoint>(*current_iter).get()->Pt[1] << std::endl;
#endif // DEBUG_NEIGHBORHOOD_SORT
                if (visited_points.count(&(*current_iter)) != 0)
                {
                    std::cerr << "ERROR: Unexpected inner loop, can't fill hole" << std::endl;
                    return std::vector<IritPoint>();
                }
                neighborhood_points_vector.push_back(
                    IritPoint(CagdPtCopy(std::get<IritPoint>(*current_iter).get()), CagdPtFree));
                last_point = &(*current_iter);
                visited_points.insert(last_point);
            }
        }
        else
        {
            CagdCrvStruct *curve = std::get<CagdCrvStruct *>(*current_iter);
#ifdef DEBUG_NEIGHBORHOOD_SORT
            std::cout << "\t\t\tCurve: " << curve->Points[1][0] << ", " << curve->Points[2][0] << " - "
                      << curve->Points[1][curve->Length - 1] << ", " << curve->Points[2][curve->Length - 1]
                      << std::endl;
#endif // DEBUG_NEIGHBORHOOD_SORT
            if (m_curve_to_offset_curves.count(curve) != 0 && !m_curve_to_offset_curves[curve].empty())
            {
                std::cerr << "ERROR: Invalid curve in neighborhood" << std::endl;
                throw std::runtime_error("Invalid curve in neighborhood");
            }
            for (const auto &junction : m_curve_to_junctions[curve])
            {
                if (junction != current_junction)
                {
                    current_junction = junction;
                    break;
                }
            }
            auto iter = std::find(neighborhood_points[current_junction].begin(),
                                  neighborhood_points[current_junction].end(), *current_iter);
            if (iter == neighborhood_points[current_junction].end())
            {
                std::cerr << "ERROR: Invalid curve in neighborhood" << std::endl;
                throw std::runtime_error("Invalid curve in neighborhood");
            }
            current_iter = iter;
        }
        ++current_iter;
        if (current_iter == neighborhood_points[current_junction].end())
        {
            current_iter = neighborhood_points[current_junction].begin();
        }
    } while (std::holds_alternative<CagdCrvStruct *>(*current_iter) ||
             std::get<IritPoint>(*current_iter) != std::get<IritPoint>(*first_point));

#ifdef DEBUG_NEIGHBORHOOD_SORT
    std::cout << "\t\tEnd of sorting neighborhood points" << std::endl;
#endif // DEBUG_NEIGHBORHOOD_SORT

    return neighborhood_points_vector;
}

std::tuple<CagdCrvStruct *, CagdRType, CagdRType> CurvesGenerator::get_points_matching_curve(
    const std::vector<CagdCrvStruct *> &curves, const IritPoint &p0, const IritPoint &p1)
{
    for (const auto &curve : curves)
    {
        IrtPtType Pl = {curve->Points[1][0], curve->Points[2][0], 0};
        IrtPtType Vl = {curve->Points[1][curve->Length - 1] - curve->Points[1][0],
                        curve->Points[2][curve->Length - 1] - curve->Points[2][0], 0};
        IrtPtType p0_point = {p0->Pt[0], p0->Pt[1], 0};
        IrtPtType p1_point = {p1->Pt[0], p1->Pt[1], 0};
        IrtPtType closest_point0;
        IrtPtType closest_point1;
        CagdRType t0 = GMPointFromPointLine(p0_point, Pl, Vl, closest_point0);
        CagdRType t1 = GMPointFromPointLine(p1_point, Pl, Vl, closest_point1);
        if (t0 >= 0 && t0 <= 1 && t1 >= 0 && t1 <= 1 && GMDistPointPoint(p0_point, closest_point0) < 0.001 &&
            GMDistPointPoint(p1_point, closest_point1) < 0.001)
        {
            return std::make_tuple(curve, t0, t1);
        }
    }
    return std::make_tuple(nullptr, 0, 0);
}
