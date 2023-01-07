#include "curves_generator.h"

#include "math.h"
#include "process_graph.h"
#include "utils.h"

#include <cmath>
#include <execution>
#include <inc_irit/cagd_lib.h>
#include <inc_irit/iritprsr.h>

#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/johnson_all_pairs_shortest.hpp>

CurvesGenerator::CurvesGenerator(Graph &graph, int max_order, int target_order, double extrusion_amount,
                                 const Image &reference_image, int distance_to_boundary_samples,
                                 int distance_to_boundary_threshold, double curve_density, double junction_radius_adder)
    : m_graph(graph)
    , m_curves()
    , m_max_order(max_order)
    , m_target_order(target_order)
    , m_offset_curves()
    , m_extrusion_amount(extrusion_amount)
    , m_reference_image(reference_image)
    , m_distance_to_boundary_samples(distance_to_boundary_samples)
    , m_distance_to_boundary_threshold(distance_to_boundary_threshold)
    , m_curve_density(curve_density)
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

    TIMED_INNER_FUNCTION(generate_image_graph(), "Generating image graph");
    TIMED_INNER_FUNCTION(generate_curves(), "Generating curves");
    TIMED_INNER_FUNCTION(decrease_curves_order(), "Decrease curves order");
    TIMED_INNER_FUNCTION(generate_offset_curves(), "Generating offset curves");
    TIMED_INNER_FUNCTION(sort_junction_curves(), "Sorting junction curves");
    TIMED_INNER_FUNCTION(generate_surfaces_from_junctions(), "Generating surfaces from junctions");
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
                    ImageGraph::vertex_descriptor v1;
                    if (m_image_graph_map.count(p) == 0)
                    {
                        v1 = boost::add_vertex(p, m_image_graph);
                        m_image_graph_map[p] = v1;
                    }
                    else
                    {
                        v1 = m_image_graph_map[p];
                    }
                    ImageGraph::vertex_descriptor v2;
                    if (m_image_graph_map.count(inner_p) == 0)
                    {
                        v2 = boost::add_vertex(inner_p, m_image_graph);
                        m_image_graph_map[inner_p] = v2;
                    }
                    else
                    {
                        v2 = m_image_graph_map[inner_p];
                    }

                    boost::add_edge(v1, v2, m_image_graph);
                }
            }
        }
        std::cout << "\x1b[A";
        std::cout << "\t\tFinished " << i << " columns out of " << m_reference_image.cols << std::endl;
    }
    std::cout << "Image graph has " << boost::num_vertices(m_image_graph) << " vertices and "
              << boost::num_edges(m_image_graph) << " edges" << std::endl;

    std::vector<size_t> distances(boost::num_vertices(m_image_graph));
    auto recorder = boost::record_distances(distances.data(), boost::on_tree_edge{});

    boost::breadth_first_search(m_image_graph, 0, boost::visitor(boost::make_bfs_visitor(recorder)));
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
    std::mutex lock;
    std::cout << "\t\tFinished " << i << " curves out of " << m_curves.size() << std::endl;
    std::for_each(std::execution::par, std::begin(m_curves), std::end(m_curves), [&](auto &item) {
        lock.lock();
        const Curve &curve = std::get<0>(item);
        const Curve &width_curve = std::get<1>(item);
        const Curve &opposite_width_curve = std::get<2>(item);
        std::vector<VertexDescriptor> junctions;
        if (std::get<3>(item) != -1)
        {
            junctions.push_back(std::get<3>(item));
            if (m_junction_to_curves.count(std::get<3>(item)) == 0)
            {
                m_junction_to_curves[std::get<3>(item)] = std::vector<CagdCrvStruct *>();
            }
        }
        if (std::get<4>(item) != -1)
        {
            junctions.push_back(std::get<4>(item));
            if (m_junction_to_curves.count(std::get<4>(item)) == 0)
            {
                m_junction_to_curves[std::get<4>(item)] = std::vector<CagdCrvStruct *>();
            }
        }
        lock.unlock();
        Curve offset_curve = Curve(SymbCrvVarOffset(curve.get(), width_curve.get(), FALSE), CagdCrvFree);
        Curve opposite_offset_curve =
            Curve(SymbCrvVarOffset(curve.get(), opposite_width_curve.get(), FALSE), CagdCrvFree);
        // Curve offset_curve = Curve(SymbCrvOffset(curve.get(), 1, FALSE), CagdCrvFree);
        // Curve opposite_offset_curve = Curve(SymbCrvOffset(curve.get(), -1, FALSE), CagdCrvFree);
        Curve new_offset_curve = trim_curve_to_fit_boundary(curve, width_curve, offset_curve);
        Curve new_opposite_offset_curve =
            trim_curve_to_fit_boundary(curve, opposite_width_curve, opposite_offset_curve);

        CagdPtStruct *point = CagdPtNew();
        double max_distance_offset_curve = 0;
        double min_distance_offset_curve = std::numeric_limits<double>::max();
        double max_distance_opposite_curve = 0;
        double min_distance_opposite_curve = std::numeric_limits<double>::max();
        for (int i = 0; i < m_distance_to_boundary_samples; i++)
        {
            CAGD_CRV_EVAL_E2(new_offset_curve.get(), static_cast<double>(i) / (m_distance_to_boundary_samples - 1),
                             &point->Pt[0]);
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

        CAGD_CRV_EVAL_E2(new_offset_curve.get(), 0.1, &point->Pt[0]);
        cv::Point p0(point->Pt[0], point->Pt[1]);
        CAGD_CRV_EVAL_E2(new_offset_curve.get(), 0.9, &point->Pt[0]);
        cv::Point p1(point->Pt[0], point->Pt[1]);
        // CAGD_CRV_EVAL_E2(new_offset_curve.get(), 0.5, &point->Pt[0]);
        // cv::Point p2(point->Pt[0], point->Pt[1]);
        CAGD_CRV_EVAL_E2(new_opposite_offset_curve.get(), 0.1, &point->Pt[0]);
        cv::Point p3(point->Pt[0], point->Pt[1]);
        CAGD_CRV_EVAL_E2(new_opposite_offset_curve.get(), 0.9, &point->Pt[0]);
        cv::Point p4(point->Pt[0], point->Pt[1]);
        // CAGD_CRV_EVAL_E2(new_opposite_offset_curve.get(), 0.5, &point->Pt[0]);
        // cv::Point p5(point->Pt[0], point->Pt[1]);
        // CagdPtFree(point);

        // double diff1 = std::abs(distance_to_boundary(p2, m_distance_to_boundary_threshold * 2) -
        //                         distance_to_boundary(p0, m_distance_to_boundary_threshold));
        // double diff2 = std::abs(distance_to_boundary(p2, m_distance_to_boundary_threshold * 2) -
        //                         distance_to_boundary(p1, m_distance_to_boundary_threshold));
        // double diff3 = std::abs(distance_to_boundary(p5, m_distance_to_boundary_threshold * 2) -
        //                         distance_to_boundary(p3, m_distance_to_boundary_threshold));
        // double diff4 = std::abs(distance_to_boundary(p5, m_distance_to_boundary_threshold * 2) -
        //                         distance_to_boundary(p4, m_distance_to_boundary_threshold));
        double max_diff = std::max(
            {(max_distance_offset_curve - min_distance_offset_curve) / CagdCrvArcLenPoly(new_offset_curve.get()),
             (max_distance_opposite_curve - min_distance_opposite_curve) /
                 CagdCrvArcLenPoly(new_opposite_offset_curve.get())});

        double offset_curve_length = CagdCrvArcLenPoly(new_offset_curve.get());
        double opposite_offset_curve_length = CagdCrvArcLenPoly(new_opposite_offset_curve.get());

        lock.lock();
        if (junctions.size() < 2 ||
            (max_distance_offset_curve < m_distance_to_boundary_threshold &&
             max_distance_opposite_curve < m_distance_to_boundary_threshold && max_diff < 0.1 &&
             distance_in_boundary(closest_point_on_boundary(p0, m_distance_to_boundary_threshold),
                                  closest_point_on_boundary(p1, m_distance_to_boundary_threshold)) <
                 1000 * offset_curve_length &&
             distance_in_boundary(closest_point_on_boundary(p3, m_distance_to_boundary_threshold),
                                  closest_point_on_boundary(p4, m_distance_to_boundary_threshold)) <
                 1000 * opposite_offset_curve_length))
        {
            m_offset_curves.push_back({std::move(new_offset_curve), junctions});
            m_offset_curves.push_back({std::move(new_opposite_offset_curve), junctions});
            m_curve_to_offset_curves[curve.get()] = {std::get<0>(m_offset_curves.back()).get(),
                                                     std::get<0>(*std::prev(m_offset_curves.end(), 2)).get()};
            for (const auto &junction : junctions)
            {
                m_junction_to_curves[junction].push_back(curve.get());
                // m_junction_to_curves[junction].push_back(std::get<0>(m_offset_curves.back()).get());
                // m_junction_to_curves[junction].push_back(std::get<0>(*std::prev(m_offset_curves.end(),
                // 2)).get());
            }
        }
        i++;
        std::cout << "\x1b[A";
        std::cout << "\t\tFinished " << i << " curves out of " << m_curves.size() << std::endl;
        lock.unlock();
    });
}

void CurvesGenerator::sort_junction_curves()
{
    for (auto &item : m_junction_to_curves)
    {
        const cv::Point &junction_point = m_graph[item.first].p;
        std::sort(
            item.second.begin(), item.second.end(), [junction_point](const CagdCrvStruct *a, const CagdCrvStruct *b) {
                double a_cartesian = std::atan2(a->Points[2][1] - junction_point.y, a->Points[1][1] - junction_point.x);
                double b_cartesian = std::atan2(b->Points[2][1] - junction_point.y, b->Points[1][1] - junction_point.x);
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
        std::vector<IritPoint> points = get_intersection_points(junction_matcher);
        if (points.size() == 3)
        {
            IritSurface surface = IritSurface(
                CagdBilinearSrf(points[0].get(), points[1].get(), points[2].get(), points[1].get(), CAGD_PT_E2_TYPE),
                CagdSrfFree);
            fix_surface_orientation(surface);
            // CagdRType min, max;
            // CagdQuadGetPlnrSrfJacobianMinMax(surface.get(), &min, &max, TRUE);
            // CagdSrfReverse
            // if ((min < 0 && max > 0) || (min > 0 && max < 0))
            // {
            //     std::cerr << "Failed to get jacobian" << std::endl;
            // }
            // if (min < 0)
            // {
            //     surface = IritSurface(CagdBilinearSrf(points[1].get(), points[0].get(), points[1].get(),
            //                                           points[2].get(), CAGD_PT_E2_TYPE),
            //                           CagdSrfFree);
            // }
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
            std::sort(points.begin(), points.end(), [junction_point](const IritPoint &a, const IritPoint &b) {
                double a_cartesian = std::atan2(a->Pt[1] - junction_point.y, a->Pt[0] - junction_point.x);
                double b_cartesian = std::atan2(b->Pt[1] - junction_point.y, b->Pt[0] - junction_point.x);
                return a_cartesian < b_cartesian;
            });
            IritPoint pivot = IritPoint(CagdPtNew(), CagdPtFree);
            pivot->Pt[0] = junction_point.x;
            pivot->Pt[1] = junction_point.y;
            for (int i = 0; i < points.size(); i++)
            {
                IritSurface surface =
                    IritSurface(CagdBilinearSrf(points[i].get(), pivot.get(), points[(i + 1) % points.size()].get(),
                                                pivot.get(), CAGD_PT_E2_TYPE),
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
        CagdCrvStruct *wanted_curve1 = (offset_curve1_junction1_subdiv > 0 && offset_curve1_junction1_subdiv < 0.5)
                                           ? sliced_offset_curve1->Pnext
                                           : sliced_offset_curve1;
        CagdCrvStruct *wanted_curve2 = (offset_curve2_junction1_subdiv > 0 && offset_curve2_junction1_subdiv < 0.5)
                                           ? sliced_offset_curve2->Pnext
                                           : sliced_offset_curve2;
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
    for (size_t i = 0; i < junction_matcher.second.size(); ++i)
    {
        CagdCrvStruct *curve = junction_matcher.second[i];
        CagdCrvStruct *next_curve = junction_matcher.second[(i + 1) % junction_matcher.second.size()];
        bool found = false;
        CagdCrvStruct *choosen_curve = nullptr;
        CagdCrvStruct *choosen_next_curve = nullptr;
        CagdPtStruct *point = CagdPtNew();
        double distance_0 =
            distance(cv::Point(curve->Points[1][0], curve->Points[2][0]), m_graph[junction_matcher.first].p);
        double distance_1 =
            distance(cv::Point(curve->Points[1][curve->Length - 1], curve->Points[2][curve->Length - 1]),
                     m_graph[junction_matcher.first].p);
        CagdRType choosen_t1 = (distance_0 <= distance_1) ? 0 : 1;
        distance_0 =
            distance(cv::Point(next_curve->Points[1][0], next_curve->Points[2][0]), m_graph[junction_matcher.first].p);
        distance_1 = distance(
            cv::Point(next_curve->Points[1][next_curve->Length - 1], next_curve->Points[2][next_curve->Length - 1]),
            m_graph[junction_matcher.first].p);
        CagdRType choosen_t2 = (distance_0 <= distance_1) ? 0 : 1;
        CagdPtFree(point);
        for (auto offset_curve : m_curve_to_offset_curves[curve])
        {
            for (auto next_offset_curve : m_curve_to_offset_curves[next_curve])
            {
                CagdPtStruct *intersections = CagdCrvCrvInter(offset_curve, next_offset_curve, 0.00001);
                CagdPtStruct *intersections2 = CagdCrvCrvInter(next_offset_curve, offset_curve, 0.00001);
                for (CagdPtStruct *t = intersections; t != nullptr; t = t->Pnext)
                {
                    assert(intersections2 != nullptr);
                    CagdPtStruct *point = CagdPtNew();
                    CAGD_CRV_EVAL_E2(offset_curve, t->Pt[0], &point->Pt[0]);
                    double intersection_distance_to_junction =
                        distance(cv::Point(point->Pt[0], point->Pt[1]), m_graph[junction_matcher.first].p);
                    double curve_distance_to_junction_1 = distance(cv::Point(curve->Points[1][0], curve->Points[2][0]),
                                                                   m_graph[junction_matcher.first].p);
                    double curve_distance_to_junction_2 =
                        distance(cv::Point(curve->Points[1][curve->Length - 1], curve->Points[2][curve->Length - 1]),
                                 m_graph[junction_matcher.first].p);
                    if (t->Pt[0] > 0 && t->Pt[0] < 1 &&
                        intersection_distance_to_junction <
                            (curve_distance_to_junction_1 + curve_distance_to_junction_2) * 0.75)
                    {
                        if (found)
                        {
                            // std::cerr << "Found more than one intersection in junction" << std::endl;
                        }
                        CagdRType sub1 = std::min(t->Pt[0], 1 - t->Pt[0]);
                        CagdRType sub2 = std::min(choosen_t1, 1 - choosen_t1);
                        if (sub1 > sub2)
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
                                if (distance(cv::Point(point->Pt[0], point->Pt[1]),
                                             cv::Point(reference->Pt[0], reference->Pt[1])) <
                                    distance(cv::Point(point->Pt[0], point->Pt[1]),
                                             cv::Point(best->Pt[0], best->Pt[1])))
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
        }
    }
    std::set<CagdCrvStruct *> used;
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
        points.push_back(IritPoint(point, CagdPtFree));
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
                if (curve->Points[1][0] == m_graph[junction_matcher.first].p.x &&
                    curve->Points[2][0] == m_graph[junction_matcher.first].p.y)
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
            }
        }
    }
    return points;
}

void CurvesGenerator::add_surface_from_4_points(const IritPoint &p0, const IritPoint &p1, const IritPoint &p2,
                                                const IritPoint &p3)
{
    Curve curve1 = Curve(BzrCrvNew(2, CAGD_PT_E2_TYPE), CagdCrvFree);
    curve1->Points[1][0] = p0->Pt[0];
    curve1->Points[2][0] = p0->Pt[1];
    curve1->Points[1][1] = p1->Pt[0];
    curve1->Points[2][1] = p1->Pt[1];
    Curve curve2 = Curve(BzrCrvNew(2, CAGD_PT_E2_TYPE), CagdCrvFree);
    curve2->Points[1][0] = p2->Pt[0];
    curve2->Points[2][0] = p2->Pt[1];
    curve2->Points[1][1] = p3->Pt[0];
    curve2->Points[2][1] = p3->Pt[1];
    CagdPtStruct *intersections = CagdCrvCrvInter(curve1.get(), curve2.get(), 0.00001);
    if (intersections == nullptr)
    {
        add_surface_from_2_lines(p0, p1, p2, p3);
    }
    else
    {
        add_surface_from_2_lines(p0, p2, p1, p3);
    }
    CagdPtFreeList(intersections);
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
    CagdPtStruct *intersections = CagdCrvCrvInter(curve1.get(), curve2.get(), 0.00001);
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

    if (m_image_graph_map.find(p0) == m_image_graph_map.end())
    {
        std::cerr << "p0 is not in the graph" << std::endl;
        return std::numeric_limits<int>::max();
    }
    if (m_image_graph_map.find(p1) == m_image_graph_map.end())
    {
        std::cerr << "p1 is not in the graph" << std::endl;
        return std::numeric_limits<int>::max();
    }

    auto source = m_image_graph_map[p0];
    auto target = m_image_graph_map[p1];

    std::vector<int> distances(boost::num_vertices(m_image_graph), std::numeric_limits<int>::max());
    auto recorder = boost::record_distances(distances.data(), boost::on_tree_edge{});

    boost::breadth_first_search(m_image_graph, source, boost::visitor(boost::make_bfs_visitor(recorder)));

    return distances.at(target);
}

Curve CurvesGenerator::trim_curve_to_fit_boundary(const Curve &curve, const Curve &width_curve,
                                                  const Curve &curve_to_trim)
{
    double start_point = 0;
    double end_point = 1;
    cv::Point junction0(curve->Points[1][0], curve->Points[2][0]);
    double junction0_radius = std::abs(width_curve->Points[1][0]) + m_junction_radius_adder;
    cv::Point junction1(curve->Points[1][curve->Length - 1], curve->Points[2][curve->Length - 1]);
    double junction1_radius = std::abs(width_curve->Points[1][width_curve->Length - 1]) + m_junction_radius_adder;
    double curve_length = CagdCrvArcLenPoly(curve.get());
    for (double i = 0; i < 0.5; i += 1 / curve_length)
    {
        CagdPtStruct *point = CagdPtNew();
        CAGD_CRV_EVAL_E2(curve.get(), i, &point->Pt[0]);
        cv::Point p(point->Pt[0], point->Pt[1]);
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
    for (double i = 1; i > 0.5; i -= 1 / curve_length)
    {
        CagdPtStruct *point = CagdPtNew();
        CAGD_CRV_EVAL_E2(curve.get(), i, &point->Pt[0]);
        cv::Point p(point->Pt[0], point->Pt[1]);

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

void CurvesGenerator::fix_surface_orientation(IritSurface &surface)
{

    double min, max;
    CagdQuadGetPlnrSrfJacobianMinMax(surface.get(), &min, &max, TRUE);
    if (min * max < 0)
    {
        std::cerr << "Warning: surface is self intersecting: min: " << min << ", max: " << max << "Point0: ("
                  << surface->Points[1][0] << ", " << surface->Points[2][0] << ")" << std::endl;
        // throw std::runtime_error("Surface is self intersecting");
        surface.reset();
        return;
    }
    if (max > 0)
    {
        surface.reset(CagdSrfReverse2(surface.get()));
    }
}
