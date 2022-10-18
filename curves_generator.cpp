#include "curves_generator.h"

#include "process_graph.h"
#include "utils.h"

#include <inc_irit/iritprsr.h>

CurvesGenerator::CurvesGenerator(Graph &graph, int max_order)
    : m_graph(graph)
    , m_curves()
    , m_max_order(max_order)
    , m_offset_curves()
{
    if (max_order == -1)
    {
        m_max_order = std::numeric_limits<int>::max();
    }
    TIMED_INNER_FUNCTION(generate_curves(), "Generating curves");
    TIMED_INNER_FUNCTION(generate_offset_curves(), "Generating offset curves");
    TIMED_INNER_FUNCTION(generate_surface_from_junctions(), "Generating surfaces");
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
        VertexDescriptor junction = -1;
        VertexDescriptor junction2 = -1;
        // Take candidate if available
        if (candidates.size() > 0)
        {
            edge = std::get<0>(candidates.back());
            junction = std::get<1>(candidates.back());
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
        std::tie(std::ignore, route, route_edges) =
            ProcessGraph::walk_to_next_junction(boost::source(edge, m_graph), boost::target(edge, m_graph), m_graph);
        if (route.back() != route.front())
        {
            junction2 = route.back();
        }

        // Add curve
        Curve curve = Curve(
            BspCrvNew(route.size(), (route.size() > m_max_order - 1) ? m_max_order : route.size(), CAGD_PT_E2_TYPE),
            CagdCrvFree);
        Curve width_curve = Curve(
            BspCrvNew(route.size(), (route.size() > m_max_order - 1) ? m_max_order : route.size(), CAGD_PT_E1_TYPE),
            CagdCrvFree);
        Curve opposite_width_curve = Curve(
            BspCrvNew(route.size(), (route.size() > m_max_order - 1) ? m_max_order : route.size(), CAGD_PT_E1_TYPE),
            CagdCrvFree);
        Curve height_curve = Curve(
            BspCrvNew(route.size(), (route.size() > m_max_order - 1) ? m_max_order : route.size(), CAGD_PT_E3_TYPE),
            CagdCrvFree);
        BspKnotUniformOpen(route.size(), (route.size() > m_max_order - 1) ? m_max_order : route.size(),
                           curve->KnotVector);
        BspKnotUniformOpen(route.size(), (route.size() > m_max_order - 1) ? m_max_order : route.size(),
                           width_curve->KnotVector);
        BspKnotUniformOpen(route.size(), (route.size() > m_max_order - 1) ? m_max_order : route.size(),
                           opposite_width_curve->KnotVector);
        BspKnotUniformOpen(route.size(), (route.size() > m_max_order - 1) ? m_max_order : route.size(),
                           height_curve->KnotVector);
        for (int i = 0; i < route.size(); ++i)
        {
            curve->Points[1][i] = m_graph[route[i]].p.x;
            curve->Points[2][i] = m_graph[route[i]].p.y;
            width_curve->Points[1][i] = m_graph[route[i]].distance_to_source;
            opposite_width_curve->Points[1][i] = -m_graph[route[i]].distance_to_source;

            height_curve->Points[1][i] = m_graph[route[i]].p.x;
            height_curve->Points[2][i] = m_graph[route[i]].p.y;
            height_curve->Points[3][i] = m_graph[route[i]].distance_to_source;
        }
        m_curves.push_back(
            {std::move(curve), std::move(width_curve), std::move(opposite_width_curve), junction, junction2});
        m_height_curves.push_back(std::move(height_curve));

        // Clean route_edges
        for (const auto &edge : route_edges)
        {
            remaining.erase(edge);
        }
        // CagdCrvCrvInter
    }
}

void CurvesGenerator::generate_offset_curves()
{
    int i = 0;
    for (auto &item : m_curves)
    {
        if (i != 0)
        {
            std::cout << "\x1b[A";
        }
        std::cout << "Finished " << i << " curves out of " << m_curves.size() << std::endl;
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
        Curve offset_curve = Curve(SymbCrvVarOffset(curve.get(), width_curve.get(), FALSE), CagdCrvFree);
        Curve opposite_offset_curve =
            Curve(SymbCrvVarOffset(curve.get(), opposite_width_curve.get(), FALSE), CagdCrvFree);
        // Curve offset_curve = Curve(SymbCrvOffset(curve.get(), 1, FALSE), CagdCrvFree);
        // Curve opposite_offset_curve = Curve(SymbCrvOffset(curve.get(), -1, FALSE), CagdCrvFree);
        m_offset_curves.push_back({std::move(offset_curve), junctions});
        m_offset_curves.push_back({std::move(opposite_offset_curve), junctions});
        m_curve_to_offset_curves[curve.get()] = {std::get<0>(m_offset_curves.back()).get(),
                                                 std::get<0>(*std::prev(m_offset_curves.end(), 2)).get()};
        for (const auto &junction : junctions)
        {
            m_junction_to_curves[junction].push_back(std::get<0>(m_offset_curves.back()).get());
            m_junction_to_curves[junction].push_back(std::get<0>(*std::prev(m_offset_curves.end(), 2)).get());
        }
        i++;
    }
    std::cout << "Finished " << i << " curves out of " << m_curves.size() << std::endl;
}

void CurvesGenerator::generate_surface_from_junctions()
{
    for (const auto &junction_matcher : m_junction_to_curves)
    {
        std::vector<IritPoint> points;
        for (auto itr = junction_matcher.second.cbegin(); itr != junction_matcher.second.cend(); ++itr)
        {
            CagdCrvStruct *curve1 = *itr;
            auto itr2 = itr;
            for (itr2 = ++itr2; itr2 != junction_matcher.second.cend(); ++itr2)
            {
                CagdCrvStruct *curve2 = *itr2;
                CagdPtStruct *intersections = CagdCrvCrvInter(curve1, curve2, 0.00001);
                for (CagdPtStruct *t = intersections; t != nullptr; t = t->Pnext)
                {
                    if (t->Pt[0] > 0 && t->Pt[0] < 1)
                    {
                        CagdPtStruct *point = CagdPtNew();
                        CAGD_CRV_EVAL_E2(curve1, t->Pt[0], &point->Pt[0]);
                        points.push_back(IritPoint(point, CagdPtFree));
                    }
                }
            }
        }
        if (points.size() == 3)
        {
            m_surfaces.push_back(IritSurface(
                CagdBilinearSrf(points[0].get(), points[1].get(), points[2].get(), points[1].get(), CAGD_PT_E2_TYPE),
                CagdSrfFree));
        }
        else if (points.size() == 4)
        {
            m_surfaces.push_back(IritSurface(
                CagdBilinearSrf(points[0].get(), points[1].get(), points[3].get(), points[2].get(), CAGD_PT_E2_TYPE),
                CagdSrfFree));
        }
    }
}
