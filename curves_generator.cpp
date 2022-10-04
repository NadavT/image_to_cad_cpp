#include "curves_generator.h"

#include "process_graph.h"
#include "utils.h"

#include <inc_irit/iritprsr.h>

CurvesGenerator::CurvesGenerator(Graph &graph)
    : m_graph(graph)
    , m_curves()
{
    TIMED_INNER_FUNCTION(generate_curves(), "Generating curves");
}

void CurvesGenerator::write(const std::string &filename)
{
    int handler = IPOpenDataFile(filename.c_str(), FALSE, TRUE);
    for (auto &curve : m_curves)
    {
        char *error;
        BzrCrvWriteToFile2(curve.get(), handler, 4, nullptr, &error);
    }
}

void CurvesGenerator::generate_curves()
{
    // Get candidates
    std::vector<EdgeDescriptor> candidates;
    for (const auto &vertex : boost::make_iterator_range(boost::vertices(m_graph)))
    {
        if (boost::degree(vertex, m_graph) > 0 && boost::degree(vertex, m_graph) != 2)
        {
            for (const auto &edge : boost::make_iterator_range(boost::out_edges(vertex, m_graph)))
            {
                candidates.push_back(edge);
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
            edge = candidates.back();
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

        // Add curve
        Curve curve =
            Curve(BspCrvNew(route.size(), (route.size() > 3) ? 4 : route.size(), CAGD_PT_E3_TYPE), CagdCrvFree);
        BspKnotUniformOpen(route.size(), (route.size() > 3) ? 4 : route.size(), curve->KnotVector);
        for (int i = 0; i < route.size(); ++i)
        {
            curve->Points[1][i] = m_graph[route[i]].p.x;
            curve->Points[2][i] = m_graph[route[i]].p.y;
            curve->Points[3][i] = 0; // m_graph[route[i]].distance_to_source;
        }
        m_curves.push_back(std::move(curve));

        // Clean route_edges
        for (const auto &edge : route_edges)
        {
            remaining.erase(edge);
        }
    }
}
