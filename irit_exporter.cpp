#include "irit_exporter.h"

#include <fstream>

#include "process_graph.h"

IritExporter::IritExporter(Graph &graph)
    : m_graph(graph)
{
}

void IritExporter::write(const std::string &filename)
{
    // Open export file for writing
    std::ofstream export_file(filename);

    // Write header
    export_file << "[OBJECT SCENE\n";

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

    std::set<EdgeDescriptor> remaining;
    for (const auto &edge : boost::make_iterator_range(boost::edges(m_graph)))
    {
        remaining.insert(edge);
    }

    // Write curves
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
        // Curve header
        export_file << "    [CURVE BEZIER " << route.size() << " E3\n";
        // Iterate over route
        for (const auto &vertex : route)
        {
            // Write vertex
            export_file << "            [" << m_graph[vertex].p.x << " " << m_graph[vertex].p.y << " " << 0 << "]\n";
        }
        // Curve footer
        export_file << "	]\n";
        // Clean route_edges
        for (const auto &edge : route_edges)
        {
            remaining.erase(edge);
        }
    }

    // Write footer
    export_file << "]\n";
}
