#include "process_graph.h"

#include "math.h"
#include "utils.h"

ProcessGraph::ProcessGraph(Graph &graph, VertexDescriptorMap &map, double reduction_proximity, double hanging_threshold,
                           double junction_collapse_threshold)
    : m_graph(graph)
    , m_vertex_descriptor_map(map)
    , m_reduction_proximity(reduction_proximity)
    , m_hanging_threshold(hanging_threshold)
    , m_junction_collapse_threshold(junction_collapse_threshold)
{
    TIMED_INNER_FUNCTION(reduce(), "Reducing graph");

    cv::Mat image_graph(2000, 4000, CV_8UC3, cv::Scalar(255, 255, 255));

    auto edges = boost::edges(m_graph);

    for (auto it = edges.first; it != edges.second; ++it)
    {
        const Vertex &u = m_graph[boost::source(*it, m_graph)];
        const Vertex &v = m_graph[boost::target(*it, m_graph)];
        cv::line(image_graph, u.p, v.p, cv::Scalar(0, 0, 255), 1);
    }

    cv::imwrite("C:/technion/image_to_cad_cpp/results/voronoi4.png", image_graph);
}

void ProcessGraph::reduce()
{
    std::unordered_set<Segment> small_edges;

    auto edges = boost::edges(m_graph);
    for (auto it = edges.first; it != edges.second; ++it)
    {
        Vertex &u = m_graph[boost::source(*it, m_graph)];
        Vertex &v = m_graph[boost::target(*it, m_graph)];
        assert(u.p != v.p);
        double length = m_graph[*it];
        if (length < m_reduction_proximity)
        {
            if (u.distance_to_source >= v.distance_to_source)
            {
                small_edges.insert({u.p, v.p});
            }
            else
            {
                small_edges.insert({v.p, u.p});
            }
        }
    }

    while (small_edges.size() > 0)
    {
        const auto &small_edge = *small_edges.begin();
        cv::Point u = small_edge[0];
        cv::Point v = small_edge[1];
        for (const auto &neighbor :
             boost::make_iterator_range(boost::adjacent_vertices(m_vertex_descriptor_map[v], m_graph)))
        {
            Vertex &collapsed = m_graph[neighbor];
            if (collapsed.p == u)
            {
                continue;
            }

            if (small_edges.count({collapsed.p, v}) > 0)
            {
                small_edges.erase({collapsed.p, v});
            }
            if (small_edges.count({v, collapsed.p}) > 0)
            {
                small_edges.erase({v, collapsed.p});
            }
        }

        // std::unordered_set<Segment> new_small_edges = contract_edge(small_edge);
        contract_edge(small_edge, small_edges);
        assert(small_edges.count(small_edge) > 0);
        small_edges.erase(small_edge);
    }
}

void ProcessGraph::remove_hanging()
{
    bool changed = true;
    while (changed)
    {
        changed = false;
        std::vector<VertexDescriptor> leafs;
        for (const auto &vertex : boost::make_iterator_range(boost::vertices(m_graph)))
        {
            if (boost::degree(vertex, m_graph) == 1)
            {
                leafs.push_back(vertex);
            }
        }

        for (const auto &vertex : leafs)
        {
            std::vector<VertexDescriptor> to_remove({vertex});

        }
    }
}

void ProcessGraph::contract_edge(const Segment &edge, std::unordered_set<Segment> &small_edges)
{
    cv::Point u = edge[0];
    cv::Point v = edge[1];
    assert(u != v);

    assert(m_vertex_descriptor_map.count(u) > 0);
    Vertex &u_details = m_graph[m_vertex_descriptor_map[u]];

    assert(m_vertex_descriptor_map.count(v) > 0);
    for (const auto &neighbor :
         boost::make_iterator_range(boost::adjacent_vertices(m_vertex_descriptor_map[v], m_graph)))
    {
        Vertex &collapsed = m_graph[neighbor];
        assert(collapsed.p != v);
        if (collapsed.p == u)
        {
            continue;
        }

        assert(u != collapsed.p);
        assert(m_vertex_descriptor_map.count(collapsed.p) > 0);
        assert(m_vertex_descriptor_map[u] != m_vertex_descriptor_map[collapsed.p]);
        assert(m_vertex_descriptor_map[u] != m_vertex_descriptor_map[v]);
        assert(m_vertex_descriptor_map[u] != m_vertex_descriptor_map[collapsed.p]);
        assert(u != collapsed.p);
        boost::add_edge(m_vertex_descriptor_map[u], m_vertex_descriptor_map[collapsed.p], distance(u, collapsed.p),
                        m_graph);

        if (distance(u, collapsed.p) < m_reduction_proximity)
        {
            if (u_details.distance_to_source >= collapsed.distance_to_source)
            {
                small_edges.insert({u, collapsed.p});
            }
            else
            {
                small_edges.insert({collapsed.p, u});
            }
        }
    }

    boost::clear_vertex(m_vertex_descriptor_map[v], m_graph);
    // boost::remove_vertex(m_vertex_descriptor_map[v], m_graph);
    // m_vertex_descriptor_map.clear();
    // for (const auto &vertex : boost::make_iterator_range(boost::vertices(m_graph)))
    // {
    //     m_vertex_descriptor_map[m_graph[vertex].p] = vertex;
    // }
}
