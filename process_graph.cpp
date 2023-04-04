#include "process_graph.h"

#include "math.h"
#include "utils.h"

ProcessGraph::ProcessGraph(Graph &graph, VertexDescriptorMap &map, std::unordered_set<Segment> &added_edges,
                           double reduction_proximity, double hanging_threshold, double junction_collapse_threshold,
                           double junction_smooth_threshold, int width, int height, bool add_border)
    : m_graph(graph)
    , m_vertex_descriptor_map(map)
    , m_added_edges(added_edges)
    , m_reduction_proximity(reduction_proximity)
    , m_hanging_threshold(hanging_threshold)
    , m_junction_collapse_threshold(junction_collapse_threshold)
    , m_junction_smooth_threshold(junction_smooth_threshold)
{
    if (add_border)
    {
        TIMED_INNER_FUNCTION(remove_border(width, height), "Removing border from graph");
    }

    TIMED_INNER_FUNCTION(reduce(m_reduction_proximity), "Reducing graph");
    cv::Mat image_graph(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    auto edges = boost::edges(m_graph);

    for (auto it = edges.first; it != edges.second; ++it)
    {
        const Vertex &u = m_graph[boost::source(*it, m_graph)];
        const Vertex &v = m_graph[boost::target(*it, m_graph)];
        cv::line(image_graph, u.p, v.p, cv::Scalar(0, 0, 255), 1);
    }

    cv::imwrite("after_reduction.png", image_graph);

    TIMED_INNER_FUNCTION(remove_hanging(width, height), "Removing hanging");
    TIMED_INNER_FUNCTION(collapse_junctions(m_junction_collapse_threshold), "Collapsing junctions");
    TIMED_INNER_FUNCTION(smooth_junctions(m_junction_smooth_threshold), "Smoothing junctions");

    image_graph = cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    edges = boost::edges(m_graph);

    for (auto it = edges.first; it != edges.second; ++it)
    {
        const Vertex &u = m_graph[boost::source(*it, m_graph)];
        const Vertex &v = m_graph[boost::target(*it, m_graph)];
        cv::line(image_graph, u.p, v.p, cv::Scalar(u.distance_to_source, 0, 255), 1);
    }

    cv::imwrite("final_graph_processing.png", image_graph);
}

void ProcessGraph::remove_border(int width, int height)
{
    // Iterate over all vertices
    auto vertices = boost::vertices(m_graph);
    for (auto it = vertices.first; it != vertices.second; ++it)
    {
        Vertex &vertex = m_graph[*it];
        if (vertex.p.x <= 8 || vertex.p.y <= 8 || vertex.p.x >= width - 9 || vertex.p.y >= height - 9)
        {
            boost::clear_vertex(*it, m_graph);
        }
    }
}

void ProcessGraph::reduce(double reduction_proximity)
{
    std::unordered_set<Segment> small_edges;

    auto edges = boost::edges(m_graph);
    for (auto it = edges.first; it != edges.second; ++it)
    {
        Vertex &u = m_graph[boost::source(*it, m_graph)];
        Vertex &v = m_graph[boost::target(*it, m_graph)];
        assert(u.p != v.p);
        double length = m_graph[*it];
        if (length < reduction_proximity)
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
        contract_edge_small_edges_update(small_edge, small_edges);
        assert(small_edges.count(small_edge) > 0);
        small_edges.erase(small_edge);
    }
}

void ProcessGraph::remove_hanging(int width, int height)
{
    bool changed = true;
    int i = 0;
    while (changed)
    {
        Graph new_graph(m_graph);
        changed = false;
        std::vector<VertexDescriptor> leafs;
        for (const auto &vertex : boost::make_iterator_range(boost::vertices(new_graph)))
        {
            // std::cout << "degree: " << boost::out_degree(vertex, m_graph) << std::endl;
            // for (const auto &neighbor : boost::make_iterator_range(boost::out_edges(vertex, m_graph)))
            // {
            //     std::cout << "\t" << m_graph[boost::source(neighbor, m_graph)].p << ", "
            //               << m_graph[boost::target(neighbor, m_graph)].p << std::endl;
            // }
            if (boost::degree(vertex, m_graph) == 1)
            {
                leafs.push_back(vertex);
            }
        }

        for (const auto &leaf : leafs)
        {
            std::vector<VertexDescriptor> to_remove({leaf});
            auto edge = *(boost::out_edges(leaf, m_graph).first);
            double leaf_length = m_graph[edge];
            VertexDescriptor parent =
                (boost::source(edge, m_graph) == leaf) ? boost::target(edge, m_graph) : boost::source(edge, m_graph);
            VertexDescriptor prev = leaf;
            while (boost::degree(parent, m_graph) == 2)
            {
                to_remove.push_back(parent);
                auto temp = parent;
                auto edge = (boost::out_edges(parent, m_graph).first);
                parent = (boost::source(*edge, m_graph) == temp) ? boost::target(*edge, m_graph)
                                                                 : boost::source(*edge, m_graph);
                if (parent == prev)
                {
                    ++edge;
                    parent = (boost::source(*edge, m_graph) == temp) ? boost::target(*edge, m_graph)
                                                                     : boost::source(*edge, m_graph);
                }
                leaf_length += m_graph[*edge];
                prev = temp;
            }
            if (leaf_length < m_hanging_threshold)
            {
                changed = true;
                for (const auto &vertex : to_remove)
                {
                    boost::clear_vertex(vertex, new_graph);
                }
            }
        }
        m_graph = new_graph;

        cv::Mat image_graph(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

        auto edges = boost::edges(m_graph);

        for (auto it = edges.first; it != edges.second; ++it)
        {
            const Vertex &u = m_graph[boost::source(*it, m_graph)];
            const Vertex &v = m_graph[boost::target(*it, m_graph)];
            cv::line(image_graph, u.p, v.p, cv::Scalar(0, 0, 255), 1);
        }

        cv::imwrite("epoch" + std::to_string(i++) + ".png", image_graph);
    }
}

void ProcessGraph::collapse_junctions(double junction_collapse_threshold)
{
    bool changed = true;
    while (changed)
    {
        changed = false;
        std::vector<VertexDescriptor> junctions = get_junctions();
        std::unordered_set<VertexDescriptor> collapsed;

        for (VertexDescriptor junction : junctions)
        {
            if (collapsed.count(junction) > 0)
            {
                continue;
            }
            bool reset = true;
            while (reset)
            {
                reset = false;
                for (const auto &neighbor : boost::make_iterator_range(boost::adjacent_vertices(junction, m_graph)))
                {
                    double length = 0;
                    std::vector<VertexDescriptor> route;
                    std::tie(length, route, std::ignore) = walk_to_next_junction(junction, neighbor, m_graph);
                    if (length < junction_collapse_threshold)
                    {
                        for (const auto &vertex : route)
                        {
                            if (vertex == junction || vertex == route.back())
                            {
                                collapsed.insert(vertex);
                                continue;
                            }
                            boost::clear_vertex(vertex, m_graph);
                        }
                        contract_vertices(junction, route.back());
                        m_graph[junction].p = (m_graph[junction].p + m_graph[route.back()].p) / 2;
                        auto incident_segment = m_graph[junction].incident_segment;
                        m_graph[junction].distance_to_source =
                            distance_to_edge(m_graph[junction].p, incident_segment[0], incident_segment[1]);
                        smooth_neighbors(junction, length);
                        reset = true;
                        changed = true;
                        break;
                    }
                }
            }
        }
    }
}

void ProcessGraph::smooth_junctions(double smooth_distance)
{
    std::vector<VertexDescriptor> junctions = get_junctions();
    for (VertexDescriptor junction : junctions)
    {
        double min_distance = std::numeric_limits<double>::max();
        for (const auto &neighbor : boost::make_iterator_range(boost::adjacent_vertices(junction, m_graph)))
        {
            double junction_distance;
            std::tie(junction_distance, std::ignore, std::ignore) = walk_to_next_junction(junction, neighbor, m_graph);
            if (junction_distance < min_distance)
            {
                min_distance = junction_distance;
            }
        }
        smooth_distance = std::min(smooth_distance, min_distance / 4);
        smooth_neighbors(junction, smooth_distance);
    }
}

std::vector<VertexDescriptor> ProcessGraph::get_junctions()
{
    std::vector<VertexDescriptor> junctions;
    for (const auto &vertex : boost::make_iterator_range(boost::vertices(m_graph)))
    {
        if (boost::degree(vertex, m_graph) > 2)
        {
            junctions.push_back(vertex);
        }
    }

    return junctions;
}

std::tuple<double, std::vector<VertexDescriptor>, std::vector<EdgeDescriptor>> ProcessGraph::walk_to_next_junction(
    VertexDescriptor source, VertexDescriptor direction, const Graph &graph, bool get_cycle)
{
    std::vector<VertexDescriptor> route({source, direction});
    std::vector<EdgeDescriptor> route_edges;
    for (const auto &edge : boost::make_iterator_range(boost::out_edges(source, graph)))
    {
        if (boost::target(edge, graph) == direction)
        {
            route_edges.push_back(edge);
            break;
        }
    }
    double length = distance(graph[source].p, graph[direction].p);
    VertexDescriptor parent = direction;
    VertexDescriptor prev = source;
    std::unordered_set<VertexDescriptor> visited({source, direction});
    while (boost::degree(parent, graph) == 2)
    {
        auto temp = parent;
        auto edge = (boost::out_edges(parent, graph).first);
        parent = (boost::source(*edge, graph) == temp) ? boost::target(*edge, graph) : boost::source(*edge, graph);
        if (parent == prev)
        {
            ++edge;
            parent = (boost::source(*edge, graph) == temp) ? boost::target(*edge, graph) : boost::source(*edge, graph);
        }
        if (visited.count(parent) > 0)
        {
            if (get_cycle)
            {
                route.push_back(parent);
                route_edges.push_back(*edge);
                length += graph[*edge];
            }
            break;
        }
        route.push_back(parent);
        route_edges.push_back(*edge);
        length += graph[*edge];
        visited.insert(parent);
        prev = temp;
    }

    return {length, route, route_edges};
}

Graph &ProcessGraph::get_graph()
{
    return m_graph;
}

void ProcessGraph::contract_edge_small_edges_update(const Segment &edge, std::unordered_set<Segment> &small_edges)
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
        if (collapsed.p == u || m_added_edges.count({u, collapsed.p}) > 0 || m_added_edges.count({collapsed.p, u}) > 0)
        {
            continue;
        }

        assert(u != collapsed.p);
        assert(m_vertex_descriptor_map.count(collapsed.p) > 0);
        assert(m_vertex_descriptor_map[u] != m_vertex_descriptor_map[collapsed.p]);
        assert(m_vertex_descriptor_map[u] != m_vertex_descriptor_map[v]);
        assert(u != collapsed.p);
        boost::add_edge(m_vertex_descriptor_map[u], neighbor, distance(u, collapsed.p), m_graph);
        m_added_edges.insert({u, collapsed.p});

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

    // m_graph[m_vertex_descriptor_map[u]].p = (u + v) / 2;
    // m_vertex_descriptor_map[(u + v) / 2] = m_vertex_descriptor_map[u];
    boost::clear_vertex(m_vertex_descriptor_map[v], m_graph);
    // boost::remove_vertex(m_vertex_descriptor_map[v], m_graph);
    // m_vertex_descriptor_map.clear();
    // for (const auto &vertex : boost::make_iterator_range(boost::vertices(m_graph)))
    // {
    //     m_vertex_descriptor_map[m_graph[vertex].p] = vertex;
    // }
}

void ProcessGraph::contract_vertices(VertexDescriptor vertex1, VertexDescriptor vertex2)
{
    Vertex &vertex1_details = m_graph[vertex1];
    Vertex &vertex2_details = m_graph[vertex2];

    for (const auto &neighbor : boost::make_iterator_range(boost::adjacent_vertices(vertex2, m_graph)))
    {
        Vertex &collapsed = m_graph[neighbor];
        if (collapsed.p == vertex1_details.p || m_added_edges.count({vertex1_details.p, collapsed.p}) > 0 ||
            m_added_edges.count({collapsed.p, vertex1_details.p}) > 0)
        {
            continue;
        }

        assert(vertex1_details.p != collapsed.p);
        assert(m_vertex_descriptor_map.count(collapsed.p) > 0);
        assert(vertex1 != m_vertex_descriptor_map[collapsed.p]);
        assert(vertex1 != vertex2);
        assert(vertex1_details.p != collapsed.p);
        boost::add_edge(vertex1, neighbor, distance(vertex1_details.p, collapsed.p), m_graph);
        m_added_edges.insert({vertex1_details.p, collapsed.p});
    }

    boost::clear_vertex(vertex2, m_graph);
}

void ProcessGraph::smooth_neighbors(VertexDescriptor vertex, double distance)
{
    bool changed = true;
    while (changed)
    {
        changed = false;
        for (const auto &edge : boost::make_iterator_range(boost::out_edges(vertex, m_graph)))
        {
            if (m_graph[edge] < distance)
            {
                VertexDescriptor neighbor = (boost::source(edge, m_graph) == vertex) ? boost::target(edge, m_graph)
                                                                                     : boost::source(edge, m_graph);
                // double neighbor_distance = m_graph[neighbor].distance_to_source;
                contract_vertices(vertex, neighbor);
                // m_graph[vertex].distance_to_source = neighbor_distance;
                changed = true;
                break;
            }
        }
    }
}
