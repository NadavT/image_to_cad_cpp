#ifndef PROCESS_GRAPH_H
#define PROCESS_GRAPH_H

#include "types.h"

class ProcessGraph
{
  public:
    ProcessGraph(Graph &graph, VertexDescriptorMap &map, std::unordered_set<Segment> &added_edges,
                 double reduction_proximity, double hanging_threshold, double junction_collapse_threshold,
                 double junction_smooth_threshold, int width, int height);

    static std::tuple<double, std::vector<VertexDescriptor>, std::vector<EdgeDescriptor>> walk_to_next_junction(
        VertexDescriptor source, VertexDescriptor direction, const Graph &graph, bool get_cycle = false);

    Graph &get_graph();

  private:
    void reduce(double reduction_proximity);
    void remove_hanging();
    void collapse_junctions(double junction_collapse_threshold);
    void smooth_junctions(double smooth_distance);

    std::vector<VertexDescriptor> get_junctions();

    void contract_edge_small_edges_update(const Segment &edge, std::unordered_set<Segment> &small_edges);
    void contract_vertices(VertexDescriptor vertex1, VertexDescriptor vertex2);
    void smooth_neighbors(VertexDescriptor vertex, double distance);

  private:
    Graph m_graph;
    VertexDescriptorMap m_vertex_descriptor_map;
    std::unordered_set<Segment> m_added_edges;

    double m_reduction_proximity;
    double m_hanging_threshold;
    double m_junction_collapse_threshold;
    double m_junction_smooth_threshold;
};

#endif /* PROCESS_GRAPH_H */
