#ifndef PROCESS_GRAPH_H
#define PROCESS_GRAPH_H

#include "types.h"

class ProcessGraph
{
  public:
    ProcessGraph(Graph &graph, VertexDescriptorMap &map, std::unordered_set<Segment> &added_edges,
                 double reduction_proximity, double hanging_threshold, double junction_collapse_threshold);

  private:
    void reduce(double reduction_proximity);
    void remove_hanging();
    void collapse_junctions(double junction_collapse_threshold);

    std::tuple<double, std::vector<VertexDescriptor>> walk_to_next_junction(VertexDescriptor source,
                                                                            VertexDescriptor direction);
    void contract_edge_small_edges_update(const Segment &edge, std::unordered_set<Segment> &small_edges);
    void contract_vertices(VertexDescriptor vertex1, VertexDescriptor vertex2);

  private:
    Graph m_graph;
    VertexDescriptorMap m_vertex_descriptor_map;
    std::unordered_set<Segment> m_added_edges;

    double m_reduction_proximity;
    double m_hanging_threshold;
    double m_junction_collapse_threshold;
};

#endif /* PROCESS_GRAPH_H */
