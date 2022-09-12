#ifndef PROCESS_GRAPH_H
#define PROCESS_GRAPH_H

#include "types.h"

class ProcessGraph
{
  public:
    ProcessGraph(Graph &graph, VertexDescriptorMap &map, std::unordered_set<Segment> &added_edges,
                 double reduction_proximity, double hanging_threshold, double junction_collapse_threshold);

  private:
    void reduce();
    void remove_hanging();

    void contract_edge(const Segment &edge, std::unordered_set<Segment> &small_edges);

  private:
    Graph m_graph;
    VertexDescriptorMap m_vertex_descriptor_map;
    std::unordered_set<Segment> m_added_edges;

    double m_reduction_proximity;
    double m_hanging_threshold;
    double m_junction_collapse_threshold;
};

#endif /* PROCESS_GRAPH_H */
