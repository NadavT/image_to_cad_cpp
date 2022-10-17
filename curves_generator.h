#ifndef CURVES_GENERATOR_H
#define CURVES_GENERATOR_H

#include "types.h"

using OffsetCurveDetails = std::tuple<Curve, std::vector<VertexDescriptor>>;

class CurvesGenerator
{
  public:
    CurvesGenerator(Graph &graph, int max_order);

    std::vector<Curve> get_curves();

    void write_curves(const std::string &filename);
    void write_offset_curves(const std::string &filename);

  private:
    void generate_curves();
    void generate_offset_curves();

  private:
    Graph &m_graph;
    std::vector<Curve> m_height_curves;
    std::vector<std::tuple<Curve, Curve, Curve, VertexDescriptor, VertexDescriptor>> m_curves;
    int m_max_order;
    std::vector<OffsetCurveDetails> m_offset_curves;
    std::unordered_map<VertexDescriptor, std::vector<std::vector<OffsetCurveDetails>::const_iterator>>
        m_junction_to_curves;
};

#endif /* CURVES_GENERATOR_H */
