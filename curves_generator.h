#ifndef CURVES_GENERATOR_H
#define CURVES_GENERATOR_H

#include "types.h"

using OffsetCurveDetails = std::tuple<Curve, std::vector<VertexDescriptor>>;
using OffsetCurveMatcher = std::unordered_map<CagdCrvStruct *, std::vector<CagdCrvStruct *>>;

class CurvesGenerator
{
  public:
    CurvesGenerator(Graph &graph, int max_order);

    std::vector<Curve> get_curves();

    void write_curves(const std::string &filename);
    void write_offset_curves(const std::string &filename);
    void write_surfaces(const std::string &filename);

  private:
    void generate_curves();
    void generate_offset_curves();
    void generate_surface_from_junctions();

  private:
    Graph &m_graph;
    std::vector<Curve> m_height_curves;
    std::vector<std::tuple<Curve, Curve, Curve, VertexDescriptor, VertexDescriptor>> m_curves;
    int m_max_order;
    std::vector<OffsetCurveDetails> m_offset_curves;
    OffsetCurveMatcher m_curve_to_offset_curves;
    std::unordered_map<VertexDescriptor, std::vector<CagdCrvStruct *>> m_junction_to_curves;
    std::vector<IritSurface> m_surfaces;
};

#endif /* CURVES_GENERATOR_H */
