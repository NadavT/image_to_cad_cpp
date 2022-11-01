#ifndef CURVES_GENERATOR_H
#define CURVES_GENERATOR_H

#include "types.h"

using OffsetCurveDetails = std::tuple<Curve, std::vector<VertexDescriptor>>;
using OffsetCurveMatcher = std::unordered_map<CagdCrvStruct *, std::vector<CagdCrvStruct *>>;

class CurvesGenerator
{
  public:
    CurvesGenerator(Graph &graph, int max_order, int target_order, double extrusion_amount);

    std::vector<Curve> get_curves();

    void write_curves(const std::string &filename);
    void write_offset_curves(const std::string &filename);
    void write_surfaces(const std::string &filename);
    void write_extrusions(const std::string &filename);

  private:
    void generate_curves();
    void decrease_curves_order();
    void generate_offset_curves();
    void sort_junction_curves();
    void generate_surfaces_from_junctions();
    void generate_surfaces_from_curves();
    void extrude_surfaces();

    std::vector<IritPoint> get_intersection_points(
        const std::pair<const VertexDescriptor, std::vector<CagdCrvStruct *>> &junction_matcher);
    void add_surface_from_4_points(const IritPoint &p0, const IritPoint &p1, const IritPoint &p2, const IritPoint &p3);
    void add_surface_from_2_lines(const IritPoint &line0_p0, const IritPoint &line0_p1, const IritPoint &line1_p0,
                                  const IritPoint &line1_p1);

  private:
    Graph &m_graph;
    std::vector<Curve> m_height_curves;
    std::vector<std::tuple<Curve, Curve, Curve, VertexDescriptor, VertexDescriptor>> m_curves;
    int m_max_order;
    int m_target_order;
    std::vector<OffsetCurveDetails> m_offset_curves;
    OffsetCurveMatcher m_curve_to_offset_curves;
    std::unordered_map<VertexDescriptor, std::vector<CagdCrvStruct *>> m_junction_to_curves;
    std::unordered_map<CagdCrvStruct *, std::unordered_map<VertexDescriptor, CagdRType>>
        m_offset_curve_subdivision_params;
    std::vector<IritSurface> m_surfaces;
    double m_extrusion_amount;
    std::vector<IritTV> m_extrusions;
};

#endif /* CURVES_GENERATOR_H */
