#ifndef CURVES_GENERATOR_H
#define CURVES_GENERATOR_H

#include "types.h"

using OffsetCurveDetails = std::tuple<Curve, std::vector<VertexDescriptor>>;
using OffsetCurveMatcher = std::unordered_map<CagdCrvStruct *, std::vector<CagdCrvStruct *>>;

using ImageGraph = boost::adjacency_list<boost::setS, boost::vecS, boost::bidirectionalS, cv::Point,
                                         boost::property<boost::edge_weight_t, float>>;

class CurvesGenerator
{
  public:
    CurvesGenerator(Graph &graph, int max_order, int target_order, double extrusion_amount,
                    const Image &reference_image, int distance_to_boundary_samples, int distance_to_boundary_threshold,
                    double distance_in_boundary_backoff, double distance_in_boundary_factor, double curve_density,
                    int min_curve_length, double junction_radius_adder);

    std::vector<Curve> get_curves();

    void write_curves(const std::string &filename);
    void write_offset_curves(const std::string &filename);
    void write_surfaces(const std::string &filename);
    void write_extrusions(const std::string &filename);

  private:
    void generate_image_graph();
    void generate_curves();
    void decrease_curves_order();
    void generate_offset_curves();
    void sort_junction_curves();
    void generate_surfaces_from_junctions();
    void generate_surfaces_from_curves();
    void extrude_surfaces();

    std::vector<std::pair<std::vector<VertexDescriptor>, std::vector<EdgeDescriptor>>> split_junction_walk(
        const std::pair<std::vector<VertexDescriptor>, std::vector<EdgeDescriptor>> &junction_walk);
    void generate_curve(const std::vector<VertexDescriptor> &route, const std::vector<EdgeDescriptor> &route_edges);
    std::vector<IritPoint> get_intersection_points(
        const std::pair<const VertexDescriptor, std::vector<CagdCrvStruct *>> &junction_matcher);
    void add_surface_from_4_points(const IritPoint &p0, const IritPoint &p1, const IritPoint &p2, const IritPoint &p3);
    void add_surface_from_2_lines(const IritPoint &line0_p0, const IritPoint &line0_p1, const IritPoint &line1_p0,
                                  const IritPoint &line1_p1);
    int distance_to_boundary(const cv::Point &point, int maximum_distance);
    cv::Point closest_point_on_boundary(const cv::Point &point, int maximum_distance);
    int distance_in_boundary(const cv::Point &p0, const cv::Point &p1);
    Curve trim_curve_to_fit_boundary(const Curve &curve, const Curve &width_curve, const Curve &curve_to_trim);
    void fix_surface_orientation(IritSurface &surface);

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
    const Image &m_reference_image;
    int m_distance_to_boundary_samples;
    int m_distance_to_boundary_threshold;
    double m_distance_in_boundary_backoff;
    double m_distance_in_boundary_factor;
    double m_curve_density;
    int m_min_curve_length;
    ImageGraph m_image_graph;
    VertexDescriptorMap m_image_graph_map;
    double m_junction_radius_adder;
};

#endif /* CURVES_GENERATOR_H */
