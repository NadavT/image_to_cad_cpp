#ifndef CURVES_GENERATOR_H
#define CURVES_GENERATOR_H

#include "types.h"

using OffsetCurveDetails = std::tuple<Curve, std::vector<VertexDescriptor>>;
using OffsetCurveMatcher = std::unordered_map<CagdCrvStruct *, std::vector<CagdCrvStruct *>>;

using ImageGraph = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, unsigned int,
                                         boost::property<boost::edge_weight_t, double>>;
using ImageVertexDescriptor = ImageGraph::vertex_descriptor;
using ImageEdgeDescriptor = ImageGraph::edge_descriptor;
using ImageVertexDescriptorMap = std::unordered_map<cv::Point, ImageVertexDescriptor>;

class CurvesGenerator
{
  public:
    CurvesGenerator(Graph &graph, int max_order, int target_order, double extrusion_amount, bool filter_curves,
                    const Image &reference_image, int distance_to_boundary_samples, int distance_to_boundary_threshold,
                    double distance_in_boundary_backoff, double distance_in_boundary_factor, double curve_density,
                    int min_curve_length, double junction_radius_adder);

    std::vector<Curve> get_curves();

    void write_curves(const std::string &filename);
    void write_offset_curves_before_trim(const std::string &filename);
    void write_offset_curves(const std::string &filename);
    void write_filtered_offset_curves(const std::string &filename);
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
    void fill_holes();
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
    void fix_offset_curves_surface_self_intersection(Curve &offset_curve, Curve &opposite_offset_curve);
    void fix_offset_curves_surface_self_intersection(Curve &offset_curve, Curve &connection_0_curve,
                                                     Curve &connection_1_curve);
    void fix_surface_orientation(IritSurface &surface, bool print_error = true);
    IritSurface generate_surface_from_pivot_and_points(const IritPoint &pivot, const IritPoint &p0, const IritPoint &p1,
                                                       double radius);
    std::unordered_set<VertexDescriptor> get_marked_neighborhood(const VertexDescriptor &junction);
    std::unordered_set<VertexDescriptor> get_marked_neighborhood(const VertexDescriptor &junction,
                                                                 std::unordered_set<VertexDescriptor> &visited);

  private:
    Graph &m_graph;
    std::vector<Curve> m_height_curves;
    std::vector<std::tuple<Curve, Curve, Curve, VertexDescriptor, VertexDescriptor>> m_curves;
    int m_max_order;
    int m_target_order;
    std::vector<Curve> m_offset_curves_before_trim;
    std::vector<OffsetCurveDetails> m_offset_curves;
    bool m_filter_offset_curves;
    std::vector<OffsetCurveDetails> m_filtered_offset_curves;
    OffsetCurveMatcher m_curve_to_offset_curves;
    std::unordered_map<VertexDescriptor, std::vector<CagdCrvStruct *>> m_junction_to_curves;
    std::set<std::pair<VertexDescriptor, VertexDescriptor>> m_connections;
    std::unordered_map<VertexDescriptor, unsigned int> m_marked_junctions;
    std::unordered_map<CagdCrvStruct *, std::unordered_map<VertexDescriptor, CagdRType>>
        m_offset_curve_subdivision_params;
    std::unordered_map<CagdPtStruct *, std::vector<std::tuple<CagdCrvStruct *, CagdCrvStruct *, CagdRType>>>
        m_point_to_originating_curve;
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
    double m_junction_radius_adder;
    std::unordered_map<cv::Point, double> m_junctions_radius;
    std::mutex m_junction_radius_lock;
};

#endif /* CURVES_GENERATOR_H */
