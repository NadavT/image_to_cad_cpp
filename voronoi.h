#ifndef VORONOI_H
#define VORONOI_H

#include "types.h"

#include <unordered_map>

typedef cv::Point point_type;
typedef Segment segment_type;
typedef int coordinate_type;
typedef boost::polygon::voronoi_cell<int> cell_type;
typedef boost::polygon::voronoi_edge<int> edge_type;
typedef boost::polygon::voronoi_cell<int>::source_index_type source_index_type;
typedef boost::polygon::voronoi_cell<int>::source_category_type source_category_type;

class VoronoiCalculator
{
  public:
    VoronoiCalculator(const Image &image, const Segments &segments);

    VoronoiDiagram &get_diagram();
    Graph &get_graph();
    VertexDescriptorMap &get_vertex_descriptor_map();
    std::unordered_set<Segment> &get_added_edges();

  private:
    void calculate();
    bool check_mask(int x, int y);
    static double distance_to_edge(cv::Point point, cv::Point edge_start, cv::Point edge_end);
    void draw_graph();

    point_type retrieve_point(const cell_type &cell);
    segment_type retrieve_segment(const cell_type &cell);
    void sample_curved_edge(const edge_type &edge, std::vector<point_type> *sampled_edge);

  private:
    const Image &m_image;
    const Segments &m_segments;
    VoronoiDiagram m_diagram;
    Graph m_graph;
    VertexDescriptorMap m_vertex_descriptor_map;
    std::unordered_set<Segment> m_added_edges;
};

#endif /* VORONOI_H */
