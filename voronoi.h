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
    VoronoiCalculator(Image &image, const Image &grayscale_image);

    VoronoiDiagram &get_diagram();
    Graph &get_graph();
    VertexDescriptorMap &get_vertex_descriptor_map();
    std::unordered_set<Segment> &get_added_edges();

  private:
    void find_segments();
    void calculate();
    bool check_mask(int x, int y);
    void draw_graph();

    point_type retrieve_point(const cell_type &cell);
    segment_type retrieve_segment(const cell_type &cell);
    void sample_curved_edge(const edge_type &edge, std::vector<point_type> *sampled_edge);

  private:
    Image &m_image;
    const Image &m_grayscale_image;
    Segments m_segments;
    std::vector<cv::Vec4i> m_hierarchy;
    VoronoiDiagram m_diagram;
    Graph m_graph;
    VertexDescriptorMap m_vertex_descriptor_map;
    std::unordered_set<Segment> m_added_edges;
};

#endif /* VORONOI_H */
