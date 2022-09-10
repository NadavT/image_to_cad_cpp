#ifndef VORONOI_H
#define VORONOI_H

#include "types.h"

#include <unordered_map>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

typedef cv::Point point_type;
typedef std::vector<cv::Point> segment_type;
typedef int coordinate_type;
typedef boost::polygon::voronoi_cell<int> cell_type;
typedef boost::polygon::voronoi_edge<int> edge_type;
typedef boost::polygon::voronoi_cell<int>::source_index_type source_index_type;
typedef boost::polygon::voronoi_cell<int>::source_category_type source_category_type;

using Vertex = cv::Point;
using Edge = double;
using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Vertex, Edge>;
using VertexDescriptor = Graph::vertex_descriptor;

template <> struct std::hash<Vertex>
{
    std::size_t operator()(const Vertex &v) const noexcept
    {
        std::size_t seed = 0;
        boost::hash_combine(seed, v.x);
        boost::hash_combine(seed, v.y);
        return seed;
    }
};

class VoronoiCalculator
{
  public:
    VoronoiCalculator(const Image &image, const Segments &segments);

    VoronoiDiagram &get_diagram();

  private:
    void calculate();
    bool check_mask(int x, int y);
    static double distance_to_edge(cv::Point point, cv::Point edge_start, cv::Point edge_end);
    static double distance(cv::Point p1, cv::Point p2);
    void draw_graph();

    point_type retrieve_point(const cell_type &cell);
    segment_type retrieve_segment(const cell_type &cell);
    void sample_curved_edge(const edge_type &edge, std::vector<point_type> *sampled_edge);

  private:
    const Image &m_image;
    const Segments &m_segments;
    VoronoiDiagram m_diagram;
    Graph m_graph;
    std::unordered_map<Vertex, VertexDescriptor> m_vertex_descriptor_map;
};

#endif /* VORONOI_H */
