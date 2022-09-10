#ifndef VORONOI_H
#define VORONOI_H

#include "types.h"

typedef cv::Point point_type;
typedef std::vector<cv::Point> segment_type;
typedef int coordinate_type;
typedef boost::polygon::voronoi_cell<double> cell_type;
typedef boost::polygon::voronoi_edge<double> edge_type;
typedef boost::polygon::voronoi_cell<double>::source_index_type source_index_type;
typedef boost::polygon::voronoi_cell<double>::source_category_type source_category_type;

class VoronoiCalculator
{
  public:
    VoronoiCalculator(const Image &image, const Segments &segments);

    VoronoiDiagram &get_diagram();

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
};

#endif /* VORONOI_H */
