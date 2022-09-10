#ifndef VORONOI_H
#define VORONOI_H

#include "types.h"

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

  private:
    const Image &m_image;
    const Segments &m_segments;
    VoronoiDiagram m_diagram;
};

#endif /* VORONOI_H */
