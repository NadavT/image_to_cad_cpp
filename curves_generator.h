#ifndef CURVES_GENERATOR_H
#define CURVES_GENERATOR_H

#include "types.h"

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
    std::vector<std::tuple<Curve, Curve, Curve>> m_curves;
    int m_max_order;
    std::vector<Curve> m_offset_curves;
};

#endif /* CURVES_GENERATOR_H */
