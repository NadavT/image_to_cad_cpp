#ifndef CURVES_GENERATOR_H
#define CURVES_GENERATOR_H

#include "types.h"

class CurvesGenerator
{
  public:
    CurvesGenerator(Graph &graph, int max_order);

    std::vector<Curve> get_curves();

    void write(const std::string &filename);

  private:
    void generate_curves();

  private:
    Graph &m_graph;
    std::vector<Curve> m_curves;
    int m_max_order;
};

#endif /* CURVES_GENERATOR_H */
