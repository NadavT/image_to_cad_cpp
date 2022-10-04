#ifndef CURVES_GENERATOR_H
#define CURVES_GENERATOR_H

#include "types.h"

class CurvesGenerator
{
  public:
    CurvesGenerator(Graph &graph);

    std::vector<Curve> get_curves();

    void write(const std::string &filename);

  private:
    void generate_curves();

  private:
    Graph &m_graph;
    std::vector<Curve> m_curves;
};

#endif /* CURVES_GENERATOR_H */
