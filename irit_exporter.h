#ifndef IRIT_EXPORTER_H
#define IRIT_EXPORTER_H

#include "types.h"

#include <iostream>
#include <string>

class IritExporter
{
  public:
    IritExporter(Graph &graph);

    void write(const std::string &filename);

  private:
    Graph &m_graph;
};

#endif /* IRIT_EXPORTER_H */
