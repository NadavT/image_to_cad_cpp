#ifndef TYPES_H
#define TYPES_H

#include <memory>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/polygon/voronoi.hpp>
#include <inc_irit/cagd_lib.h>
#include <opencv2/opencv.hpp>
#include <vector>

using Image = cv::Mat;
using Segment = std::vector<cv::Point>;
using Segments = std::vector<Segment>;
using VoronoiDiagram = boost::polygon::voronoi_diagram<double>;

struct Vertex
{
    cv::Point p;
    std::vector<cv::Point> incident_segment;
    double distance_to_source;
};

using Edge = double;
using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Vertex, Edge>;
using VertexDescriptor = Graph::vertex_descriptor;
using EdgeDescriptor = Graph::edge_descriptor;
using VertexDescriptorMap = std::unordered_map<cv::Point, VertexDescriptor>;
template <> struct std::hash<cv::Point>
{
    std::size_t operator()(const cv::Point &v) const noexcept
    {
        std::size_t seed = 0;
        boost::hash_combine(seed, v.x);
        boost::hash_combine(seed, v.y);
        return seed;
    }
};

template <> struct std::hash<Segment>
{
    std::size_t operator()(const Segment &segment) const noexcept
    {
        std::size_t seed = 0;
        boost::hash_combine(seed, std::hash<cv::Point>()(segment[0]));
        boost::hash_combine(seed, std::hash<cv::Point>()(segment[1]));
        return seed;
    }
};

using Curve = std::unique_ptr<CagdCrvStruct, decltype(&CagdCrvFree)>;

#endif /* TYPES_H */
