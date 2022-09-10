#ifndef TYPES_H
#define TYPES_H

#include <boost/polygon/voronoi.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

using Image = cv::Mat;
using Segments = std::vector<std::vector<cv::Point>>;
using VoronoiDiagram = boost::polygon::voronoi_diagram<double>;

#endif /* TYPES_H */
