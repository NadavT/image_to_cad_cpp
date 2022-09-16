#ifndef MATH_H
#define MATH_H

#include <cmath>
#include <opencv2/opencv.hpp>

static inline double distance(cv::Point p1, cv::Point p2)
{
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

static inline double distance_to_edge(cv::Point point, cv::Point edge_start, cv::Point edge_end)
{
    double x1 = edge_start.x;
    double y1 = edge_start.y;
    double x2 = edge_end.x;
    double y2 = edge_end.y;
    double x0 = point.x;
    double y0 = point.y;

    double numerator = std::abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1));
    double denominator = std::sqrt(std::pow(y2 - y1, 2) + std::pow(x2 - x1, 2));

    return numerator / denominator;
}

#endif /* MATH_H */
