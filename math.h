#ifndef MATH_H
#define MATH_H

#include <cmath>
#include <inc_irit/irit_sm.h>
#include <opencv2/opencv.hpp>

template <typename T> static inline double distance_squared(T p1, T p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

template <typename T> static inline double distance(T p1, T p2)
{
    return std::sqrt(distance_squared(p1, p2));
}

template <typename T> static inline double distance_to_edge(T point, T edge_start, T edge_end)
{
    double length_squared = distance_squared(edge_start, edge_end);
    cv::norm(edge_end - edge_start);
    if (length_squared == 0)
    {
        return distance(cv::Point2d(point), cv::Point2d(edge_start));
    }
    double t = std::clamp((point - edge_start).dot(edge_end - edge_start) / length_squared, 0.0, 1.0);
    cv::Point2d projection = cv::Point2d(edge_start) + t * cv::Point2d(edge_end - edge_start);

    return distance(cv::Point2d(point), projection);
}

template <typename T> static inline double angle_between(T p0, T midpoint, T p1)
{
    T v1 = p0 - midpoint;
    T v2 = midpoint - p1;
    double dot = v1.dot(v2);
    return std::acos(dot / (cv::norm(v1) * cv::norm(v2)));
}

static inline double degrees_to_radians(double degrees)
{
    return degrees * M_PI / 180.0;
}

#endif /* MATH_H */
