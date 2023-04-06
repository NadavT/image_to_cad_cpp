#ifndef MATH_H
#define MATH_H

#include <cmath>
#include <inc_irit/irit_sm.h>
#include <opencv2/opencv.hpp>

static inline double distance_squared(cv::Point p1, cv::Point p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

static inline double distance(cv::Point p1, cv::Point p2)
{
    return std::sqrt(distance_squared(p1, p2));
}

static inline double distance_to_edge(cv::Point point, cv::Point edge_start, cv::Point edge_end)
{
    double length_squared = distance_squared(edge_start, edge_end);
    cv::norm(edge_end - edge_start);
    if (length_squared == 0)
    {
        return distance(point, edge_start);
    }
    double t = std::clamp((point - edge_start).dot(edge_end - edge_start) / length_squared, 0.0, 1.0);
    cv::Point projection = edge_start + t * (edge_end - edge_start);

    return distance(point, projection);
}

static inline double angle_between(cv::Point p0, cv::Point midpoint, cv::Point p1)
{
    cv::Point v1 = p0 - midpoint;
    cv::Point v2 = midpoint - p1;
    double dot = v1.dot(v2);
    return std::acos(dot / (cv::norm(v1) * cv::norm(v2)));
}

static inline double degrees_to_radians(double degrees)
{
    return degrees * M_PI / 180.0;
}

#endif /* MATH_H */
