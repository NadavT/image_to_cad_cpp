#ifndef MATH_H
#define MATH_H

static inline double distance(cv::Point p1, cv::Point p2)
{
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

#endif /* MATH_H */
