#include <boost/polygon/voronoi.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

#include "preprocess.h"
#include "utils.h"

cv::Mat load_image(const std::string &path)
{
    return cv::imread(path, cv::IMREAD_GRAYSCALE);
}

namespace boost
{
namespace polygon
{
template <> struct geometry_concept<cv::Point>
{
    typedef point_concept type;
};

template <> struct point_traits<cv::Point>
{
    typedef int coordinate_type;

    static inline coordinate_type get(const cv::Point &point, orientation_2d orient)
    {
        return (orient == HORIZONTAL) ? point.x : point.y;
    }
};

template <> struct geometry_concept<std::vector<cv::Point>>
{
    typedef segment_concept type;
};

template <> struct segment_traits<std::vector<cv::Point>>
{
    typedef int coordinate_type;
    typedef cv::Point point_type;

    static inline point_type get(const std::vector<cv::Point> &segment, direction_1d dir)
    {
        return dir.to_int() ? segment[1] : segment[0];
    }
};
} // namespace polygon
} // namespace boost

void calculate_voronoi(std::vector<std::vector<cv::Point>> &contours, boost::polygon::voronoi_diagram<double> &vd)
{
    std::cout << "\t"
              << "Calculating voronoi" << std::endl;
    std::vector<cv::Point> points;
    boost::polygon::construct_voronoi(points.begin(), points.end(), contours.begin(), contours.end(), &vd);
}

bool check_mask(const cv::Mat &image, int x, int y)
{
    if (x < 0 || y < 0 || x >= image.cols || y >= image.rows)
    {
        return false;
    }
    return image.at<cv::Vec3b>({x, y})[2] == 255;
}

double distance_to_edge(cv::Point point, cv::Point edge_start, cv::Point edge_end)
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

void draw_voronoi(cv::Mat &image, boost::polygon::voronoi_diagram<double> &vd,
                  std::vector<std::vector<cv::Point>> &contours)
{
    int width = image.cols;
    int height = image.rows;
    cv::Mat image2(image);
    cv::Mat image_vor(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    int line_width = 1;
    for (const auto &contour : contours)
    {
        cv::polylines(image_vor, contour, false, cv::Scalar(0, 0, 0), line_width);
    }
    for (const auto &cell : vd.cells())
    {
        auto *edge = cell.incident_edge();
        do
        {
            auto *start = edge->vertex0();
            auto *end = edge->vertex1();
            if (start && end && edge->is_primary() && edge->is_finite())
            {
                if (edge->is_linear())
                {
                    if (check_mask(image, start->x(), start->y()) && check_mask(image, end->x(), end->y()))
                    {
                        cv::line(image_vor, cv::Point(start->x(), start->y()), cv::Point(end->x(), end->y()),
                                 cv::Scalar(0, 0, 255), line_width);
                        cv::line(image2, cv::Point(start->x(), start->y()), cv::Point(end->x(), end->y()),
                                 cv::Scalar(0, 0, 255), line_width);
                    }
                }
                else
                {
                    if (check_mask(image, start->x(), start->y()) && check_mask(image, end->x(), end->y()))
                    {
                        cv::line(image_vor, cv::Point(start->x(), start->y()), cv::Point(end->x(), end->y()),
                                 cv::Scalar(0, 0, 255), line_width);
                        cv::line(image2, cv::Point(start->x(), start->y()), cv::Point(end->x(), end->y()),
                                 cv::Scalar(0, 0, 255), line_width);
                    }
                    assert(edge->cell()->source_category() != SOURCE_CATEGORY_SINGLE_POINT);
                    // point_type point =
                }
            }
            edge = edge->next();
        } while (edge != cell.incident_edge());
    }
    cv::imwrite("C:/technion/image_to_cad_cpp/results/voronoi.png", image_vor);
    cv::imwrite("C:/technion/image_to_cad_cpp/results/voronoi2.png", image2);
}

int main(int argc, char **argv)
{
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);

    cv::Mat image; // variable image of datatype Matrix
    image = load_image("C:/technion/image_to_cad_cpp/xPhys.ppm");

    // cv::imshow("Display Image", image);
    cv::imwrite("C:/technion/image_to_cad_cpp/results/original.png", image);
    // cv::waitKey(0);

    TIMED_FUNCTION(PreprocessImage preprocess_image(image, 4, 4), "Preprocessing");

    boost::polygon::voronoi_diagram<double> vd;
    auto start = std::chrono::high_resolution_clock::now();
    calculate_voronoi(preprocess_image.get_segments(), vd);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "\t"
              << "Finished calculating in " << elapsed.count() << std::endl;

    start = std::chrono::high_resolution_clock::now();
    draw_voronoi(preprocess_image.get_colored_image(), vd, preprocess_image.get_segments());
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    std::cout << "\t"
              << "Finished drawing in " << elapsed.count() << std::endl;
    // cv::waitKey(0);

    return 0;
}
