#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <boost/polygon/voronoi.hpp>

std::list<cv::Point> get_near_surrounding(cv::Mat& image, int x, int y, std::vector<std::vector<bool>>& checked)
{
    std::list<cv::Point> surrounding;
    for (int i = std::max(x-1, 0); i < std::min(x+2, image.cols); i++)
    {
        for (int j = std::max(y-1, 0); j < std::min(y+2, image.rows); j++)
        {
            if (i == x && j == y)
            {
                continue;
            }
            if (image.at<unsigned char>({i, j}) == 0 && !checked[i][j])
            {
                surrounding.push_back(cv::Point(i, j));
                checked[i][j] = true;
            }
        }
    }
    
    return surrounding;
}

std::tuple<int, std::vector<cv::Point>> count_surrounding(cv::Mat& image, int x, int y, std::vector<std::vector<bool>>& checked)
{
    int count = 1;
    std::list<cv::Point> to_check = get_near_surrounding(image, x, y, checked);
    std::vector<cv::Point> to_paint = {{x, y}};
    while (to_check.size() > 0)
    {
        cv::Point p = to_check.back();
        to_check.pop_back();
        checked[p.x][p.y] = true;
        to_paint.push_back(p);
        count++;
        std::list<cv::Point> surrounding = get_near_surrounding(image, p.x, p.y, checked);
        to_check.insert(to_check.end(), surrounding.begin(), surrounding.end());
    }

    return {count, to_paint};
}

cv::Mat remove_islands(cv::Mat& image, double threshold)
{
    std::vector<std::vector<bool>> checked(image.cols, std::vector<bool>(image.rows, false));
    for (int i = 0; i < image.cols; i++)
    {
        for (int j = 0; j < image.rows; j++)
        {
            if (image.at<unsigned char>({i, j}) == 0 && !checked[i][j])
            {
                int count;
                std::vector<cv::Point> to_paint;
                std::tie(count, to_paint) = count_surrounding(image, i, j, checked);
                if (count <= threshold)
                {
                    for (cv::Point p : to_paint)
                    {
                        image.at<unsigned char>(p) = 255;
                    }
                }
            }
            if (!checked[i][j])
            {
                checked[i][j] = true;
            }
        }
    }

    return image;
}

cv::Mat load_image(const std::string& path)
{
    return cv::imread(path, cv::IMREAD_GRAYSCALE);
}

std::tuple<cv::Mat, cv::Mat, std::vector<std::vector<cv::Point>>, std::vector<cv::Vec4i>> preprocess_image(cv::Mat& image, double scale_factor, double islands_threshold)
{
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << "\t" << "Removing islands" << std::endl;
    image = remove_islands(image, islands_threshold);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "\t" << "Finished removing islands in " << elapsed.count() << std::endl;
    cv::Mat output;
    cv::Mat colored;
    cv::Mat edged;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::resize(image, output, cv::Size(image.cols * scale_factor, image.rows * scale_factor), 0, 0, cv::INTER_NEAREST_EXACT);

    cv::cvtColor(output, colored, cv::COLOR_GRAY2BGR);

    cv::Canny(output, edged, 30, 200);

    std::cout << "\t" << "Finding contours" << std::endl;
    cv::findContours(edged, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    contours.reserve(contours.size() + 4);

    std::vector<cv::Point> line1(output.rows);
    int i = 0;
    std::generate(line1.begin(), line1.end(), [&](){return cv::Point(0, i);});
    contours.push_back(line1);

    std::vector<cv::Point> line2(output.cols);
    i = 0;
    std::generate(line2.begin(), line2.end(), [&](){return cv::Point(i, 0);});
    contours.push_back(line2);

    std::vector<cv::Point> line3(output.rows);
    i = 0;
    std::generate(line3.begin(), line3.end(), [&](){return cv::Point(output.cols, i);});
    contours.push_back(line3);

    std::vector<cv::Point> line4(output.cols);
    i = 0;
    std::generate(line4.begin(), line4.end(), [&](){return cv::Point(i, output.rows);});
    contours.push_back(line4);

    contours.shrink_to_fit();

    return {colored, output, contours, hierarchy};
}

namespace boost {
namespace polygon {
template <>
struct geometry_concept<cv::Point> {
  typedef point_concept type;
};

template <>
struct point_traits<cv::Point> {
  typedef int coordinate_type;

  static inline coordinate_type get(
      const cv::Point& point, orientation_2d orient) {
    return (orient == HORIZONTAL) ? point.x : point.y;
  }
};

template <>
struct geometry_concept<std::vector<cv::Point>> {
  typedef segment_concept type;
};

template <>
struct segment_traits<std::vector<cv::Point>> {
  typedef int coordinate_type;
  typedef cv::Point point_type;

  static inline point_type get(const std::vector<cv::Point>& segment, direction_1d dir) {
    return dir.to_int() ? segment[1] : segment[0];
  }
};
}  // polygon
}  // boost

void calculate_voronoi(std::vector<std::vector<cv::Point>>& contours, boost::polygon::voronoi_diagram<double>& vd)
{
    std::cout << "\t" << "Calculating voronoi" << std::endl;
    std::vector<cv::Point> points;
    boost::polygon::construct_voronoi(points.begin(), points.end(), contours.begin(), contours.end(), &vd);
}

bool check_mask(const cv::Mat& image, int x, int y)
{
    if (x < 0 || y < 0 || x >= image.cols || y >= image.rows)
    {
        return false;
    }
    return image.at<cv::Vec3b>({y, x})[2] == 255;
}

bool distance_to_edge(cv::Point point, cv::Point edge_start, cv::Point edge_end)
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



int main(int argc, char** argv )
{
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);
    cv::Mat image;  // variable image of datatype Matrix
    image = load_image("C:/technion/image_to_cad_cpp/xPhys.ppm");

    cv::imshow("Display Image", image);
    cv::imwrite("C:/technion/image_to_cad_cpp/results/original.png", image);
    cv::waitKey(0);

    cv::Mat grayscale;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::tie(image, grayscale, contours, hierarchy) = preprocess_image(image, 4, 4);
    cv::drawContours(image, contours, -1, {0xff, 0, 0}, 1);
    cv::namedWindow("contours", cv::WINDOW_NORMAL);
    cv::imshow("contours", image);
    cv::imwrite("C:/technion/image_to_cad_cpp/results/contours.png", image);
    std::cout << "Finished preprocessing" << std::endl;

    boost::polygon::voronoi_diagram<double> vd;
    calculate_voronoi(contours, vd);

    cv::waitKey(0);

    return 0;
}
