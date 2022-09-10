#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <opencv2/opencv.hpp>
#include <vector>

class PreprocessImage
{
  public:
    PreprocessImage(const cv::Mat &image, double scale_factor, double island_threshold);

    cv::Mat &get_colored_image();
    cv::Mat &get_grayscale_image();
    std::vector<std::vector<cv::Point>> &get_segments();
    std::vector<cv::Vec4i> &get_hierarchy();

  private:
    std::list<cv::Point> get_near_surrounding(int x, int y);
    std::vector<cv::Point> count_surrounding(int x, int y);
    void remove_islands();
    void find_segments();

  private:
    cv::Mat m_colored_image;
    cv::Mat m_grayscale_image;
    std::vector<std::vector<cv::Point>> m_segments;
    std::vector<cv::Vec4i> m_hierarchy;
    double m_scale_factor;
    double m_island_threshold;

    std::vector<std::vector<bool>> m_checked;
};

std::tuple<cv::Mat, cv::Mat, std::vector<std::vector<cv::Point>>, std::vector<cv::Vec4i>> preprocess_image(
    cv::Mat &image, double scale_factor, double islands_threshold);

#endif /* PREPROCESS_H */
