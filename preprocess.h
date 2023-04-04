#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "types.h"

class PreprocessImage
{
  public:
    PreprocessImage(const Image &image, double scale_factor, double island_threshold, bool add_border);

    Image &get_colored_image();
    Image &get_grayscale_image();
    Segments &get_segments();
    std::vector<cv::Vec4i> &get_hierarchy();

  private:
    void convert_to_black_and_white();
    void crop_to_fit();
    void add_border_to_image();
    void scale();
    std::list<cv::Point> get_near_surrounding(int x, int y, int color);
    std::vector<cv::Point> count_surrounding(int x, int y, int color);
    void remove_islands();
    void find_segments();

  private:
    Image m_colored_image;
    Image m_grayscale_image;
    Segments m_segments;
    std::vector<cv::Vec4i> m_hierarchy;
    double m_scale_factor;
    double m_island_threshold;

    std::vector<std::vector<bool>> m_checked;
};

#endif /* PREPROCESS_H */
