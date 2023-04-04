#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "types.h"

class PreprocessImage
{
  public:
    PreprocessImage(const Image &image, bool should_convert_to_black_and_white, bool should_crop_to_fit,
                    int crop_to_fit_pad_left, int crop_to_fit_pad_right, int crop_to_fit_pad_top,
                    int crop_to_fit_pad_bottom, double island_threshold, bool should_add_border, double scale_factor);

    Image &get_colored_image();
    Image &get_grayscale_image();
    Segments &get_segments();
    std::vector<cv::Vec4i> &get_hierarchy();

  private:
    void convert_to_black_and_white();
    void crop_to_fit(int crop_to_fit_pad_left, int crop_to_fit_pad_right, int crop_to_fit_pad_top,
                     int crop_to_fit_pad_bottom);
    void add_border();
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
