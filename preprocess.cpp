#include "preprocess.h"

#include <opencv2/core/utils/logger.hpp>
#include <opencv2/opencv.hpp>

#include "utils.h"

PreprocessImage::PreprocessImage(const Image &image, bool should_convert_to_black_and_white, bool should_crop_to_fit,
                                 int crop_to_fit_pad_left, int crop_to_fit_pad_right, int crop_to_fit_pad_top,
                                 int crop_to_fit_pad_bottom, double island_threshold, bool should_add_border,
                                 double scale_factor)
    : m_grayscale_image(image)
    , m_scale_factor(scale_factor)
    , m_island_threshold(island_threshold)
    , m_checked(image.cols, std::vector<bool>(image.rows, false))
{
    if (should_convert_to_black_and_white)
    {
        TIMED_INNER_FUNCTION(convert_to_black_and_white(), "Converting to black and white");
    }
    if (should_crop_to_fit)
    {
        TIMED_INNER_FUNCTION(
            crop_to_fit(crop_to_fit_pad_left, crop_to_fit_pad_right, crop_to_fit_pad_top, crop_to_fit_pad_bottom),
            "Cropping to fit");
    }
    TIMED_INNER_FUNCTION(remove_islands(), "Removing islands");

    if (should_add_border)
    {
        TIMED_INNER_FUNCTION(add_border(), "Adding border");
    }

    if (scale_factor != 1.0)
    {
        TIMED_INNER_FUNCTION(scale(), "Scaling");
    }

    cv::imwrite("preprocessed.png", m_grayscale_image);
}

cv::Mat &PreprocessImage::get_grayscale_image()
{
    return m_grayscale_image;
}

void PreprocessImage::convert_to_black_and_white()
{
    for (int i = 0; i < m_grayscale_image.cols; i++)
    {
        for (int j = 0; j < m_grayscale_image.rows; j++)
        {
            if (m_grayscale_image.at<unsigned char>({i, j}) < 128)
            {
                m_grayscale_image.at<unsigned char>({i, j}) = 0;
            }
            else
            {
                m_grayscale_image.at<unsigned char>({i, j}) = 255;
            }
        }
    }
}

void PreprocessImage::crop_to_fit(int crop_to_fit_pad_left, int crop_to_fit_pad_right, int crop_to_fit_pad_top,
                                  int crop_to_fit_pad_bottom)
{
    int leftmost = m_grayscale_image.cols;
    int rightmost = 0;
    int topmost = m_grayscale_image.rows;
    int bottommost = 0;
    for (int i = 0; i < m_grayscale_image.cols; i++)
    {
        for (int j = 0; j < m_grayscale_image.rows; j++)
        {
            if (m_grayscale_image.at<unsigned char>({i, j}) == 0)
            {
                leftmost = std::min(leftmost, i);
                rightmost = std::max(rightmost, i);
                topmost = std::min(topmost, j);
                bottommost = std::max(bottommost, j);
            }
        }
    }
    assert(leftmost < rightmost);
    assert(topmost < bottommost);
    if (leftmost >= rightmost || topmost >= bottommost)
    {
        std::cerr << "ERROR: Image is empty" << std::endl;
        throw std::runtime_error("Image is empty");
    }
    leftmost = std::clamp(leftmost - crop_to_fit_pad_left, 0, m_grayscale_image.cols);
    rightmost = std::clamp(rightmost + crop_to_fit_pad_right, 0, m_grayscale_image.cols);
    topmost = std::clamp(topmost - crop_to_fit_pad_top, 0, m_grayscale_image.rows);
    bottommost = std::clamp(bottommost + crop_to_fit_pad_bottom, 0, m_grayscale_image.rows);
    cv::Rect roi = cv::Rect(leftmost, topmost, rightmost - leftmost, bottommost - topmost);
    m_grayscale_image = m_grayscale_image(roi);
}

void PreprocessImage::add_border()
{
    cv::Mat bordered(m_grayscale_image.rows + 4, m_grayscale_image.cols + 4, m_grayscale_image.type(), cv::Scalar(255));
    cv::Rect offset_rect = cv::Rect(2, 2, m_grayscale_image.cols, m_grayscale_image.rows);
    m_grayscale_image.copyTo(bordered(offset_rect));
    m_grayscale_image = bordered;
}

void PreprocessImage::scale()
{
    cv::resize(m_grayscale_image, m_grayscale_image,
               cv::Size(m_grayscale_image.cols * m_scale_factor, m_grayscale_image.rows * m_scale_factor), 0, 0,
               cv::INTER_LINEAR);
}

std::list<cv::Point> PreprocessImage::get_near_surrounding(int x, int y, int color)
{
    std::list<cv::Point> surrounding;
    for (int i = std::max(x - 1, 0); i < std::min(x + 2, m_grayscale_image.cols); i++)
    {
        for (int j = std::max(y - 1, 0); j < std::min(y + 2, m_grayscale_image.rows); j++)
        {
            if (i == x && j == y)
            {
                continue;
            }
            if (m_grayscale_image.at<unsigned char>({i, j}) == color && !m_checked[i][j])
            {
                surrounding.push_back(cv::Point(i, j));
                m_checked[i][j] = true;
            }
        }
    }

    return surrounding;
}

std::vector<cv::Point> PreprocessImage::count_surrounding(int x, int y, int color)
{
    int count = 1;
    std::list<cv::Point> to_check = get_near_surrounding(x, y, color);
    std::vector<cv::Point> to_paint = {{x, y}};
    while (to_check.size() > 0)
    {
        cv::Point p = to_check.back();
        to_check.pop_back();
        m_checked[p.x][p.y] = true;
        to_paint.push_back(p);
        count++;
        std::list<cv::Point> surrounding = get_near_surrounding(p.x, p.y, color);
        to_check.insert(to_check.end(), surrounding.begin(), surrounding.end());
    }

    return to_paint;
}

void PreprocessImage::remove_islands()
{
    for (int color : {0, 255})
    {
        for (int i = 0; i < m_grayscale_image.cols; i++)
        {
            for (int j = 0; j < m_grayscale_image.rows; j++)
            {
                if (m_grayscale_image.at<unsigned char>({i, j}) == color && !m_checked[i][j])
                {
                    std::vector<cv::Point> to_paint = count_surrounding(i, j, color);
                    size_t count = to_paint.size();
                    if (count <= m_island_threshold)
                    {
                        for (cv::Point p : to_paint)
                        {
                            m_grayscale_image.at<unsigned char>(p) = 255 - color;
                        }
                    }
                }
                m_checked[i][j] = true;
            }
        }
    }
}
