#include "preprocess.h"

#include <opencv2/core/utils/logger.hpp>
#include <opencv2/opencv.hpp>

#include "utils.h"

PreprocessImage::PreprocessImage(const Image &image, double scale_factor, double island_threshold, bool add_border)
    : m_grayscale_image(image)
    , m_scale_factor(scale_factor)
    , m_island_threshold(island_threshold)
    , m_checked(image.cols, std::vector<bool>(image.rows, false))
{
    TIMED_INNER_FUNCTION(remove_islands(), "removing islands");

    if (add_border)
    {
        cv::Mat bordered(image.rows + 4, image.cols + 4, image.type(), cv::Scalar(255));
        cv::Rect offset_rect = cv::Rect(2, 2, image.cols, image.rows);
        image.copyTo(bordered(offset_rect));
        m_grayscale_image = bordered;
    }

    cv::resize(m_grayscale_image, m_grayscale_image,
               cv::Size(m_grayscale_image.cols * scale_factor, m_grayscale_image.rows * scale_factor), 0, 0,
               cv::INTER_NEAREST_EXACT);

    TIMED_INNER_FUNCTION(find_segments(), "finding segments");

    cv::drawContours(m_colored_image, m_segments, -1, {0xff, 0, 0}, 1);
    cv::imwrite("contours.png", m_colored_image);
}

cv::Mat &PreprocessImage::get_colored_image()
{
    return m_colored_image;
}

cv::Mat &PreprocessImage::get_grayscale_image()
{
    return m_grayscale_image;
}

std::vector<std::vector<cv::Point>> &PreprocessImage::get_segments()
{
    return m_segments;
}

std::vector<cv::Vec4i> &PreprocessImage::get_hierarchy()
{
    return m_hierarchy;
}

std::list<cv::Point> PreprocessImage::get_near_surrounding(int x, int y)
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
            if (m_grayscale_image.at<unsigned char>({i, j}) == 0 && !m_checked[i][j])
            {
                surrounding.push_back(cv::Point(i, j));
                m_checked[i][j] = true;
            }
        }
    }

    return surrounding;
}

std::vector<cv::Point> PreprocessImage::count_surrounding(int x, int y)
{
    int count = 1;
    std::list<cv::Point> to_check = get_near_surrounding(x, y);
    std::vector<cv::Point> to_paint = {{x, y}};
    while (to_check.size() > 0)
    {
        cv::Point p = to_check.back();
        to_check.pop_back();
        m_checked[p.x][p.y] = true;
        to_paint.push_back(p);
        count++;
        std::list<cv::Point> surrounding = get_near_surrounding(p.x, p.y);
        to_check.insert(to_check.end(), surrounding.begin(), surrounding.end());
    }

    return to_paint;
}

void PreprocessImage::remove_islands()
{
    for (int i = 0; i < m_grayscale_image.cols; i++)
    {
        for (int j = 0; j < m_grayscale_image.rows; j++)
        {
            if (m_grayscale_image.at<unsigned char>({i, j}) == 0 && !m_checked[i][j])
            {
                std::vector<cv::Point> to_paint = count_surrounding(i, j);
                size_t count = to_paint.size();
                if (count <= m_island_threshold)
                {
                    for (cv::Point p : to_paint)
                    {
                        m_grayscale_image.at<unsigned char>(p) = 255;
                    }
                }
            }
            m_checked[i][j] = true;
        }
    }
}

void PreprocessImage::find_segments()
{
    cv::Mat edged;
    std::vector<std::vector<cv::Point>> contours;

    cv::cvtColor(m_grayscale_image, m_colored_image, cv::COLOR_GRAY2BGR);

    cv::Canny(m_grayscale_image, edged, 30, 200);

    cv::findContours(edged, contours, m_hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    contours.reserve(contours.size() + 4);

    std::vector<cv::Point> line1(m_grayscale_image.rows);
    int i = 0;
    std::generate(line1.begin(), line1.end(), [&]() { return cv::Point(0, i++); });
    contours.push_back(line1);

    std::vector<cv::Point> line2(m_grayscale_image.cols);
    i = 0;
    std::generate(line2.begin(), line2.end(), [&]() { return cv::Point(i++, 0); });
    contours.push_back(line2);

    std::vector<cv::Point> line3(m_grayscale_image.rows);
    i = 0;
    std::generate(line3.begin(), line3.end(), [&]() { return cv::Point(m_grayscale_image.cols - 1, i++); });
    contours.push_back(line3);

    std::vector<cv::Point> line4(m_grayscale_image.cols);
    i = 0;
    std::generate(line4.begin(), line4.end(), [&]() { return cv::Point(i++, m_grayscale_image.rows - 1); });
    contours.push_back(line4);

    for (const auto &contour : contours)
    {
        const cv::Point *prev = nullptr;
        for (const cv::Point &point : contour)
        {
            if (prev)
            {
                m_segments.push_back({*prev, point});
            }
            prev = &point;
        }
    }

    m_segments.shrink_to_fit();
}
