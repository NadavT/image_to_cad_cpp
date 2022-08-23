#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <boost/polygon/voronoi.hpp>

int main(int argc, char** argv )
{
    boost::polygon::voronoi_diagram<double> vd;
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);
    cv::Mat image;  // variable image of datatype Matrix
    image = cv::imread("./OpenCV.png");

    cv::imshow("Display Image", image);
    cv::waitKey(0);
    return 0;
}