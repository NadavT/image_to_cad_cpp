#include <opencv2/core/utils/logger.hpp>

#include "preprocess.h"
#include "process_graph.h"
#include "types.h"
#include "utils.h"
#include "voronoi.h"

cv::Mat load_image(const std::string &path)
{
    return cv::imread(path, cv::IMREAD_GRAYSCALE);
}

int main(int argc, char **argv)
{
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);

    auto start = std::chrono::high_resolution_clock::now();

    Image image; // variable image of datatype Matrix
    image = load_image("C:/technion/image_to_cad_cpp/xPhys.ppm");
    cv::imwrite("C:/technion/image_to_cad_cpp/results/original.png", image);

    TIMED_FUNCTION(PreprocessImage preprocess_image(image, 4, 4), "Preprocessing");
    TIMED_FUNCTION(
        VoronoiCalculator voronoi_calculator(preprocess_image.get_colored_image(), preprocess_image.get_segments()),
        "Calculating Voronoi");
    TIMED_FUNCTION(ProcessGraph process_graph(voronoi_calculator.get_graph(),
                                              voronoi_calculator.get_vertex_descriptor_map(), 2, 250, 14),
                   "Processing graph");

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << std::endl << "Finished all in: " << elapsed.count() << " seconds" << std::endl;

    return 0;
}
