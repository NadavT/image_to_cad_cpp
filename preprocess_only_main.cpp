#include <argparse/argparse.hpp>
#include <opencv2/core/utils/logger.hpp>

#include <filesystem>

#include "preprocess.h"
#include "utils.h"

cv::Mat load_image(const std::string &path)
{
    return cv::imread(path, cv::IMREAD_GRAYSCALE);
}

int main(int argc, char **argv)
{
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);

    auto start = std::chrono::high_resolution_clock::now();

    argparse::ArgumentParser program("Image to cad preprocessing only");

    program.add_argument("-i", "--input").help("Input image").required().action([](const std::string &value) {
        return load_image(value);
    });
    program.add_argument("-o", "--output_dir").help("Output directory").default_value(std::string("results"));
    program.add_argument("-gc", "--gamma_correction").help("Gamma correction").default_value(1.0).scan<'g', double>();
    program.add_argument("-no_cbw", "--no_convert_to_black_and_white")
        .help("Should not convert to black and white")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("-no_cft", "--no_crop_to_fit")
        .help("Should not crop to fit")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("-ctf_pl", "--crop_to_fit_padding_left")
        .help("Crop to fit padding left")
        .default_value(0)
        .scan<'i', int>();
    program.add_argument("-ctf_pr", "--crop_to_fit_padding_right")
        .help("Crop to fit padding right")
        .default_value(0)
        .scan<'i', int>();
    program.add_argument("-ctf_pt", "--crop_to_fit_padding_top")
        .help("Crop to fit padding top")
        .default_value(0)
        .scan<'i', int>();
    program.add_argument("-ctf_pb", "--crop_to_fit_padding_bottom")
        .help("Crop to fit padding bottom")
        .default_value(0)
        .scan<'i', int>();
    program.add_argument("-it", "--islands_threshold")
        .help("Islands threshold")
        .default_value(25.0)
        .scan<'g', double>();
    program.add_argument("-s", "--scale").help("Scale factor").default_value(4.0).scan<'g', double>();
    program.add_argument("-si", "--scale_interpolation")
        .help("Scale interpolation method (linear|nearest|linear exact|nearest exact)")
        .default_value(std::string("linear"));
    program.add_argument("-b", "--border").help("Should add border").default_value(false).implicit_value(true);

    try
    {
        program.parse_args(argc, argv);
    }
    catch (const std::runtime_error &err)
    {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        std::exit(1);
    }

    int scale_interpolation;
    if (program.get<std::string>("--scale_interpolation") == std::string("linear"))
    {
        scale_interpolation = cv::INTER_LINEAR;
    }
    else if (program.get<std::string>("--scale_interpolation") == std::string("nearest"))
    {
        scale_interpolation = cv::INTER_NEAREST;
    }
    else if (program.get<std::string>("--scale_interpolation") == std::string("linear exact"))
    {
        scale_interpolation = cv::INTER_LINEAR_EXACT;
    }
    else if (program.get<std::string>("--scale_interpolation") == std::string("nearest exact"))
    {
        scale_interpolation = cv::INTER_NEAREST_EXACT;
    }
    else
    {
        std::cerr << "Invalid scale interpolation method" << std::endl;
        std::exit(1);
    }

    if (!std::filesystem::exists(program.get<std::string>("--output_dir")))
    {
        std::filesystem::create_directories(program.get<std::string>("--output_dir"));
    }
    std::filesystem::current_path(program.get<std::string>("--output_dir"));

    Image grayscale_image = program.get<Image>("--input");
    cv::imwrite("original.png", grayscale_image);

    TIMED_FUNCTION(PreprocessImage preprocess_image(
                       grayscale_image, program.get<double>("--gamma_correction"),
                       !program.get<bool>("--no_convert_to_black_and_white"), !program.get<bool>("--no_crop_to_fit"),
                       program.get<int>("--crop_to_fit_padding_left"), program.get<int>("--crop_to_fit_padding_right"),
                       program.get<int>("--crop_to_fit_padding_top"), program.get<int>("--crop_to_fit_padding_bottom"),
                       program.get<double>("--islands_threshold"), program.get<bool>("--border"),
                       program.get<double>("--scale"), scale_interpolation),
                   "Preprocessing");

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << std::endl << "Finished all in: " << elapsed.count() << " seconds" << std::endl;

    exit(0);
    return 0;
}
