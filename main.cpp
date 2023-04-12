#include <argparse/argparse.hpp>
#include <opencv2/core/utils/logger.hpp>

#include <filesystem>

#include "irit_exporter.h"
#include "preprocess.h"
#include "process_graph.h"
#include "surfaces_generator.h"
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

    argparse::ArgumentParser program("Image to cad");

    program.add_argument("-i", "--input").help("Input image").required().action([](const std::string &value) {
        return load_image(value);
    });
    program.add_argument("-o", "--output_dir").help("Output directory").default_value(std::string("results"));
    program.add_argument("-no_pp", "--no_preprocessing")
        .help("Should not apply preprocessing to input image")
        .default_value(false)
        .implicit_value(true);
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
    program.add_argument("-b", "--border").help("Should add border").default_value(false).implicit_value(true);
    program.add_argument("-r", "--reduction_proximity")
        .help("Reduction proximity")
        .default_value(10.0)
        .scan<'g', double>();
    program.add_argument("-lt", "--hanging_leaf_threshold")
        .help("Hanging leaf threshold")
        .default_value(250.0)
        .scan<'g', double>();
    program.add_argument("-jct", "--junction_collapse_threshold")
        .help("Junction collapse threshold")
        .default_value(20.0)
        .scan<'g', double>();
    program.add_argument("-jst", "--junction_smooth_threshold")
        .help("Junction smooth threshold")
        .default_value(10.0)
        .scan<'g', double>();
    program.add_argument("-co", "--curve_order")
        .help("Assign the maximal order of the curve (B-Spline), use -1 to unlimited (Bezier curve)")
        .default_value(100)
        .scan<'i', int>();
    program.add_argument("-tco", "--target_curve_order")
        .help("Assign the target order of the curve (B-Spline), use -1 to unlimited (Bezier curve)")
        .default_value(4)
        .scan<'i', int>();
    program.add_argument("-cd", "--curve_density")
        .help("density of the curves (number of points per unit length of arc)")
        .default_value(0.1)
        .scan<'g', double>();
    program.add_argument("-cml", "--curve_min_length")
        .help("minimum control points of the curves")
        .default_value(3)
        .scan<'i', int>();
    program.add_argument("-jra", "--junction_radius_adder")
        .help("The radius of the junctions will be increased by this value (for treaming the outgoing curves")
        .default_value(6.0)
        .scan<'g', double>();
    program.add_argument("-ex", "--extrusion").help("Extrusion amount").default_value(25.0).scan<'g', double>();
    program.add_argument("-foc", "--filter_offset_curves")
        .help("Should filter offset curves")
        .default_value(false)
        .implicit_value(true);
    program.add_argument("-dbs", "--distance_to_boundary_samples")
        .help("distance to boundary samples")
        .default_value(5)
        .scan<'i', int>();
    program.add_argument("-dbt", "--distance_to_boundary_threshold")
        .help("distance to boundary threshold")
        .default_value(2)
        .scan<'i', int>();
    program.add_argument("-dbb", "--distance_in_boundary_backoff")
        .help("distance in boundary backoff")
        .default_value(0.1)
        .scan<'g', double>();
    program.add_argument("-dbf", "--distance_in_boundary_factor")
        .help("distance in boundary factor")
        .default_value(10.0)
        .scan<'g', double>();

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

    if (!std::filesystem::exists(program.get<std::string>("--output_dir")))
    {
        std::filesystem::create_directories(program.get<std::string>("--output_dir"));
    }
    std::filesystem::current_path(program.get<std::string>("--output_dir"));

    Image grayscale_image = program.get<Image>("--input");
    cv::imwrite("original.png", grayscale_image);

    if (!program.get<bool>("--no_preprocessing"))
    {
        TIMED_FUNCTION(PreprocessImage preprocess_image(
                           grayscale_image, !program.get<bool>("--no_convert_to_black_and_white"),
                           !program.get<bool>("--no_crop_to_fit"), program.get<int>("--crop_to_fit_padding_left"),
                           program.get<int>("--crop_to_fit_padding_right"),
                           program.get<int>("--crop_to_fit_padding_top"),
                           program.get<int>("--crop_to_fit_padding_bottom"), program.get<double>("--islands_threshold"),
                           program.get<bool>("--border"), program.get<double>("--scale")),
                       "Preprocessing");
        grayscale_image = preprocess_image.get_grayscale_image();
    }

    Image colored_image;
    cv::cvtColor(grayscale_image, colored_image, cv::COLOR_GRAY2BGR);

    TIMED_FUNCTION(VoronoiCalculator voronoi_calculator(colored_image, grayscale_image), "Calculating Voronoi");
    TIMED_FUNCTION(ProcessGraph process_graph(
                       voronoi_calculator.get_graph(), voronoi_calculator.get_vertex_descriptor_map(),
                       voronoi_calculator.get_added_edges(), program.get<double>("--reduction_proximity"),
                       program.get<double>("--hanging_leaf_threshold"),
                       program.get<double>("--junction_collapse_threshold"),
                       program.get<double>("--junction_smooth_threshold"), colored_image.cols, colored_image.rows,
                       program.get<bool>("--border"), colored_image, program.get<double>("--scale")),
                   "Processing graph");
    TIMED_FUNCTION(
        SurfacesGenerator curves_generator(
            process_graph.get_graph(), program.get<int>("--curve_order"), program.get<int>("--target_curve_order"),
            program.get<double>("--extrusion"), program.get<bool>("--filter_offset_curves"), colored_image,
            program.get<int>("--distance_to_boundary_samples"), program.get<int>("--distance_to_boundary_threshold"),
            program.get<double>("--distance_in_boundary_backoff"), program.get<double>("--distance_in_boundary_factor"),
            program.get<double>("--curve_density"), program.get<int>("--curve_min_length"),
            program.get<double>("--junction_radius_adder")),
        "Generating surfaces");
    TIMED_FUNCTION(curves_generator.write_curves("curves.itd"), "Exporting curves to file");
    TIMED_FUNCTION(curves_generator.write_offset_curves_before_trim("offset_curves_before_trim.itd"),
                   "Exporting offset curves before trim to file");
    TIMED_FUNCTION(curves_generator.write_offset_curves("offset_curves.itd"), "Exporting offset curves to file");
    TIMED_FUNCTION(curves_generator.write_filtered_offset_curves("filtered_offset_curves.itd"),
                   "Exporting filtered offset curves to file");
    TIMED_FUNCTION(curves_generator.write_surfaces("surfaces.itd"), "Exporting surfaces to file");
    TIMED_FUNCTION(curves_generator.write_extrusions("extrusions.itd"), "Exporting extrusions to file");

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << std::endl << "Finished all in: " << elapsed.count() << " seconds" << std::endl;

    exit(0);
    return 0;
}
