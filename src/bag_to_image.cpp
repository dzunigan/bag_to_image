
#define PROGRAM_NAME \
    "bag_to_image"

#define FLAGS_CASES                                                                                \
    FLAG_CASE(string, output_path, "", "Output images path")                                       \
    FLAG_CASE(string, images_dir, "data", "Output images path")                                    \
    FLAG_CASE(string, timestamps_file, "data.csv", "Output timestamps file")                       \
    FLAG_CASE(uint64, compression_level, 9, "PNG compression level (0-9)")                         \
    FLAG_CASE(bool, show_images, false, "Preview images while extracting them")

#define ARGS_CASES                                                                                 \
    ARG_CASE(rosbag)                                                                               \
    ARG_CASE(topic)

#include <iostream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/Image.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>

#include "args.hpp"
#include "io.hpp"
#include "macros.h"

static const std::string OPENCV_WINDOW = "Image window";

inline std::string to_string(size_t n, int w) {

    std::stringstream ss;
    ss << std::setw(w) << std::setfill('0') << n;
    return ss.str();
}

void ValidateFlags() {
    if (!FLAGS_output_path.empty()) {
        RUNTIME_ASSERT(boost::filesystem::is_directory(FLAGS_output_path));
    } else {
        FLAGS_output_path = boost::filesystem::current_path().string();
    }
    boost::filesystem::create_directory(boost::filesystem::path(FLAGS_output_path) / FLAGS_images_dir);
    RUNTIME_ASSERT(boost::filesystem::is_empty(boost::filesystem::path(FLAGS_output_path) / FLAGS_images_dir));
    RUNTIME_ASSERT(!boost::filesystem::exists(boost::filesystem::path(FLAGS_output_path) / FLAGS_timestamps_file));
    RUNTIME_ASSERT(FLAGS_compression_level < 10);
//    if (FLAGS_n == 0) {
//        FLAGS_n = std::numeric_limits<std::size_t>::max();
//    }
}

void ValidateArgs() {
    RUNTIME_ASSERT(boost::filesystem::is_regular_file(ARGS_rosbag));
}

int main(int argc, char* argv[]) {

    // Handle help flag
    if (args::HelpRequired(argc, argv)) {
        args::ShowHelp();
        return 0;
    }

    // Parse input flags
    args::ParseCommandLineNonHelpFlags(&argc, &argv, true);

    // Check number of args
    if (argc-1 != args::NumArgs()) {
        args::ShowHelp();
        return -1;
    }

    // Parse input args
    args::ParseCommandLineArgs(argc, argv);

    // Validate input arguments
    ValidateFlags();
    ValidateArgs();

    rosbag::Bag bag;
    bag.open(ARGS_rosbag, rosbag::bagmode::Read);

    std::vector<std::string> topics = {ARGS_topic};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // Prepare output
    boost::filesystem::path output_path(FLAGS_output_path);
    boost::filesystem::path images_dir(FLAGS_images_dir);
    boost::filesystem::path timestamp_file(FLAGS_timestamps_file);

    std::vector<int> compression_params = {CV_IMWRITE_PNG_COMPRESSION, static_cast<int>(FLAGS_compression_level)};
    size_t num_images = std::distance(view.begin(), view.end());
    //RUNTIME_ASSERT(FLAGS_o < num_images);
    //size_t n = std::min(FLAGS_n, static_cast<std::size_t>(std::ceil(static_cast<double>(num_images - FLAGS_o) / static_cast<double>(FLAGS_s + 1))));
    size_t n = num_images;
    //int w = std::to_string(n).size() + 1;

    io::Records records;
    rosbag::View::iterator it = view.begin();
    //std::advance(it, FLAGS_o);
    for(size_t ctr = 0; ctr < n; ++ctr) {
        std::cout << "\r" << (ctr + 1) << " / " << n << std::flush;
        const rosbag::MessageInstance& m = *it;
        sensor_msgs::Image::ConstPtr msg = m.instantiate<sensor_msgs::Image>();
        if (msg != nullptr) {
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return -1;
            }

            if (FLAGS_show_images) {
                cv::imshow(OPENCV_WINDOW, cv_ptr->image);
                cv::waitKey(3);
            }

            unsigned long ns = static_cast<unsigned long>(msg->header.stamp.toSec() * 1e9);

            std::string image_name = std::to_string(ns) + ".png";
            boost::filesystem::path image_path = output_path / images_dir / image_name;
            cv::rotate(cv_ptr->image, cv_ptr->image, 1);
            cv::imwrite(image_path.string(), cv_ptr->image, compression_params);

            records.emplace_back(std::make_pair(ns, image_name));

            //std::advance(it, FLAGS_s + 1);
            std::advance(it, 1);
        }
    }
    std::cout << std::endl;

    bag.close();

    io::write_file(records, (output_path / timestamp_file).string());

    return 0;
}
