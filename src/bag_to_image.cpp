
#define PROGRAM_NAME \
    "bag_to_image"

#define FLAGS_CASES                                                                                \
    FLAG_CASE(string, output_path, "", "Output images path")                                       \
    FLAG_CASE(string, images_dir, "images", "Output images path")                                  \
    FLAG_CASE(string, timestamps_file, "timestamps.txt", "Output timestamps file")                 \
    FLAG_CASE(uint64, compression_level, 9, "PNG compression level (0-9)")                         \
    FLAG_CASE(bool, show_images, false, "Preview images while extracting them")

#define ARGS_CASES                                                                                 \
    ARG_CASE(rosbag)                                                                               \
    ARG_CASE(topic)

#include <iostream>
#include <iterator>
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

void ValidateFlags() {
    if (!FLAGS_output_path.empty()) {
        RUNTIME_ASSERT(boost::filesystem::is_directory(FLAGS_output_path));
    } else {
        FLAGS_output_path = boost::filesystem::current_path().string();
    }
    boost::filesystem::create_directory(boost::filesystem::path(FLAGS_output_path) / FLAGS_images_dir);
    RUNTIME_ASSERT(boost::filesystem::is_empty(FLAGS_images_dir));
    RUNTIME_ASSERT(!boost::filesystem::exists(FLAGS_timestamps_file));
    RUNTIME_ASSERT(FLAGS_compression_level < 10);
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
    std::size_t n = std::distance(view.begin(), view.end());
    int w = std::to_string(n).size() + 1;

    int ctr = 0;
    io::Records records;
    for(rosbag::View::iterator it = view.begin(); it != view.end(); ++it) {
        std::cout << "\r" << std::distance(view.begin(), it) + 1 << " / " << n << std::flush;
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

            std::string image_name = io::to_string(ctr, w) + ".png";
            boost::filesystem::path image_path = output_path / images_dir / image_name;
            cv::imwrite(image_path.string(), cv_ptr->image, compression_params);

            records.emplace_back(std::make_pair(msg->header.stamp.toSec(), (images_dir / image_name).string()));

            ++ctr;
        }
    }
    std::cout << std::endl;

    bag.close();

    io::write_file(records, timestamp_file.string());

    return 0;
}
