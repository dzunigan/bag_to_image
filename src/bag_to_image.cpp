
#define PROGRAM_NAME \
    "bag_to_image"

#define FLAGS_CASES                                                                                \
    FLAG_CASE(string, images_path, "images", "Output images path")                                 \
    FLAG_CASE(string, timestamps_file, "timestamps.txt", "Output timestamps file")                 \
    FLAG_CASE(bool, show_images, true, "Preview images while extracting them")

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
    boost::filesystem::create_directory(FLAGS_images_path);
    RUNTIME_ASSERT(boost::filesystem::is_empty(FLAGS_images_path));
    RUNTIME_ASSERT(boost::filesystem::exists(FLAGS_timestamps_file));
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
    boost::filesystem::path output_dir(FLAGS_images_path);
    boost::filesystem::path timestamp_file(FLAGS_timestamps_file);

    std::vector<int> compression_params = {CV_IMWRITE_PNG_COMPRESSION, 9};
    int w = std::to_string(std::distance(view.begin(), view.end())).size() + 1;

    int ctr = 0;
    io::Records records;
    for(rosbag::MessageInstance const m: view) {
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
            boost::filesystem::path image_path = output_dir / image_name;
            cv::imwrite(image_path.string(), cv_ptr->image, compression_params);

            records.emplace_back(std::make_pair(msg->header.stamp.toSec(), image_name));

            ++ctr;
        }
    }

    bag.close();

    io::write_file(records, timestamp_file.string());

    return 0;
}
