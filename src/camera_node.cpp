#include <array>
#include <chrono>
#include <iostream>

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <fmt/core.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <boost/filesystem.hpp>

#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include "unistd.h"

namespace fs = boost::filesystem;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "camera");

    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    sensor_msgs::ImagePtr msg;
    sensor_msgs::CameraInfo c_info;

    // Parse camera name argument first
    const std::string default_camera_name = "camera";
    const std::string camera_name =
        nh.param("camera_name", default_camera_name);
    ROS_INFO("Provided camera name: %s", camera_name.c_str());

    // Default path to camera calibration file is guessed from camera name
    const std::string default_camera_info_url =
        fmt::format("file://{}/camera_info/{}.yaml",
                    ros::package::getPath("tag_utils"), camera_name);

    const std::string camera_info_url = fmt::format(
        "file://{}",
        fs::absolute(nh.param("camera_info_url", default_camera_info_url))
            .generic_string());

    camera_info_manager::CameraInfoManager c_info_man(nh, camera_name,
                                                      camera_info_url);
    if (!c_info_man.loadCameraInfo(camera_info_url)) {
        ROS_WARN(
            "Failed to load camera info from %s! No valid camera info will be "
            "published.",
            camera_info_url.c_str());
    } else {
        ROS_INFO("Loaded camera info from %s", camera_info_url.c_str());
        c_info = c_info_man.getCameraInfo();
    }

    const bool use_webcam = nh.param("use_webcam", false);
    const std::string default_device = "/dev/video0";
    const std::string device =
        use_webcam ? nh.param("device", default_device) : "";

    const int capture_width = nh.param<int>("capture_width", c_info.width);
    const int capture_height = nh.param<int>("capture_height", c_info.height);
    const int display_width = nh.param<int>("display_width", capture_width);
    const int display_height = nh.param<int>("display_height", capture_height);
    const int framerate = nh.param<int>("framerate", 30);
    const int flip_method = nh.param<int>("flip_method", 0);

    const std::array<std::string, 2> base_pipe = {
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int){0}, "
        "height=(int){1}, format=(string)NV12, framerate=(fraction){2}/1 ! "
        "nvvidconv flip-method={3} ! video/x-raw, width=(int){4}, "
        "height=(int){5}, format=(string)BGRx ! videoconvert ! video/x-raw, "
        "format=(string)BGR ! appsink",
        "v4l2src device={6} ! video/x-raw, width=(int){0}, "
        "height=(int){1}, framerate=(fraction){2}/1 ! "
        "videoflip method={3} ! video/x-raw, width=(int){4}, "
        "height=(int){5} ! videoconvert ! video/x-raw, "
        "format=(string)BGR ! appsink",
    };

    const std::string pipeline = fmt::format(
        base_pipe[use_webcam], capture_width, capture_height, framerate,
        flip_method, display_width, display_height, device);

    ROS_INFO("Using gstreamer pipeline: \x1B[32m%s", pipeline.c_str());

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera.");
        return 1;
    }
    const auto img_pub_name = fmt::format("/{}/image_rect", camera_name);
    auto img_pub = it.advertise(img_pub_name, 1);
    ROS_INFO("Publishing images to: %s", img_pub_name.c_str());

    const auto info_pub_name = fmt::format("/{}/camera_info", camera_name);
    auto info_pub = nh.advertise<sensor_msgs::CameraInfo>(info_pub_name, 1);
    ROS_INFO("Publishing info to: %s", info_pub_name.c_str());

    ros::Rate loop_rate(framerate);

    cv::Mat frame, gray;
    while (ros::ok()) {
        cap >> frame;

        if (frame.empty()) {
            ROS_ERROR("No images captured!");
            ros::Duration(1.0).sleep();
            continue;
        }
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        auto current_time = ros::Time::now();
        msg =
            cv_bridge::CvImage(std_msgs::Header(), "mono8", gray).toImageMsg();
        msg->header.stamp = current_time;
        c_info.header.stamp = current_time;
        img_pub.publish(msg);
        info_pub.publish(c_info);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
