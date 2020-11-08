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
    std::string camera_name = "camera";
    if (nh.getParam("camera_name", camera_name)) {
        ROS_INFO("Provided camera name: %s", camera_name.c_str());
    }

    // Default path to camera calibration file is guessed from camera name
    std::string camera_info_url =
        fmt::format("file://{}/camera_info/{}.yaml",
                    ros::package::getPath("tag_utils"), camera_name);

    if (nh.getParam("camera_info_url", camera_info_url)) {
        camera_info_url = fmt::format(
            "file://{}", fs::absolute(camera_info_url).generic_string());
    }

    camera_info_manager::CameraInfoManager c_info_man(nh, camera_name,
                                                      camera_info_url);
    if (!c_info_man.loadCameraInfo(camera_info_url)) {
        ROS_ERROR("Calibration file not found");
        return 1;
    } else {
        ROS_INFO("Camera successfully calibrated");
        c_info = c_info_man.getCameraInfo();
    }

    int capture_width;
    if (!nh.getParam("capture_width", capture_width)) {
        capture_width = c_info.width;
    }

    int capture_height;
    if (!nh.getParam("capture_height", capture_width)) {
        capture_height = c_info.height;
    }

    int display_width;
    if (!nh.getParam("display_width", display_width)) {
        display_width = c_info.width;
    }

    int display_height;
    if (!nh.getParam("display_height", display_height)) {
        display_height = c_info.height;
    }

    int framerate;
    if (!nh.getParam("framerate", framerate)) {
        framerate = 30;
    }

    int flip_method;
    if (!nh.getParam("flip_method", flip_method)) {
        flip_method = 0;
    }

    const std::string pipeline = fmt::format(
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int){}, "
        "height=(int){}, format=(string)NV12, framerate=(fraction){}/1 ! "
        "nvvidconv flip-method={} ! video/x-raw, width=(int){}, "
        "height=(int){}, format=(string)BGRx ! videoconvert ! video/x-raw, "
        "format=(string)BGR ! appsink",
        capture_width, capture_height, framerate, flip_method, display_width,
        display_height);

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
