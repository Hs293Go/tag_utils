#include <functional>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32MultiArray.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "transform_backend");

    ros::NodeHandle nh;

    std::string body_frame_name;
    if (!nh.getParam("/body_frame", body_frame_name)) {
        ROS_INFO("Using default body frame name: %s", body_frame_name.c_str());
    } else {
        ROS_INFO("Got body frame name: %s", body_frame_name.c_str());
    }

    // Read transformation matrix from transforms.yaml into std::vector
    std::vector<double> quat_tmp, pos_tmp;
    if (!nh.getParam("/rotation", quat_tmp)) {
        ROS_ERROR("No rotation from camera to body provided");
    }

    if (!nh.getParam("/translation", pos_tmp)) {
        ROS_ERROR("No translation from body to camera provided");
    }

    if (quat_tmp.size() != 4) {
        ROS_ERROR(
            "Not a valid Quaternion representing rotation from camera to body. "
            "Has %lu elements",
            quat_tmp.size());
    }

    if (pos_tmp.size() != 3) {
        ROS_ERROR(
            "Not a valid vector representing translation from body to camera. "
            "Has %lu elements",
            pos_tmp.size());
    }

    const Eigen::Map<const Eigen::Quaterniond> q_c_b(quat_tmp.data());
    const Eigen::Map<const Eigen::Vector3d> r_c_b_c(pos_tmp.data());

    ROS_INFO(
        "Got translation of camera frame relative to body frame [%.3f, %.3f, "
        "%.3f] m",
        -pos_tmp[0], -pos_tmp[1], -pos_tmp[2]);

    // Convert rotation of camera frame relative to body frame into euler angles
    // for display
    constexpr double k180ByPi = 57.29577951;

    const Eigen::Vector3d eul =
        k180ByPi * q_c_b.toRotationMatrix().eulerAngles(2, 1, 0);

    ROS_INFO(
        "Got rotation of body frame relative to camera frame %.3f deg-x, %.3f "
        "deg-y, %.3f-deg z",
        eul[0], eul[1], eul[2]);

    ros::Publisher pose_publisher =
        nh.advertise<geometry_msgs::PoseStamped>("/pose_tag_body", 1);

    ros::Subscriber pose_subscriber =
        nh.subscribe<apriltag_ros::AprilTagDetectionArray>(
            "/tag_detections", 100, [&](const auto &msg) {
                using Eigen::Map;
                using Eigen::Vector3d;
                using Eigen::Quaterniond;

                if (msg->detections.size() > 1) {
                    ROS_WARN("Detection for %lu tags is not implemented",
                             msg->detections.size());
                    return;
                }

                if (msg->detections.size() == 0) {
                    return;
                }
                geometry_msgs::PoseStamped tf_tag_body;

                const auto &T_p_c = msg->detections[0].pose.pose.pose;

                const Map<const Vector3d> r_c_p_c(&T_p_c.position.x);
                const Map<const Quaterniond> q_p_c(&T_p_c.orientation.x);

                Map<Vector3d> r_b_p_b(&tf_tag_body.pose.position.x);
                Map<Quaterniond> q_p_b(&tf_tag_body.pose.orientation.x);

                q_p_b = q_p_c * q_c_b;
                r_b_p_b = q_c_b.inverse() * (r_c_p_c - r_c_b_c);

                tf_tag_body.header = msg->detections[0].pose.header;

                pose_publisher.publish(tf_tag_body);
            });

    ros::Rate loop_rate(30);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
