#include <ros/ros.h>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "transform_backend");

    ros::NodeHandle nh;

    ros::Subscriber sub_tf_;
    tf2_ros::TransformBroadcaster broadcaster;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener(tf_buffer);
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/pose_tag_body", 1);

    std::string body_frame_name;
    if (!nh.getParam("/body_frame", body_frame_name))
    {
        ROS_INFO("Using default body frame name: %s", body_frame_name.c_str());
    }
    else
    {
        ROS_INFO("Got body frame name: %s", body_frame_name.c_str());
    }

    // Read transformation matrix from transforms.yaml into std::vector
    std::vector<double> vec_camera_body;
    if (!nh.getParam("/transform_body_to_camera", vec_camera_body))
    {
        ROS_ERROR("No transformation matrix from camera to body provided");
    }

    if (vec_camera_body.size() != 16)
    {
        ROS_ERROR("Not a valid transformation matrix from camera to body. Has %lu elements", vec_camera_body.size());
    }

    // Assuming transformation matrix is listed in transforms.yaml in row-major order
    // Use Eigen::Map to move data into Eigen::Matrix using Stride parameter to account for row-major order
    Eigen::Matrix4d mat_camera_body = Eigen::Map<Eigen::Matrix4d, Eigen::Unaligned, Eigen::Stride<1, 4>>(vec_camera_body.data());
    ROS_INFO("Got translation of camera frame relative to body frame %.3f m-x, %.3f m-y, %.3f m-z", mat_camera_body(0, 3), mat_camera_body(1, 3), mat_camera_body(2, 3));

    // Convert rotation of camera frame relative to body frame into euler angles for display
    Eigen::Vector3d eul(mat_camera_body.topLeftCorner<3, 3>().eulerAngles(3, 2, 1));
    ROS_INFO("Got rotation of camera frame relative to body frame %.3f deg-x, %.3f deg-y, %.3f-deg z", eul(0) * 57.296, eul(1) * 57.296, eul(2) * 57.296);

    // Convert transformation matrix into TransformStamped to work with tf2
    geometry_msgs::TransformStamped tf_camera_body = tf2::eigenToTransform(Eigen::Isometry3d(mat_camera_body));
    tf_camera_body.header.frame_id = body_frame_name;
    tf_camera_body.child_frame_id = "camera";

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        tf_camera_body.header.stamp = ros::Time::now();
        broadcaster.sendTransform(tf_camera_body);

        ros::spinOnce();
        loop_rate.sleep();
    }
}