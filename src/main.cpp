#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>

ros::Publisher publisher_left;
ros::Publisher publisher_right;

sensor_msgs::CameraInfo left_camera_info_msg;
sensor_msgs::CameraInfo right_camera_info_msg;

std::string frameId;

sensor_msgs::CameraInfo yaml_to_CameraInfo(const std::string& yaml_fname) {
    YAML::Node calib_data = YAML::LoadFile(yaml_fname);

    sensor_msgs::CameraInfo camera_info_msg;
    camera_info_msg.width = calib_data["image_width"].as<uint32_t>();
    camera_info_msg.height = calib_data["image_height"].as<uint32_t>();

    // Assigning K
    std::vector<double> K = calib_data["camera_matrix"]["data"].as<std::vector<double>>();
    for (size_t i = 0; i < 9; ++i) {
        camera_info_msg.K[i] = K[i];
    }

    // Assigning D
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"].as<std::vector<double>>();

    // Assigning R
    std::vector<double> R = calib_data["rectification_matrix"]["data"].as<std::vector<double>>();
    for (size_t i = 0; i < 9; ++i) {
        camera_info_msg.R[i] = R[i];
    }

    // Assigning P
    std::vector<double> P = calib_data["projection_matrix"]["data"].as<std::vector<double>>();
    for (size_t i = 0; i < 12; ++i) {
        camera_info_msg.P[i] = P[i];
    }

    camera_info_msg.distortion_model = calib_data["distortion_model"].as<std::string>();

    return camera_info_msg;
}

void leftImageCallback(const sensor_msgs::ImageConstPtr& image) {
    left_camera_info_msg.header = image->header;
    if (!frameId.empty()) {
        left_camera_info_msg.header.frame_id = frameId;
    }
    publisher_left.publish(left_camera_info_msg);
}

void rightImageCallback(const sensor_msgs::ImageConstPtr& image) {
    right_camera_info_msg.header = image->header;
    if (!frameId.empty()) {
        right_camera_info_msg.header.frame_id = frameId;
    }
    publisher_right.publish(right_camera_info_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "yaml_to_camera_info");
    ros::NodeHandle nh;

    std::string left_yaml_path;
    std::string right_yaml_path;

    nh.param<std::string>("yaml_path", left_yaml_path, "/opt/ros/noetic/share/rtabmap_examples/launch/config/euroc_left.yaml");
    nh.param<std::string>("yaml_path", right_yaml_path, "/opt/ros/noetic/share/rtabmap_examples/launch/config/euroc_right.yaml");

    if (left_yaml_path.empty() || right_yaml_path.empty()) {
        ROS_ERROR("yaml_path parameter should be set to path of the calibration file!");
        return 1;
    }

    nh.param<std::string>("frame_id", frameId, "cam0");

    left_camera_info_msg = yaml_to_CameraInfo(left_yaml_path);
    right_camera_info_msg = yaml_to_CameraInfo(right_yaml_path);

    publisher_left = nh.advertise<sensor_msgs::CameraInfo>("/stereo_camera/left/camera_info", 1);
    publisher_right = nh.advertise<sensor_msgs::CameraInfo>("/stereo_camera/right/camera_info", 1);
    
    ros::Subscriber leftSubscriber = nh.subscribe("/cam1/image_raw", 1, leftImageCallback);
    ros::Subscriber rightSubscriber = nh.subscribe("/cam0/image_raw", 1, rightImageCallback);

    ros::spin();

    return 0;
}