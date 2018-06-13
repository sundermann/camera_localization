#pragma once
#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <fub_visual_odometry/VisualOdometryConfig.h>


#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>

namespace fub_visual_odometry {

struct Circle {
    cv::Point center;
    float radius;

    Circle(const cv::Point& center, float radius) : center(center), radius(radius) {};
};

class VisualOdometry {
 public:
    explicit VisualOdometry(ros::NodeHandle &nh);

 private:
    void onImage(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info_msg);
    void onReconfigure(VisualOdometryConfig &config, uint32_t level);

    void findContours(const cv::Mat &image, cv::OutputArrayOfArrays contours) const;
    void findBestMarkers(const cv::Mat &image, std::vector<Circle> &markers, int n) const;

    static std::string cvTypeToRosType(int type);

    ros::NodeHandle nodeHandle;
    dynamic_reconfigure::Server<VisualOdometryConfig> server;
    dynamic_reconfigure::Server<VisualOdometryConfig>::CallbackType f;

    image_transport::Publisher markerMaskPublisher;
    image_transport::Publisher carMaskPublisher;
    image_transport::Publisher detectionPublisher;
    ros::Publisher markerPublisher;
    ros::Publisher odomPublisher;
    image_transport::CameraSubscriber imageSubscriber;

    tf2_ros::StaticTransformBroadcaster transformBroadcaster;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    VisualOdometryConfig config;
    std::vector<Circle> foundMarkers;
    geometry_msgs::TransformStamped transformation;
};

}