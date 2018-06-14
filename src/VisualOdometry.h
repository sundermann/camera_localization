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

    /**
     * Finds contours inside an image. It will apply 3x3 erosion and dilation to the image to reduce noise to speed up calculation
     * @param image Image to detect the contours in. Will be modified with erosion and dilation
     * @param contours The detected contours without any hierarchy
     */
    void findContours(const cv::Mat &image, cv::OutputArrayOfArrays contours) const;

    /**
     * Detects markers in the image using contours. The found contours are sorted by their area and will be returned in
     * the makers param. A circle is fitted onto the found contours.
     * @param image The image to detect markers in. This should be a masked image
     * @param markers The found circle markers will be returned here
     * @param n Maximum number of markers to detect
     */
    void findBestMarkers(const cv::Mat &image, std::vector<Circle> &markers, int n) const;

    /**
     * Transforms a pixel coordinate into map coordinates using inverse reprojection.
     * @param cameraModel The camera model containing the parameters of the camera
     * @param point Pixel coordinates
     * @return A pose with the coordinates in the map frame
     */
    geometry_msgs::Pose getMapCoordinates(const image_geometry::PinholeCameraModel& cameraModel, const cv::Point2d& point) const;

    /**
     * Calculates the yaw orientation of two points in a line using atan(front - rear)
     * @param front The front point
     * @param rear The rear point
     * @return yaw orientation
     */
    double getOrientation(const geometry_msgs::Pose& front, const geometry_msgs::Pose& rear);


    static std::string cvTypeToRosType(int type);

    ros::NodeHandle nodeHandle;
    dynamic_reconfigure::Server<VisualOdometryConfig> server;
    dynamic_reconfigure::Server<VisualOdometryConfig>::CallbackType f;

    image_transport::Publisher markerMaskPublisher;
    image_transport::Publisher carFrontMaskPublisher;
    image_transport::Publisher carRearMaskPublisher;
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


    bool foundCamera = false;
    cv::Mat1d rotationMatrix;
    cv::Mat1d translationMatrix;
};

}