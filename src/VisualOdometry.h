#pragma once
#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

#include <fub_visual_odometry/VisualOdometryConfig.h>

#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <image_geometry/pinhole_camera_model.h>

namespace fub_visual_odometry {

class VisualOdometry {
 public:
    explicit VisualOdometry(ros::NodeHandle &nh);

 private:

    /**
     * Map topic callback
     * @param msg The map
     */
    void onMap(const nav_msgs::OccupancyGridConstPtr &msg);

    /**
     * Camera callback with image and calibration data
     * @param msg Image data
     * @param info_msg Calibration information data
     */
    void onImage(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info_msg);

    /**
     * Reconfigure callback, when this callback is received the camera pose is recalculated on the next image callback
     * @param config The new config
     * @param level Level counter starting at 0
     */
    void onReconfigure(VisualOdometryConfig &config, uint32_t level);

    /**
     * Transforms a pixel coordinate into map coordinates using inverse reprojection.
     * @param cameraModel The camera model containing the parameters of the camera
     * @param point Pixel coordinates
     * @return A pose with the coordinates in the map frame
     */
    geometry_msgs::Point getMapCoordinates(const image_geometry::PinholeCameraModel &cameraModel,
                                           const cv::Point2d &point,
                                           double height) const;

    /**
     * Calculates the twist from two odometry messages
     * @param front The front point
     * @param rear The rear point
     * @return yaw orientation
     */
    geometry_msgs::Twist getTwist(const nav_msgs::Odometry &last,
                                  const nav_msgs::Odometry &current,
                                  const geometry_msgs::Point &orientationPoint);

    /**
     * Calculates the yaw orientation of two points in a line using atan(front - rear)
     * @param front The front point
     * @param rear The rear point
     * @return yaw orientation
     */
    double getOrientation(const geometry_msgs::Point &front, const geometry_msgs::Point &rear);

    /**
     * Publishes a visualization marker
     * @param point The position
     * @param color Color of the marker
     * @param id Id of the marker for updates
     * @param stamp Timestamp
     */
    void addMarker(visualization_msgs::MarkerArray &markers,
                   const geometry_msgs::Point &point,
                   const std_msgs::ColorRGBA &color,
                   int id,
                   const ros::Time &stamp);

    /**
     * Checks the error of the camera pose by projecting the image coordinates back
     * @param worldPoints The world points from which to calculate the error
     * @param imagePoints The image points to check
     * @param cameraMatrix Intrinsic camera parameters
     * @param distCoeffs Distortion coefficients
     * @param rvec Camera rotation vector
     * @param tvec Camera translation vector
     * @return RMS error of the camera pose
     */
    double checkCameraPose(const std::vector<cv::Point3f> &worldPoints,
                           const std::vector<cv::Point2f> &imagePoints,
                           const cv::Mat &cameraMatrix,
                           const cv::Mat &distCoeffs,
                           const cv::Mat &rvec,
                           const cv::Mat &tvec);

    /**
     * Converts the opencv mat type to a ROS encoding type
     * @param type The opencv mat type
     * @return ROS encoding string describing the image encoding
     */
    static std::string cvTypeToRosType(int type);

    ros::NodeHandle nodeHandle;
    dynamic_reconfigure::Server<VisualOdometryConfig> server;
    dynamic_reconfigure::Server<VisualOdometryConfig>::CallbackType f;

    image_transport::Publisher detectionPublisher;
    ros::Publisher markerPublisher;
    std::unordered_map<int, ros::Publisher> odomPublishers;

    image_transport::CameraSubscriber imageSubscriber;
    ros::Subscriber mapSubscriber;

    tf2_ros::StaticTransformBroadcaster staticTransformBroadcaster;
    tf2_ros::TransformBroadcaster transformBroadcaster;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    tf2::Vector3 markerTranslation;

    VisualOdometryConfig config;
    cv::Ptr<cv::aruco::Dictionary> mapDictionary;
    cv::Ptr<cv::aruco::Dictionary> carDictionary;

    std::vector<int> mapMarkerIds;
    std::vector<std::vector<cv::Point2f>> mapMarkerCorners;

    nav_msgs::OccupancyGridConstPtr map;

    nav_msgs::Odometry lastOdometry;

    bool foundCamera = false;
    cv::Mat1d rotationMatrix;
    cv::Mat1d translationMatrix;
};

}