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

struct Circle {
    cv::Point center;
    float radius;

    Circle(const cv::Point& center, float radius) : center(center), radius(radius) {};
};

class VisualOdometry {
 public:
    explicit VisualOdometry(ros::NodeHandle &nh);

 private:

    void onMap(const nav_msgs::OccupancyGridConstPtr &msg);
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
    geometry_msgs::Point getMapCoordinates(const image_geometry::PinholeCameraModel& cameraModel, const cv::Point2d& point, double height) const;


    /**
     * Calculates the yaw orientation of two points in a line using atan(front - rear)
     * @param front The front point
     * @param rear The rear point
     * @return yaw orientation
     */
    double getOrientation(const geometry_msgs::Point& front, const geometry_msgs::Point& rear);

    /**
     * Publishes a visualization marker
     * @param point The position
     * @param color Color of the marker
     * @param id Id of the marker for updates
     * @param stamp Timestamp
     */
    void addMarker(visualization_msgs::MarkerArray& markers, const geometry_msgs::Point& point, const std_msgs::ColorRGBA& color, int id, const ros::Time& stamp);


    static std::string cvTypeToRosType(int type);

    ros::NodeHandle nodeHandle;
    dynamic_reconfigure::Server<VisualOdometryConfig> server;
    dynamic_reconfigure::Server<VisualOdometryConfig>::CallbackType f;

    image_transport::Publisher detectionPublisher;
    ros::Publisher markerPublisher;
    ros::Publisher odomPublisher;

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

    bool foundCamera = false;
    cv::Mat1d rotationMatrix;
    cv::Mat1d translationMatrix;
};

}