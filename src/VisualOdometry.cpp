#include <geos_c.h>
#include "VisualOdometry.h"


namespace fub_visual_odometry {

VisualOdometry::VisualOdometry(ros::NodeHandle& nh) : tfListener(tfBuffer) {
    image_transport::ImageTransport it(nh);
    imageSubscriber = it.subscribeCamera("/tracking/cam_left/image_raw", 1, &VisualOdometry::onImage, this, image_transport::TransportHints("compressed"));
    detectionPublisher = it.advertise("detection", 1);
    markerMaskPublisher = it.advertise("marker_mask", 1);
    carMaskPublisher = it.advertise("car_mask", 1);
    markerPublisher = nh.advertise<visualization_msgs::Marker>("marker", 1);
    odomPublisher = nh.advertise<nav_msgs::Odometry>("odom", 1);


    nodeHandle = nh;

    f = boost::bind(&VisualOdometry::onReconfigure, this, _1, _2);
    server.setCallback(f);
}

void VisualOdometry::onReconfigure(VisualOdometryConfig &config, uint32_t level) {
    this->config = config;

    ROS_ERROR("On reconfigure");
}

void VisualOdometry::onImage(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
    cv_bridge::CvImagePtr cvDetectionImage;
    cv_bridge::CvImagePtr cvMarkerMaskImage;
    cv_bridge::CvImagePtr cvCarMaskImage;

    try
    {
        cvDetectionImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cvMarkerMaskImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cvCarMaskImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // separate map makers
    cv::cvtColor(cvMarkerMaskImage->image, cvMarkerMaskImage->image, cv::COLOR_BGR2HSV);
    auto minMarkerHSV = cv::Scalar(config.marker_min_h, config.marker_min_s, config.marker_min_v);
    auto maxMarkerHSV = cv::Scalar(config.marker_max_h, config.marker_max_s, config.marker_max_v);
    cv::inRange(cvMarkerMaskImage->image, minMarkerHSV, maxMarkerHSV, cvMarkerMaskImage->image);
    cvMarkerMaskImage->encoding = cvTypeToRosType(cvMarkerMaskImage->image.type());
    std::vector<std::vector<cv::Point>> mapMarkerContours;
    findContours(cvMarkerMaskImage->image, mapMarkerContours);

    // separate car markers
    cv::cvtColor(cvCarMaskImage->image, cvCarMaskImage->image, cv::COLOR_BGR2HSV);
    auto minCarHSV = cv::Scalar(config.car_min_h, config.car_min_s, config.car_min_v);
    auto maxCarHSV = cv::Scalar(config.car_max_h, config.car_max_s, config.car_max_v);
    cv::inRange(cvCarMaskImage->image, minCarHSV, maxCarHSV, cvCarMaskImage->image);
    cvCarMaskImage->encoding = cvTypeToRosType(cvCarMaskImage->image.type());
    std::vector<std::vector<cv::Point>> carMarkerContours;
    findContours(cvCarMaskImage->image, carMarkerContours);

    std::vector<Circle> carMarkers;
    findBestMarkers(cvCarMaskImage->image, carMarkers, 1);
    for (const auto& marker : carMarkers) {
        cv::circle(cvDetectionImage->image, marker.center, static_cast<int>(marker.radius), cv::Scalar(255, 255, 255), 2);
    }

    std::vector<Circle> mapMarkers;
    std::vector<cv::Point2f> imageCoordinates;
    findBestMarkers(cvMarkerMaskImage->image, mapMarkers, 4);

    std::sort( mapMarkers.begin( ), mapMarkers.end( ), [ ]( const Circle& lhs, const Circle& rhs )
    {
        return lhs.center.x < rhs.center.x;
    });

    int i =1;
    for (const auto& marker : mapMarkers) {
        imageCoordinates.emplace_back(marker.center);
        cv::circle(cvDetectionImage->image, marker.center, static_cast<int>(marker.radius), cv::Scalar(255, 255, 255), 2);
        cv::putText(cvDetectionImage->image, cv::format("%d", i++), marker.center, cv::FONT_HERSHEY_PLAIN, 12, cv::Scalar(255, 255, 255), 2);
    }

    std::vector<cv::Point3f> worldCoordinates = {
        {3.44, 0.04, 0}, {5.49, 0.27, 0}, {5.38, 3.96, 0}, {3.40, 3.95, 0}
    };

    if (mapMarkers.size() == 4) {
        image_geometry::PinholeCameraModel cameraModel;
        cameraModel.fromCameraInfo(info_msg);

        cv::Mat1d rvec = cv::Mat1d::zeros(3, 1);
        cv::Mat1d tvec = cv::Mat1d::zeros(3, 1);

        cv::solvePnP(worldCoordinates,
                     imageCoordinates,
                     cameraModel.intrinsicMatrix(),
                     cameraModel.distortionCoeffs(),
                     rvec,
                     tvec,
                     false,
                     cv::SOLVEPNP_EPNP);

        cv::Mat1d rod = cv::Mat1d::zeros(3, 3);
        cv::Rodrigues(rvec, rod);

        rod = rod.t();
        tvec = -rod * tvec;

        tf2::Matrix3x3 cameraRotation(rod(0, 0), rod(0, 1), rod(0, 2),
                                      rod(1, 0), rod(1, 1), rod(1, 2),
                                      rod(2, 0), rod(2, 1), rod(2, 2));

        tf2::Vector3 cameraTranslation(tvec(0, 0), tvec(1, 0), tvec(2, 0));

        geometry_msgs::TransformStamped cameraTransform;
        cameraTransform.header = msg->header;
        cameraTransform.header.stamp = ros::Time::now();
        cameraTransform.header.frame_id = "map";
        cameraTransform.child_frame_id = "cam_left";
        geometry_msgs::Vector3 m;
        tf2::convert(cameraTranslation, m);
        cameraTransform.transform.translation = m;
        geometry_msgs::Quaternion q;
        tf2::Quaternion tfq;
        cameraRotation.getRotation(tfq);
        tf2::convert(tfq, q);
        cameraTransform.transform.rotation = q;

        transformBroadcaster.sendTransform(cameraTransform);

        cv::Mat1d A(3,3);
        A << cameraTranslation.z(), 0, -cameraTranslation.x(), 0, cameraTranslation.z(), -cameraTranslation.y(), 0, 0, -1;
        cv::Mat1d mat = A * rod * cv::Mat(cameraModel.intrinsicMatrix().inv());
        cv::Mat1d p(3,1);
        p << carMarkers[0].center.x, carMarkers[0].center.y, 1.0;

        cv::Mat1d p2 = mat * p;
        p2 /= p2(2,0);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "car_coordinate";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = p2[0][0];
        marker.pose.position.y = p2[1][0];
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        markerPublisher.publish(marker);


        nav_msgs::Odometry odometry;
        odometry.header.frame_id = "/map";
        odometry.header.stamp = ros::Time::now();
        odometry.pose.pose.position.x = p2[0][0];
        odometry.pose.pose.position.y = p2[1][0];
        odometry.pose.pose.position.z = 0;
        odometry.pose.pose.orientation.x = 0.0;
        odometry.pose.pose.orientation.y = 0.0;
        odometry.pose.pose.orientation.z = 0.0;
        odometry.pose.pose.orientation.w = 1.0;

        odomPublisher.publish(odometry);
    }

    detectionPublisher.publish(cvDetectionImage->toImageMsg());
    carMaskPublisher.publish(cvCarMaskImage->toImageMsg());
    markerMaskPublisher.publish(cvMarkerMaskImage->toImageMsg());
}

void VisualOdometry::findContours(const cv::Mat &image, cv::OutputArrayOfArrays contours) const {
    cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
}

void VisualOdometry::findBestMarkers(const cv::Mat &image, std::vector<Circle> &markers, int n) const {
    std::vector<std::vector<cv::Point>> contours;
    findContours(image, contours);

    std::map<double, std::vector<cv::Point>> contoursByArea;
    for (auto& contour : contours) {
        auto contourArea = cv::contourArea(contour);

        contoursByArea[contourArea] = contour;
    }

    int i = 1;
    for(auto it = contoursByArea.rbegin(); it != contoursByArea.rend(); ++it, i++) {
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(it->second, center, radius);
        markers.emplace_back(center, radius);
        cv::circle(image, center, static_cast<int>(radius), cv::Scalar(255, 255, 255), 2);

        if (i >= n) {
            break;
        }
    }
}

std::string VisualOdometry::cvTypeToRosType(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}
}
