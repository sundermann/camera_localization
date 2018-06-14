#include <geos_c.h>
#include "VisualOdometry.h"


namespace fub_visual_odometry {

VisualOdometry::VisualOdometry(ros::NodeHandle& nh) : tfListener(tfBuffer) {
    image_transport::ImageTransport it(nh);
    imageSubscriber = it.subscribeCamera("/tracking/cam_left/image_raw", 1, &VisualOdometry::onImage, this, image_transport::TransportHints("compressed"));
    detectionPublisher = it.advertise("detection", 1);
    markerMaskPublisher = it.advertise("marker_mask", 1);
    carFrontMaskPublisher = it.advertise("car_front_mask", 1);
    carRearMaskPublisher = it.advertise("car_rear_mask", 1);
    markerPublisher = nh.advertise<visualization_msgs::Marker>("marker", 1);
    odomPublisher = nh.advertise<nav_msgs::Odometry>("odom", 1);


    nodeHandle = nh;

    f = boost::bind(&VisualOdometry::onReconfigure, this, _1, _2);
    server.setCallback(f);
}

void VisualOdometry::onReconfigure(VisualOdometryConfig &config, uint32_t level) {
    this->config = config;

    // Find the camera again with the new parameters
    foundCamera = false;
}

void VisualOdometry::onImage(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
    cv_bridge::CvImagePtr cvDetectionImage;
    cv_bridge::CvImagePtr cvMapMarkerMaskImage;
    cv_bridge::CvImagePtr cvCarFrontMarkerImage;
    cv_bridge::CvImagePtr cvCarRearMarkerImage;

    try
    {
        cvDetectionImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cvMapMarkerMaskImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cvCarFrontMarkerImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cvCarRearMarkerImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    image_geometry::PinholeCameraModel cameraModel;
    cameraModel.fromCameraInfo(info_msg);

    // separate map makers
    cv::cvtColor(cvMapMarkerMaskImage->image, cvMapMarkerMaskImage->image, cv::COLOR_BGR2HSV);
    auto minMarkerHSV = cv::Scalar(config.map_marker_min_h, config.map_marker_min_s, config.map_marker_min_v);
    auto maxMarkerHSV = cv::Scalar(config.map_marker_max_h, config.map_marker_max_s, config.map_marker_max_v);
    cv::inRange(cvMapMarkerMaskImage->image, minMarkerHSV, maxMarkerHSV, cvMapMarkerMaskImage->image);
    cvMapMarkerMaskImage->encoding = cvTypeToRosType(cvMapMarkerMaskImage->image.type());
    std::vector<std::vector<cv::Point>> mapMarkerContours;
    findContours(cvMapMarkerMaskImage->image, mapMarkerContours);

    // separate front car markers
    cv::cvtColor(cvCarFrontMarkerImage->image, cvCarFrontMarkerImage->image, cv::COLOR_BGR2HSV);
    auto minFrontCarHSV = cv::Scalar(config.car_front_marker_min_h, config.car_front_marker_min_s, config.car_front_marker_min_v);
    auto maxFrontCarHSV = cv::Scalar(config.car_front_marker_max_h, config.car_front_marker_max_s, config.car_front_marker_max_v);
    cv::inRange(cvCarFrontMarkerImage->image, minFrontCarHSV, maxFrontCarHSV, cvCarFrontMarkerImage->image);
    cvCarFrontMarkerImage->encoding = cvTypeToRosType(cvCarFrontMarkerImage->image.type());
    std::vector<std::vector<cv::Point>> carFrontMarkerContours;
    findContours(cvCarFrontMarkerImage->image, carFrontMarkerContours);

    // separate rear car markers
    cv::cvtColor(cvCarRearMarkerImage->image, cvCarRearMarkerImage->image, cv::COLOR_BGR2HSV);
    auto minRearCarHSV = cv::Scalar(config.car_rear_marker_min_h, config.car_rear_marker_min_s, config.car_rear_marker_min_v);
    auto maxRearCarHSV = cv::Scalar(config.car_rear_marker_max_h, config.car_rear_marker_max_s, config.car_rear_marker_max_v);
    cv::inRange(cvCarRearMarkerImage->image, minRearCarHSV, maxRearCarHSV, cvCarRearMarkerImage->image);
    cvCarRearMarkerImage->encoding = cvTypeToRosType(cvCarRearMarkerImage->image.type());
    std::vector<std::vector<cv::Point>> carRearMarkerContours;
    findContours(cvCarRearMarkerImage->image, carRearMarkerContours);

    std::vector<Circle> carFrontMarkers;
    findBestMarkers(cvCarFrontMarkerImage->image, carFrontMarkers, 1);
    for (const auto& marker : carFrontMarkers) {
        cv::circle(cvDetectionImage->image, marker.center, static_cast<int>(marker.radius), cv::Scalar(255, 255, 255), 2);
    }

    std::vector<Circle> carRearMarkers;
    findBestMarkers(cvCarRearMarkerImage->image, carRearMarkers, 1);
    for (const auto& marker : carRearMarkers) {
        cv::circle(cvDetectionImage->image, marker.center, static_cast<int>(marker.radius), cv::Scalar(255, 255, 255), 2);
    }

    std::vector<Circle> mapMarkers;
    std::vector<cv::Point2f> imageCoordinates;
    findBestMarkers(cvMapMarkerMaskImage->image, mapMarkers, 4);

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

    if (foundCamera) {
        auto frontPoint = getMapCoordinates(cameraModel, carFrontMarkers[0].center);
        auto rearPoint = getMapCoordinates(cameraModel, carRearMarkers[0].center);

        visualization_msgs::Marker frontMarker;
        frontMarker.header.frame_id = "map";
        frontMarker.header.stamp = ros::Time::now();
        frontMarker.ns = "car_coordinate";
        frontMarker.id = 0;
        frontMarker.type = visualization_msgs::Marker::SPHERE;
        frontMarker.action = visualization_msgs::Marker::ADD;
        frontMarker.pose = frontPoint;
        frontMarker.pose.position.z = 0;
        frontMarker.pose.orientation.x = 0.0;
        frontMarker.pose.orientation.y = 0.0;
        frontMarker.pose.orientation.z = 0.0;
        frontMarker.pose.orientation.w = 1.0;
        frontMarker.scale.x = 0.1;
        frontMarker.scale.y = 0.1;
        frontMarker.scale.z = 0.1;
        frontMarker.color.a = 1.0;
        frontMarker.color.r = 0.0;
        frontMarker.color.g = 0.0;
        frontMarker.color.b = 1.0;
        markerPublisher.publish(frontMarker);

        visualization_msgs::Marker rearMarker;
        rearMarker.header.frame_id = "map";
        rearMarker.header.stamp = ros::Time::now();
        rearMarker.ns = "car_coordinate";
        rearMarker.id = 1;
        rearMarker.type = visualization_msgs::Marker::SPHERE;
        rearMarker.action = visualization_msgs::Marker::ADD;
        rearMarker.pose = rearPoint;
        rearMarker.pose.position.z = 0;
        rearMarker.pose.orientation.x = 0.0;
        rearMarker.pose.orientation.y = 0.0;
        rearMarker.pose.orientation.z = 0.0;
        rearMarker.pose.orientation.w = 1.0;
        rearMarker.scale.x = 0.1;
        rearMarker.scale.y = 0.1;
        rearMarker.scale.z = 0.1;
        rearMarker.color.a = 1.0;
        rearMarker.color.r = 1.0;
        rearMarker.color.g = 0.0;
        rearMarker.color.b = 0.0;
        markerPublisher.publish(rearMarker);

        nav_msgs::Odometry odometry;
        odometry.header.frame_id = "map";
        odometry.header.stamp = ros::Time::now();
        odometry.pose.pose = frontPoint;
        odometry.pose.pose.position.z = 0;
        auto yaw = getOrientation(frontPoint, rearPoint);

        geometry_msgs::Quaternion orientation;
        orientation.x = 0;
        orientation.y = 0;
        orientation.z = sin(yaw / 2);
        orientation.w = cos(yaw / 2);

        odometry.pose.pose.orientation = orientation;

        odomPublisher.publish(odometry);
    } else if (mapMarkers.size() == 4) {
        std::vector<cv::Point3f> worldCoordinates = {
            {3.44, 0.04, 0}, {5.49, 0.27, 0}, {5.38, 3.79, 0}, {3.40, 3.80, 0}
        };

        cv::Mat1d rvec = cv::Mat1d::zeros(3, 1);
        translationMatrix = cv::Mat1d::zeros(3, 1);

        cv::solvePnP(worldCoordinates,
                     imageCoordinates,
                     cameraModel.intrinsicMatrix(),
                     cameraModel.distortionCoeffs(),
                     rvec,
                     translationMatrix,
                     false,
                     cv::SOLVEPNP_EPNP);

        rotationMatrix = cv::Mat1d::zeros(3, 3);
        cv::Rodrigues(rvec, rotationMatrix);

        rotationMatrix = rotationMatrix.t();
        translationMatrix = -rotationMatrix * translationMatrix;

        tf2::Matrix3x3 cameraRotation(rotationMatrix(0, 0), rotationMatrix(0, 1), rotationMatrix(0, 2),
                                      rotationMatrix(1, 0), rotationMatrix(1, 1), rotationMatrix(1, 2),
                                      rotationMatrix(2, 0), rotationMatrix(2, 1), rotationMatrix(2, 2));

        tf2::Vector3 cameraTranslation(translationMatrix(0, 0), translationMatrix(1, 0), translationMatrix(2, 0));

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

        foundCamera = true;
    }

    detectionPublisher.publish(cvDetectionImage->toImageMsg());
    carFrontMaskPublisher.publish(cvCarFrontMarkerImage->toImageMsg());
    carRearMaskPublisher.publish(cvCarRearMarkerImage->toImageMsg());
    markerMaskPublisher.publish(cvMapMarkerMaskImage->toImageMsg());
}

double VisualOdometry::getOrientation(const geometry_msgs::Pose& front, const geometry_msgs::Pose& rear) {
    return atan2(front.position.y - rear.position.y, front.position.x - rear.position.x);
}

geometry_msgs::Pose VisualOdometry::getMapCoordinates(const image_geometry::PinholeCameraModel& cameraModel, const cv::Point2d& point) const {
    auto rectifiedPoint = cameraModel.rectifyPoint(point);

    cv::Mat1d A(3,3);
    auto markerCameraHeight = translationMatrix(2, 0) - 0.15;
    A << markerCameraHeight, 0, -translationMatrix(0, 0), 0, markerCameraHeight, -translationMatrix(1, 0), 0, 0, -1;
    cv::Mat1d mat = A * rotationMatrix * cv::Mat(cameraModel.intrinsicMatrix().inv());
    cv::Mat1d p(3,1);
    p << rectifiedPoint.x, rectifiedPoint.y, 1.0;

    cv::Mat1d p2 = mat * p;
    p2 /= p2(2,0);

    geometry_msgs::Pose pose;
    pose.position.x = p2[0][0];
    pose.position.y = p2[1][0];
    pose.position.z = 0;

    return pose;
}

void VisualOdometry::findContours(const cv::Mat &image, cv::OutputArrayOfArrays contours) const {
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                         cv::Size(3,3));
    cv::erode(image, image, element);
    cv::dilate(image, image, element);
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
