#include "VisualOdometry.h"


namespace fub_visual_odometry {

VisualOdometry::VisualOdometry(ros::NodeHandle& nh) : tfListener(tfBuffer) {
    image_transport::ImageTransport it(nh);
    imageSubscriber = it.subscribeCamera("/tracking/cam_left/image_raw", 10, &VisualOdometry::onImage, this, image_transport::TransportHints("compressed"));
    detectionPublisher = it.advertise("detection", 1);
    markerPublisher = nh.advertise<visualization_msgs::MarkerArray>("marker", 1);
    odomPublisher = nh.advertise<nav_msgs::Odometry>("odom", 1);

    mapDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    carDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    auto x = nh.param("front_marker_translation_x", 0.15);
    auto y = nh.param("front_marker_translation_y", 0);
    auto z = nh.param("front_marker_translation_z", 0.15);
    markerTranslation = tf2::Vector3(x, y, z);

    nodeHandle = nh;

    f = boost::bind(&VisualOdometry::onReconfigure, this, _1, _2);
    server.setCallback(f);
}

void VisualOdometry::onReconfigure(VisualOdometryConfig &config, uint32_t level) {
    this->config = config;

    // Find the camera again with the new parameters
    foundCamera = false;
}

void VisualOdometry::onMap(const nav_msgs::OccupancyGridConstPtr &msg) {
    map = msg;
}


void VisualOdometry::onImage(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
    cv_bridge::CvImagePtr cvDetectionImage;

    cv::waitKey(1);
    try
    {
        cvDetectionImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    image_geometry::PinholeCameraModel cameraModel;
    cameraModel.fromCameraInfo(info_msg);

    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();

    std::vector<int> carMarkerIds;
    std::vector<std::vector<cv::Point2f>> carMarkerCorners;
    cv::aruco::detectMarkers(cvDetectionImage->image, carDictionary, carMarkerCorners, carMarkerIds, params,
        cv::noArray(), cameraModel.intrinsicMatrix(), cameraModel.distortionCoeffs());

    if (!mapMarkerIds.empty()) {
        cv::aruco::drawDetectedMarkers(cvDetectionImage->image, mapMarkerCorners, mapMarkerIds);
        cv::aruco::drawDetectedMarkers(cvDetectionImage->image, carMarkerCorners, carMarkerIds);
    }

    if (!foundCamera)  {
        cv::aruco::detectMarkers(cvDetectionImage->image, mapDictionary, mapMarkerCorners, mapMarkerIds, params,
            cv::noArray(), cameraModel.intrinsicMatrix(), cameraModel.distortionCoeffs());

        cv::Mat1d rvec = cv::Mat1d::zeros(3, 1);
        translationMatrix = cv::Mat1d::zeros(3, 1);

        std::vector<cv::Point3f> worldCoordinates;
        std::vector<cv::Point2f> imageCoordinates;
        int i = 0;
        for (auto&& markerId : mapMarkerIds) {
            float x, y, z;
            auto has_x = nodeHandle.getParam(cv::format("marker_%d_x", markerId), x);
            auto has_y = nodeHandle.getParam(cv::format("marker_%d_y", markerId), y);
            auto has_z = nodeHandle.getParam(cv::format("marker_%d_z", markerId), z);

            if (has_x && has_y && has_z) {
                worldCoordinates.emplace_back(x, y, z);
                auto marker = mapMarkerCorners[i];
                auto rect= cv::boundingRect(marker);

                // This is probably not so accurate
                imageCoordinates.emplace_back(rect.x + rect.width / 2.0f, rect.y + rect.height / 2.0f);
            }

            i++;
        }

        if (mapMarkerIds.size() >= 4) {

            cv::solvePnP(worldCoordinates,
                         imageCoordinates,
                         cameraModel.intrinsicMatrix(),
                         cameraModel.distortionCoeffs(),
                         rvec,
                         translationMatrix,
                         false,
                         cv::SOLVEPNP_ITERATIVE);

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
            cameraTransform.header.stamp = msg->header.stamp;
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
            staticTransformBroadcaster.sendTransform(cameraTransform);

            foundCamera = true;
        }
    }

    if (foundCamera) {
        visualization_msgs::MarkerArray markers;

        if (!carMarkerCorners.empty()) {
            std::vector<cv::Vec3d> rvec, tvec;
            cv::aruco::estimatePoseSingleMarkers(carMarkerCorners, 0.05, cameraModel.intrinsicMatrix(), cameraModel.distortionCoeffs(), rvec, tvec);

            for (int i = 0; i < carMarkerCorners.size(); i++) {
                std::vector<cv::Point3f> pts = { cv::Point3f(0, 0, 0), cv::Point3f(1, 0, 0)};
                std::vector<cv::Point2f > imagePoints;
                cv::projectPoints(pts, rvec[i], tvec[i], cameraModel.intrinsicMatrix(), cameraModel.distortionCoeffs(), imagePoints);
                cv::line(cvDetectionImage->image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 3);

                /*cv::aruco::drawAxis(cvDetectionImage->image,
                                    cameraModel.intrinsicMatrix(),
                                    cameraModel.distortionCoeffs(),
                                    rvec[i],
                                    tvec[i],
                                    0.05)*/
                /*auto p1 = getMapCoordinates(cameraModel, carMarkerCorners[i][0], markerTranslation.z());
                auto p2 = getMapCoordinates(cameraModel, carMarkerCorners[i][1], markerTranslation.z());
                auto p3 = getMapCoordinates(cameraModel, carMarkerCorners[i][2], markerTranslation.z());
                auto p4 = getMapCoordinates(cameraModel, carMarkerCorners[i][3], markerTranslation.z());


                std::cout << yaw << std::endl;

                std_msgs::ColorRGBA c;
                c.a = 1.0;
                c.r = 0.0;
                c.g = 0.0;
                c.b = 1.0;

                addMarker(markers, p1, c, i, msg->header.stamp);
                addMarker(markers, p2, c, i + 1, msg->header.stamp);
                addMarker(markers, p3, c, i + 2, msg->header.stamp);
                addMarker(markers, p4, c, i + 3, msg->header.stamp);*/


                cv::Mat1d rod;
                cv::Rodrigues(rvec[i], rod);
                auto yaw = atan2(rod(0,0), rod(1,0));


                geometry_msgs::Quaternion orientation;
                orientation.x = 0;
                orientation.y = 0;
                orientation.z = sin(yaw / 2.0);
                orientation.w = cos(yaw / 2.0);

                auto rect= cv::boundingRect(carMarkerCorners[i]);

                // This is probably not so accurate
                geometry_msgs::Point frontPoint = getMapCoordinates(cameraModel, cv::Point2d(rect.x +rect.width /2.0, rect.y + rect.height / 2.0), 0.15);

                tf2::Vector3 cameraTranslation(frontPoint.x, frontPoint.y, frontPoint.z);
                geometry_msgs::TransformStamped carFrontMarkerTransform;
                carFrontMarkerTransform.header = msg->header;
                carFrontMarkerTransform.header.stamp = msg->header.stamp;
                carFrontMarkerTransform.header.frame_id = "map";
                carFrontMarkerTransform.child_frame_id = "car_front_marker";
                geometry_msgs::Vector3 m;
                tf2::convert(cameraTranslation, m);
                carFrontMarkerTransform.transform.translation = m;
                carFrontMarkerTransform.transform.rotation = orientation;
                transformBroadcaster.sendTransform(carFrontMarkerTransform);

                geometry_msgs::TransformStamped carBaseLinkTransform;
                carBaseLinkTransform.header = msg->header;
                carBaseLinkTransform.header.stamp = msg->header.stamp;
                carBaseLinkTransform.header.frame_id = "car_front_marker";
                carBaseLinkTransform.child_frame_id = "base_link";
                auto baseLinkTranslation = markerTranslation * -1;
                geometry_msgs::Vector3 baseLinkTranslationVec;
                geometry_msgs::Quaternion identity;
                tf2::convert(baseLinkTranslation, baseLinkTranslationVec);
                tf2::convert(tf2::Quaternion::getIdentity(), identity);
                carBaseLinkTransform.transform.translation = baseLinkTranslationVec;
                carBaseLinkTransform.transform.rotation = identity;
                transformBroadcaster.sendTransform(carBaseLinkTransform);

                nav_msgs::Odometry odometry;
                odometry.header.frame_id = "map";
                odometry.child_frame_id = "base_link";
                odometry.header.stamp = msg->header.stamp;
                odometry.pose.pose.orientation = orientation;
                odometry.pose.pose.position = frontPoint;
                odomPublisher.publish(odometry);

            }
        }

        markerPublisher.publish(markers);

    }

    cvDetectionImage->encoding = cvTypeToRosType(cvDetectionImage->image.type());
    detectionPublisher.publish(cvDetectionImage->toImageMsg());
}

double VisualOdometry::getOrientation(const geometry_msgs::Point& front, const geometry_msgs::Point& rear) {
    return atan2(front.y - rear.y, front.x - rear.x);
}

geometry_msgs::Point VisualOdometry::getMapCoordinates(const image_geometry::PinholeCameraModel& cameraModel, const cv::Point2d& point, double height) const {
    auto rectifiedPoint = cameraModel.rectifyPoint(point);

    cv::Mat1d A(3,3);
    auto markerCameraHeight = translationMatrix(2, 0) - height;
    A << markerCameraHeight, 0, -translationMatrix(0, 0), 0, markerCameraHeight, -translationMatrix(1, 0), 0, 0, -1;
    cv::Mat1d mat = A * rotationMatrix * cv::Mat(cameraModel.intrinsicMatrix().inv());
    cv::Mat1d p(3,1);
    p << rectifiedPoint.x, rectifiedPoint.y, 1.0;

    cv::Mat1d p2 = mat * p;
    p2 /= p2(2,0);

    geometry_msgs::Point mapPoint;
    mapPoint.x = p2[0][0];
    mapPoint.y = p2[1][0];
    mapPoint.z = height;

    return mapPoint;
}

void VisualOdometry::findContours(const cv::Mat &image, cv::OutputArrayOfArrays contours) const {
    cv::Mat kernel = cv::getStructuringElement( cv::MORPH_RECT, cv::Size(3,3));

    cv::erode(image, image, kernel);
    cv::dilate(image, image, kernel);
    cv::findContours(image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
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
        cv::RotatedRect e = cv::fitEllipse(it->second);
        cv::minEnclosingCircle(it->second, center, radius);
        markers.emplace_back(center, radius);

        if (i >= n) {
            break;
        }
    }
}

void VisualOdometry::addMarker(visualization_msgs::MarkerArray& markers, const geometry_msgs::Point& point, const std_msgs::ColorRGBA& color, int id, const ros::Time& stamp) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = stamp;
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = point;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color = color;
    markers.markers.push_back(marker);
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
