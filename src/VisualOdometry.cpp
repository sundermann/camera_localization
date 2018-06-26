#include "VisualOdometry.h"

namespace fub_visual_odometry {

VisualOdometry::VisualOdometry(ros::NodeHandle &globalNodeHandle, ros::NodeHandle &privateNodeHandle) : tfListener(tfBuffer) {
    image_transport::ImageTransport it(globalNodeHandle);
    imageSubscriber = it.subscribeCamera("image_raw",
                                         10,
                                         &VisualOdometry::onImage,
                                         this,
                                         image_transport::TransportHints("compressed"));
    detectionPublisher = it.advertise("detection", 1);
    markerPublisher = globalNodeHandle.advertise<visualization_msgs::MarkerArray>("marker", 1);

    mapDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    carDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    detectorParams = cv::aruco::DetectorParameters::create();

    for (int i = 0; i < carDictionary->bytesList.rows; i++) {
        odomPublishers[i] = globalNodeHandle.advertise<nav_msgs::Odometry>(cv::format("odom/%d", i), 1);
        lastOdometries[i] = nav_msgs::Odometry();
    }

    auto x = privateNodeHandle.param("front_marker_translation_x", 0.20);
    auto y = privateNodeHandle.param("front_marker_translation_y", 0);
    auto z = privateNodeHandle.param("front_marker_translation_z", 0.19);
    markerTranslation = tf2::Vector3(x, y, z);

    this->globalNodeHandle = globalNodeHandle;
    this->privateNodeHandle = privateNodeHandle;

    f = boost::bind(&VisualOdometry::onReconfigure, this, _1, _2);
    server.setCallback(f);
}

void VisualOdometry::onReconfigure(VisualOdometryConfig &config, uint32_t level) {
    this->config = config;

    detectorParams->adaptiveThreshConstant = config.adaptiveThreshConstant;
    detectorParams->adaptiveThreshWinSizeMin = config.adaptiveThreshWinSizeMin;
    detectorParams->adaptiveThreshWinSizeMax = config.adaptiveThreshWinSizeMax;
    detectorParams->adaptiveThreshWinSizeStep = config.adaptiveThreshWinSizeStep;
    detectorParams->cornerRefinementMaxIterations = config.cornerRefinementMaxIterations;
    detectorParams->cornerRefinementMinAccuracy = config.cornerRefinementMinAccuracy;
    detectorParams->cornerRefinementWinSize = config.cornerRefinementWinSize;
#if OPENCV_MINOR_VERSION==2
    detectorParams->doCornerRefinement = config.doCornerRefinement;
#else
    if (config.doCornerRefinement) {
        if (config.cornerRefinementSubpix) {
            detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        }
        else {
            detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
        }
    }
    else {
        detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    }
#endif
    detectorParams->errorCorrectionRate = config.errorCorrectionRate;
    detectorParams->minCornerDistanceRate = config.minCornerDistanceRate;
    detectorParams->markerBorderBits = config.markerBorderBits;
    detectorParams->maxErroneousBitsInBorderRate = config.maxErroneousBitsInBorderRate;
    detectorParams->minDistanceToBorder = config.minDistanceToBorder;
    detectorParams->minMarkerDistanceRate = config.minMarkerDistanceRate;
    detectorParams->minMarkerPerimeterRate = config.minMarkerPerimeterRate;
    detectorParams->maxMarkerPerimeterRate = config.maxMarkerPerimeterRate;
    detectorParams->minOtsuStdDev = config.minOtsuStdDev;
    detectorParams->perspectiveRemoveIgnoredMarginPerCell = config.perspectiveRemoveIgnoredMarginPerCell;
    detectorParams->perspectiveRemovePixelPerCell = config.perspectiveRemovePixelPerCell;
    detectorParams->polygonalApproxAccuracyRate = config.polygonalApproxAccuracyRate;

    // Find the camera again with the new parameters
    foundCamera = false;
}

void VisualOdometry::onMap(const nav_msgs::OccupancyGridConstPtr &msg) {
    map = msg;
}

void VisualOdometry::onImage(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info_msg) {
    cv_bridge::CvImagePtr cvDetectionImage;

    cv::waitKey(1);
    try {
        cvDetectionImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    image_geometry::PinholeCameraModel cameraModel;
    cameraModel.fromCameraInfo(info_msg);

    std::vector<int> carMarkerIds;
    std::vector<std::vector<cv::Point2f>> carMarkerCorners;
    cv::aruco::detectMarkers(cvDetectionImage->image, carDictionary, carMarkerCorners, carMarkerIds, detectorParams,
                             cv::noArray(), cameraModel.intrinsicMatrix(), cameraModel.distortionCoeffs());

    if (!mapMarkerIds.empty()) {
        cv::aruco::drawDetectedMarkers(cvDetectionImage->image, mapMarkerCorners, mapMarkerIds);
        cv::aruco::drawDetectedMarkers(cvDetectionImage->image, carMarkerCorners, carMarkerIds);
    }

    if (!foundCamera) {
        cv::aruco::detectMarkers(cvDetectionImage->image, mapDictionary, mapMarkerCorners, mapMarkerIds, detectorParams,
                                 cv::noArray(), cameraModel.intrinsicMatrix(), cameraModel.distortionCoeffs());

        cv::Mat1d rvec = cv::Mat1d::zeros(3, 1);
        translationMatrix = cv::Mat1d::zeros(3, 1);

        std::vector<cv::Point3f> worldCoordinates;
        std::vector<cv::Point2f> imageCoordinates;
        int i = 0;
        for (auto &&markerId : mapMarkerIds) {
            float x, y, z;
            auto has_x = privateNodeHandle.getParam(cv::format("marker_%d_x", markerId), x);
            auto has_y = privateNodeHandle.getParam(cv::format("marker_%d_y", markerId), y);
            auto has_z = privateNodeHandle.getParam(cv::format("marker_%d_z", markerId), z);

            if (has_x && has_y && has_z) {
                worldCoordinates.emplace_back(x, y, z);
                auto marker = mapMarkerCorners[i];
                auto rect = cv::boundingRect(marker);

                // This is probably not so accurate
                imageCoordinates.emplace_back(rect.x + rect.width / 2.0f, rect.y + rect.height / 2.0f);
            }

            i++;
        }

        if (worldCoordinates.size() >= 4) {

            cv::solvePnP(worldCoordinates,
                         imageCoordinates,
                         cameraModel.intrinsicMatrix(),
                         cameraModel.distortionCoeffs(),
                         rvec,
                         translationMatrix,
                         false,
                         cv::SOLVEPNP_ITERATIVE);

            double rms = checkCameraPose(worldCoordinates,
                                         imageCoordinates,
                                         cv::Mat(cameraModel.intrinsicMatrix()),
                                         cameraModel.distortionCoeffs(),
                                         rvec,
                                         translationMatrix);
            ROS_ERROR("Camera pose error: %f", rms);

            rotationMatrix = cv::Mat1d::zeros(3, 3);
            cv::Rodrigues(rvec, rotationMatrix);

            cv::Mat1d cameraTransformRotation = rotationMatrix.t();
            cv::Mat1d cameraTranslationMatrix = -cameraTransformRotation * translationMatrix;

            tf2::Matrix3x3 cameraRotation
                (cameraTransformRotation(0, 0), cameraTransformRotation(0, 1), cameraTransformRotation(0, 2),
                 cameraTransformRotation(1, 0), cameraTransformRotation(1, 1), cameraTransformRotation(1, 2),
                 cameraTransformRotation(2, 0), cameraTransformRotation(2, 1), cameraTransformRotation(2, 2));

            tf2::Vector3 cameraTranslation
                (cameraTranslationMatrix(0, 0), cameraTranslationMatrix(1, 0), cameraTranslationMatrix(2, 0));

            geometry_msgs::TransformStamped cameraTransform;
            cameraTransform.header = msg->header;
            cameraTransform.header.stamp = msg->header.stamp;
            cameraTransform.header.frame_id = "map";
            cameraTransform.child_frame_id = msg->header.frame_id;
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
        } else {
            ROS_ERROR("Not enough world coordinates!");
        }

    }

    if (foundCamera) {
        visualization_msgs::MarkerArray markers;

        if (!carMarkerCorners.empty()) {
            std::vector<cv::Vec3d> rvec, tvec;
            cv::aruco::estimatePoseSingleMarkers(carMarkerCorners,
                                                 0.095,
                                                 cameraModel.intrinsicMatrix(),
                                                 cameraModel.distortionCoeffs(),
                                                 rvec,
                                                 tvec);

            for (int i = 0; i < carMarkerCorners.size(); i++) {

                // calculate yaw
                std::vector<cv::Point3f> pts = {cv::Point3f(0, 0, 0), cv::Point3f(1, 0, 0)};
                std::vector<cv::Point2f> imagePoints;
                cv::projectPoints(pts,
                                  rvec[i],
                                  tvec[i],
                                  cameraModel.intrinsicMatrix(),
                                  cameraModel.distortionCoeffs(),
                                  imagePoints);
                cv::line(cvDetectionImage->image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 3);

                auto p1 = getMapCoordinates(cameraModel, imagePoints[0], markerTranslation.z());
                auto p2 = getMapCoordinates(cameraModel, imagePoints[1], markerTranslation.z());
                auto yaw = getOrientation(p2, p1);

                /*cv::Mat1d rod;
                cv::Rodrigues(rvec[i], rod);
                auto yaw = atan2(rod(0,0), rod(1,0)); */


                geometry_msgs::Quaternion orientation;
                orientation.x = 0;
                orientation.y = 0;
                orientation.z = sin(yaw / 2.0);
                orientation.w = cos(yaw / 2.0);

                auto rect = cv::boundingRect(carMarkerCorners[i]);

                // This is probably not so accurate
                geometry_msgs::Point frontPoint = getMapCoordinates(cameraModel,
                                                                    cv::Point2d(rect.x + rect.width / 2.0,
                                                                                rect.y + rect.height / 2.0),
                                                                    markerTranslation.z());

                tf2::Vector3 cameraTranslation(frontPoint.x, frontPoint.y, frontPoint.z);
                geometry_msgs::TransformStamped baseLinkTransform;
                baseLinkTransform.header = msg->header;
                baseLinkTransform.header.stamp = msg->header.stamp;
                baseLinkTransform.header.frame_id = "map";
                baseLinkTransform.child_frame_id = "base_link";
                geometry_msgs::Vector3 m;
                tf2::convert(cameraTranslation, m);
                baseLinkTransform.transform.translation = m;
                baseLinkTransform.transform.rotation = orientation;
                transformBroadcaster.sendTransform(baseLinkTransform);

                nav_msgs::Odometry odometry;
                odometry.header.frame_id = "map";
                odometry.child_frame_id = "base_link";
                odometry.header.stamp = msg->header.stamp;
                odometry.pose.pose.orientation = orientation;
                odometry.pose.pose.position = frontPoint;
                odometry.twist.twist = getTwist(odometry, lastOdometries[carMarkerIds[i]], p2);
                odomPublishers[carMarkerIds[i]].publish(odometry);

                lastOdometries[carMarkerIds[i]] = odometry;
            }
        }

        markerPublisher.publish(markers);

    }

    cvDetectionImage->encoding = cvTypeToRosType(cvDetectionImage->image.type());
    detectionPublisher.publish(cvDetectionImage->toImageMsg());
}

geometry_msgs::Twist VisualOdometry::getTwist(const nav_msgs::Odometry &last,
                                              const nav_msgs::Odometry &current,
                                              const geometry_msgs::Point &orientationPoint) {
    geometry_msgs::Twist twist;
    auto deltaTime = current.header.stamp - last.header.stamp;

    // this probably means that the vehicle moved out of the camera and now moves back in
    if (deltaTime.toSec() > 1) {
        return twist;
    }

    tf2::Vector3 lastPos, currentPos, orientationPos;
    tf2::convert(last.pose.pose.position, lastPos);
    tf2::convert(current.pose.pose.position, currentPos);
    tf2::convert(orientationPoint, orientationPos);

    auto currentDirection = orientationPos - currentPos;
    auto lastDirection = currentPos - lastPos;
    // dot product is positive if both look into the same direction
    auto direction = currentDirection.dot(lastDirection);

    twist.linear.x = std::abs(lastPos.distance(currentPos)) / deltaTime.toSec();

    if (direction < 0) {
        twist.linear.x *= -1.0;
    }

    return twist;
}

double VisualOdometry::getOrientation(const geometry_msgs::Point &front, const geometry_msgs::Point &rear) {
    return atan2(front.y - rear.y, front.x - rear.x);
}

geometry_msgs::Point VisualOdometry::getMapCoordinates(const image_geometry::PinholeCameraModel &cameraModel,
                                                       const cv::Point2d &point,
                                                       double height) const {
    auto rectifiedPoint = cameraModel.rectifyPoint(point);

    cv::Mat1d p(3, 1);
    p << rectifiedPoint.x, rectifiedPoint.y, 1.0;

    cv::Mat1d tempMat, tempMat2;
    double s = 0;
    tempMat = rotationMatrix.inv() * cv::Mat1d(cameraModel.intrinsicMatrix().inv()) * p;
    tempMat2 = rotationMatrix.inv() * translationMatrix;
    s = height + tempMat2(2, 0);
    s /= tempMat(2, 0);

    cv::Mat1d wcTranslation(3, 1);
    wcTranslation << markerTranslation.x(), markerTranslation.y(), markerTranslation.z();
    cv::Mat1d wcPoint = rotationMatrix.inv()
        * (s * cv::Mat(cameraModel.intrinsicMatrix().inv()) * p - translationMatrix + wcTranslation);
    //wcPoint = (rotationMatrix.inv()) * wcPoint - wcTranslation;

    geometry_msgs::Point mapPoint;
    mapPoint.x = wcPoint(0, 0);
    mapPoint.y = wcPoint(1, 0);
    mapPoint.z = wcPoint(2, 0);

    return mapPoint;
}

double VisualOdometry::checkCameraPose(const std::vector<cv::Point3f> &worldPoints,
                                       const std::vector<cv::Point2f> &imagePoints,
                                       const cv::Mat &cameraMatrix,
                                       const cv::Mat &distCoeffs,
                                       const cv::Mat &rvec,
                                       const cv::Mat &tvec) {
    std::vector<cv::Point2f> projectedPts;
    cv::projectPoints(worldPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPts);

    double rms = 0.0;
    for (size_t i = 0; i < projectedPts.size(); i++) {
        rms += (projectedPts[i].x - imagePoints[i].x) * (projectedPts[i].x - imagePoints[i].x)
            + (projectedPts[i].y - imagePoints[i].y) * (projectedPts[i].y - imagePoints[i].y);
    }

    return sqrt(rms / projectedPts.size());
}

void VisualOdometry::addMarker(visualization_msgs::MarkerArray &markers,
                               const geometry_msgs::Point &point,
                               const std_msgs::ColorRGBA &color,
                               int id,
                               const ros::Time &stamp) {
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

    switch (depth) {
        case CV_8U: r = "8U";
            break;
        case CV_8S: r = "8S";
            break;
        case CV_16U: r = "16U";
            break;
        case CV_16S: r = "16S";
            break;
        case CV_32S: r = "32S";
            break;
        case CV_32F: r = "32F";
            break;
        case CV_64F: r = "64F";
            break;
        default: r = "User";
            break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}

}
