#include <ros/ros.h>

#include "CameraLocalization.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_localization");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");

    camera_localization::CameraLocalization cameraLocalization(nh, nh_);

    ros::spin();
}
