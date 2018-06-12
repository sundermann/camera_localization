#include <ros/ros.h>

#include "VisualOdometry.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fub_visual_odometry");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");

    fub_visual_odometry::VisualOdometry visualOdometry(nh_);

    ros::spin();
}
