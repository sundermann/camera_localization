# Camera localization
Camera localization using [aruco](https://docs.opencv.org/master/d9/d6d/tutorial_table_of_content_aruco.html) markers.

# Setup
We installed three cameras on the ceiling facing downwards to the map surface. These cameras are connected to one computer which is responsible for providing the localization for all the cars. Each car has its own unique aruco marker (4x4_50, 9 cm) and aruco markers (6x6_250) are placed onto the map surface. Each camera has to see at least four aruco markers on the floor of which their world coordinates are known. The cameras should be installed so that they overlap each other a bit so that the marker on the car can be seen at all times by at least one camera.

## Creating the aruco markers
Inside the scripts folders there are two python scripts for creating the car and map surface markers. Run each script using python and provide an unique id:

```
python create_car_marker.py 11
python create_map_marker.py 11
```

This will create two image files for the car marker 11 and map marker 11. The map surface markers should be printed on a single A4 paper with a edge length of about 24 cm whilst four car markers should be printed onto a single A4 paper with an edge length of about 9 cm. When cutting the markers you should keep as much whitespace around the markers because that is needed for the detection to work.

## Running
The RoboticsLabLocalization.launch launch file will run everything needed for the localization. The multimaster communication needs `ROS_HOSTNAME` and `ROS_MASTER_URI` set to the ip of the computer.

```
export ROS_HOSTNAME=192.168.43.21
export ROS_MASTER_URI=http://192.168.43.21:11311/
roslaunch camera_localization RoboticsLabLocalization.launch
```

## Checking the localization
Run rviz and check if the tf frames of each camera has been positioned correctly. Place a camera onto the map surface and check if the tf frame `car_ID/base_link` and odometry are published correctly and are localized correctly. The camera localization is quite precise and is usually only 1-2 cm off if the cameras are calibrated correctly.

# Multimaster
The localization node is designed to run on a separate computer with its own roscore. The ros package [multimaster_fkie](http://wiki.ros.org/multimaster_fkie) is used to send the localization information to the individual roscores on the cars.

## Connecting to the localization computer on the car
Start the master_discovery and master_sync node from the ros-multimaster-fkie package:

```
rosrun master_discovery_fkie master_discovery 
rosrun master_sync_fkie master_sync
```

## Launchfile to connect only to the localization host
```
<launch>
    <node name="master_discovery" pkg="master_discovery_fkie" type="master_discovery" output="screen"/>
    <node name="master_sync" pkg="master_sync_fkie" type="master_sync" output="screen">
        <rosparam param="sync_hosts">['192.168.43.20']</rosparam>
    </node>
</launch>
```

# Configuration
Configuration is done through dynamic reconfigure and launch files. The dynamic reconfigure config contains the parameters for the aruco marker detection which can be adjusted while running the node. Any parameter change will trigger a redection of the markers on the map surface.

The static configuration is done through the launch files.

## Camera.launch
Runs the [usb_cam](http://wiki.ros.org/usb_cam) camera driver package for three cameras at /dev/video-left, /dev/video-middle and /dev/video-right. The exposure time is set to manual mode with a fairly low value. It is important to set the exposure time as low as possible to avoid motion blur as much as possible. If the camera cannot detect the markers you might have to find better values for exposure time and gain. You can use v4l2ucp to find better parameters for the camera. Each camera runs in its own namespace. These values are valid for Logitech C930e USB cameras and might not work with other cameras or other lighting conditions. You can also use other camera drivers as long as it publishes the camera image (compressed or raw) and the camera calibration information.

## CameraLocalization.launch
Runs a camera_localization node for each camera. Each camera_localization node runs in its own namespace along with the camera driver.

## SingleCameraLocalization.launch
Runs a single camera_localization node. This launch file contains the positions of the markers and the translation of the markers to the rear axle. The world position of each marker center on the map surface should be configured here:

```
        <param name="marker_ID_x" value="2.56" />
        <param name="marker_ID_y" value="2.745" />
        <param name="marker_ID_z" value="0.0" />
```
You should replace `ID` with the marker's id.

Since the marker might not be placed at the position of the odometry a translation can be applied to the marker position to the the odometry. In this example we shift the position back in x direction by 20cm and the height by 19 cm which is then the middle of the rear axle at the ground for our cars:
```
        <param name="front_marker_translation_x" value="0.20" />
        <param name="front_marker_translation_y" value="0" />
        <param name="front_marker_translation_z" value="0.19" />
```

## RoboticsLabLocalization.launch
Runs all the nodes needed for localization at our lab. It also includes the map_publisher package which publishes an occupancy grid of the map we are using.

## Multimaster.launch
Runs the multimaster_fkie master_discovery node so the cars and detect the computer. Make sure `ROS_HOSTNAME` and `ROS_MASTER_URI` are set to the ip of the computer.
