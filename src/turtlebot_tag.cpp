#include <iostream>
#include <ros/ros.h>
#include <sstream>

#include <std_msgs/String.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point32.h"
#include <math.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;
using ros::Subscriber;

Subscriber points;
Subscriber images;

void KinectCallback(const sensor_msgs::PointCloud2& p){
  return;
}

void OpenCVCallback(const sensor_msgs::Image& i){
  return;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "turtlebot_tag");
  ros::NodeHandle n;

  images = n.subscribe("camera/rgb/image_color", 1000, OpenCVCallback);
  points = n.subscribe("camera/depth_register/points", 1000, KinectCallback);
  // the callback_here() function takes in a point cloud

  ros::spin();
  return 0;
}