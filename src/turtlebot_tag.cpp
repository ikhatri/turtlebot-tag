#include <iostream>
#include <ros/ros.h>
#include <sstream>

#include <std_msgs/String.h>

// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include <math.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;
using ros::Subscriber;

Subscriber points;

int main(int argc, char **argv){
  ros::init(argc, argv, "turtlebot_tag");
  ros::NodeHandle n;

  //points = n.subscriber("camera/depth_register/points", callback_here);
  // the callback_here() function takes in a point cloud

  ros::spin();
  return 0;
}
