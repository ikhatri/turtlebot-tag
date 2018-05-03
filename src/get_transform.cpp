#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point32.h>

#include "turtlebot_tag/GetTransformationSrv.h"

using std::vector;

bool GetTransformationService(
    turtlebot_tag::GetTransformationSrv::Request& req,
    turtlebot_tag::GetTransformationSrv::Response& res) {

  geometry_msgs::Point32 T;

  tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.lookupTransform("/base_laser", "/base_footprint",
                             ros::Time(0), transform);

    T.x = transform.getOrigin().x();
    T.y = transform.getOrigin().y();
    T.z = transform.getOrigin().z();

    tf::Matrix3x3 B = transform.getBasis();
    res.R[0] = B.getRow(0).getX(); res.R[1] = B.getRow(0).getY(); res.R[2] = B.getRow(0).getZ();
    res.R[3] = B.getRow(1).getX(); res.R[4] = B.getRow(1).getY(); res.R[5] = B.getRow(1).getZ();
    res.R[6] = B.getRow(2).getX(); res.R[7] = B.getRow(2).getY(); res.R[8] = B.getRow(2).getZ();

  } catch(tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ROS_WARN("Returning default transform");

    T.x = 0.13;
    T.y = 0.0;
    T.z = 0.305;

    res.R[0] = res.R[4] = res.R[8] = 1.0;
    res.R[1] = res.R[2] = res.R[3] = res.R[5] = res.R[6] = res.R[7] = 0.0;
  }

  res.T = T;

  return true;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "get_transform");
  ros::NodeHandle n;

  ros::ServiceServer service1 = n.advertiseService(
    "/COMPSCI403/GetTransformation", GetTransformationService);

  ros::spin();

  return(0);
}
