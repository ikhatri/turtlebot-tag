#include <iostream>
#include <ros/ros.h>
#include <sstream>

#include <std_msgs/String.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point32.h"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Dense>
#include <math.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;
using ros::Subscriber;

Subscriber points;
Subscriber images;

image_transport::Subscriber image_sub_;
Eigen::Vector3f goal_;

static const std::string OPENCV_WINDOW = "Image window";

void KinectCallback(const sensor_msgs::PointCloud2& p){
  return;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat orange_image;
  cv::Scalar lower(0, 0, 55);
  cv::Scalar upper(20, 20, 255);
  cv::inRange(cv_ptr->image, lower, upper, orange_image);

  // Setup SimpleBlobDetector parameters.
  cv::SimpleBlobDetector::Params params;
  
  // Change thresholds
  params.minThreshold = 10;
  params.maxThreshold = 200;
  
  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 10;
  
  // Filter by Circularity
  params.filterByCircularity = true;
  params.minCircularity = 0.3;
  params.maxCircularity = 0.8;
  
  // Filter by Convexity
  params.filterByConvexity = true;
  params.minConvexity = 0.87;
  
  // Filter by Inertia
  params.filterByInertia = true;
  params.minInertiaRatio = 0.01;
  

  cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
  std::vector<cv::KeyPoint> keypoints;
  detector->detect(orange_image, keypoints);
  float x_total = 0;
  float y_total = 0;
  for(auto& k : keypoints){
    x_total += k.pt.x;
    y_total += k.pt.y;
  }
  std::vector<cv::KeyPoint> the_keypoint;
  the_keypoint.push_back(cv::KeyPoint(x_total/keypoints.size(), y_total/keypoints.size(), 100));
  cv::Mat im_with_keypoints;
  cv::drawKeypoints( cv_ptr->image, the_keypoint, im_with_keypoints, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  
  // Set the new goal point
  goal_[0] = x_total/keypoints.size();
  goal_[1] = y_total/keypoints.size();

  // // Draw an example circle on the video stream
  // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
  //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, im_with_keypoints);
  cv::waitKey(3);
}

void callback(const sensor_msgs::ImageConstPtr& i1, const sensor_msgs::ImageConstPtr& i2){
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(i1, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat orange_image;
  cv::Scalar lower(0, 0, 55);
  cv::Scalar upper(20, 20, 255);
  cv::inRange(cv_ptr->image, lower, upper, orange_image);

  // Setup SimpleBlobDetector parameters.
  cv::SimpleBlobDetector::Params params;
  
  // Change thresholds
  params.minThreshold = 10;
  params.maxThreshold = 200;
  
  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 10;
  
  // Filter by Circularity
  params.filterByCircularity = true;
  params.minCircularity = 0.3;
  params.maxCircularity = 0.8;
  
  // Filter by Convexity
  params.filterByConvexity = true;
  params.minConvexity = 0.87;
  
  // Filter by Inertia
  params.filterByInertia = true;
  params.minInertiaRatio = 0.01;
  

  cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
  std::vector<cv::KeyPoint> keypoints;
  detector->detect(orange_image, keypoints);
  float x_total = 0;
  float y_total = 0;
  for(auto& k : keypoints){
    x_total += k.pt.x;
    y_total += k.pt.y;
  }
  std::vector<cv::KeyPoint> the_keypoint;
  the_keypoint.push_back(cv::KeyPoint(x_total/keypoints.size(), y_total/keypoints.size(), 100));
  cv::Mat im_with_keypoints;
  cv::drawKeypoints( cv_ptr->image, the_keypoint, im_with_keypoints, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  

  // Encoding the depth image to be read in easily
  cv_bridge::CvImagePtr cv_ptr2;
  try {
    cv_ptr2 = cv_bridge::toCvCopy(i2, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  float depth = cv_ptr2->image.at<short int>(cv::Point(x_total/keypoints.size(), y_total/keypoints.size()));

  // Set the new goal point
  // Need to check if this is actually correct
  goal_[0] = x_total/keypoints.size();
  goal_[1] = -1*y_total/keypoints.size();
  goal_[2] = depth;
  
  cv::imshow(OPENCV_WINDOW, im_with_keypoints);
  cv::waitKey(3);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "turtlebot_tag");
  ros::NodeHandle n;
  image_transport::ImageTransport it_(n);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_policy_classification;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_(n, "/camera/rgb/image_color", 10);
  message_filters::Subscriber<sensor_msgs::Image> points(n, "/camera/depth/image", 10);
  message_filters::Synchronizer<sync_policy_classification> sync(sync_policy_classification(10), image_sub_, points);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  // image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, imageCb);
  // points = n.subscribe("/camera/depth_register/points", 1000, KinectCallback);

  cv::namedWindow(OPENCV_WINDOW);
  
  ros::spin();
  return 0;
}