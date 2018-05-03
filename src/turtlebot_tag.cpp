#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include "turtlebot_tag/CheckPointSrv.h"
#include "turtlebot_tag/GetFreePathSrv.h"
#include "turtlebot_tag/ObstacleLaserScanSrv.h"
#include "turtlebot_tag/GetCommandVelSrv.h"
#include "turtlebot_tag/GetTransformationSrv.h"

using Eigen::Matrix3f;
using Eigen::MatrixXf;
using Eigen::MatrixXd;
using Eigen::Vector3f;
using Eigen::Vector2f;
using geometry_msgs::Point32;
using geometry_msgs::Point;
using geometry_msgs::Twist;
using nav_msgs::Odometry;
using visualization_msgs::Marker;
using sensor_msgs::LaserScan;
using sensor_msgs::PointCloud;
using std::cout;
using std::vector;
using namespace std;

using turtlebot_tag::CheckPointSrv;
using turtlebot_tag::GetFreePathSrv;
using turtlebot_tag::ObstacleLaserScanSrv;
using turtlebot_tag::GetCommandVelSrv;
using turtlebot_tag::GetTransformationSrv;

// Services and Subscribers
ros::ServiceServer checkPointServer;
ros::ServiceServer getFreePathServer;
ros::ServiceServer transformPointsServer;
ros::ServiceServer getCommandVelocityServer;

ros::Subscriber laserScanSubscriber;
ros::Subscriber odometrySubscriber;
ros::ServiceClient getTransformClient;

ros::Publisher driveMsgPublisher;
ros::Publisher rvizPublisher;

// Robot Params
float robotRadius = 0.18; // 0.18 meters

float maxLinearVelocity = 0.5; // [meters / second]
float minLinearVelocity = 0.0;
float maxLinearAcceleration = 0.5; // [meters / (second^2)]

float maxAngularVelocity = 1.5; // [radians / second]
float minAngularVelocity = -maxAngularVelocity;
float maxAngularAcceleration = 2.0; // [radians / (second^2)]

float velocityCommandUpdateRate = 0.05; // (new command every 0.05 seconds = 20 hz)

float k_pi = 3.141592653589793;

// Robot State Variables
float currLinearVelocity = 0.0;
float currAngularVelocity = 0.0;
float currHeading = 0.0;
float desiredHeading = 0.0;

vector<Point32> currPoints;
float currMaxDistance = 4.0;

// Weights for objective function
float angularWeight = 0.0;
float freePathWeight = 0.0;
float speedWeight = 0.0;
float clearanceWeight = 0.0;
float stopDistanceOffset = 0.0;

Point32 target;
bool finished = false; 

/*
// Global Parameters
const float gRobotRadius = 0.18;
const float gRobotHeight = 0.36;
const float gEpsilon = 0.15;
const float gAMaxLin = 1 * 0.5; // m/s^2
const float gAMaxRot = 1 * 2.0; // rad/s^2
const float gVMaxLin = 0.5; // m/s
const float gVMaxRot = 1.5; // rad/s
const float gDT = 0.02; // s
*/

// general helper
float constrainHeading(float heading) {
  // corrections for if you've turned past pi radians (180 degrees)
  // and you are now turning back towards the target instead of away.
  if (heading > k_pi) {
    return heading - (2 * k_pi);
  }
  else if (heading < -k_pi) {
    return heading + (2 * k_pi);
  }
  return heading;
}

// helper for visualization
Marker getMarker(float r, float g, float b) {
  Marker marker;

  marker.header.frame_id = "/base_footprint";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = Marker::SPHERE_LIST;
  marker.action = Marker::ADD;

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.a = 1.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;

  return marker;
}

// helper for checkPoint
Point32 constructPoint(float x, float y, float z) {
  Point32 returnVal;
  returnVal.x = x;
  returnVal.y = y;
  returnVal.z = z;
  return returnVal;
}

// helper for checkPoint
float distanceFormula(const Point32& a, const Point32& b) {
  float deltaX = b.x - a.x;
  float deltaY = b.y - a.y;
  return sqrt((deltaX * deltaX) + (deltaY * deltaY));
}

// helper for checkPoint
float lawOfCosines(float sideA, float sideB, float sideC) {
  float numerator = (sideA * sideA) + (sideB * sideB) - (sideC * sideC);
  float denominator = 2 * sideA * sideB;
  float theta = acos(numerator / denominator);
  return theta;
}

// helper for checkPoint
bool isAlongStraightPath(Point32& p) {
  return (-robotRadius <= p.y && p.y <= robotRadius);
}

// helper for checkPoint
bool isAlongCurvedPath(float robotCurvature, float pointCurvature) {
  float minRadiusOfCurvature = robotCurvature - robotRadius;
  float maxRadiusOfCurvature = robotCurvature + robotRadius;
  return (minRadiusOfCurvature <= pointCurvature) && (pointCurvature <= maxRadiusOfCurvature);
}

// Callback for "/COMPSCI403/CheckPoint"
bool checkPoint(CheckPointSrv::Request& input, CheckPointSrv::Response& output) {
  /* in the event that the given point isn't on the path of the robot,
     then the free_path_length that's returned is the point's distance from the
     robot's path
  */

  if (input.w == 0) {
    if (isAlongStraightPath(input.P)) {
      output.is_obstacle = true;
      output.free_path_length = input.P.x - sqrt((robotRadius * robotRadius) - (input.P.y * input.P.y));
    } else {
      output.is_obstacle = false;
      output.free_path_length = fabs(input.P.y) - robotRadius;
    }
  }
  else if (input.w != 0) {
    // calculate radii of curvature (note: omega * r = v)
    Point32 centerOfCurvature = constructPoint(0, (input.v / input.w), 0);
    float radiusOfCurvature_robot = fabs(centerOfCurvature.y);
    float radiusOfCurvature_obstacle = distanceFormula(centerOfCurvature, input.P);

    // if there's potential for collision, calculate free path length
    if (isAlongCurvedPath(radiusOfCurvature_robot, radiusOfCurvature_obstacle)) {
      output.is_obstacle = true;

      // start by calculating the angle to the point along the current path
      float euclideanDistance = distanceFormula(constructPoint(0, 0, 0), input.P);
      float angleToPoint = lawOfCosines(radiusOfCurvature_robot, radiusOfCurvature_obstacle, euclideanDistance);
      if (input.P.x < 0) {
        angleToPoint = (2 * k_pi) - angleToPoint;
      }

      // then calculate angle to point when the point is in contact with the robot
      // (i.e. the minimum allowable angle to the point without colliding with it)
      float angleOfIntersection = lawOfCosines(radiusOfCurvature_robot, radiusOfCurvature_obstacle, robotRadius);

      // the difference in these two angles is the angle through which the robot
      // can travel without colliding with the point.
      output.free_path_length = (angleToPoint - angleOfIntersection) * radiusOfCurvature_robot;
    }
    else {
      // obstacle isn't on circular path
      output.is_obstacle = false;
      output.free_path_length = fabs(radiusOfCurvature_obstacle - radiusOfCurvature_robot) - robotRadius;
    }
  }
  return true;
}

// general function to get points from laser scan
vector<Point32> laserScanToPoints(const LaserScan& scan, Point32& translation) {
  vector<Point32> returnVal;

  // angles in the range [angle_min, angle_max)
  int numOfDataPoints = (scan.angle_max - scan.angle_min) / scan.angle_increment;

  // convert all points that are within the range of the
  // laser scanner to (x, y, z) coordinates
  for (int i = 0; i < numOfDataPoints; i++) {
    float range = scan.ranges[i];

    if (scan.range_min <= range && range <= scan.range_max) {
      // simultaneously calculate coordinates of currPoint in frame of scanner,
      // and apply the given transformation
      float currAngle = scan.angle_min + i * (scan.angle_increment);
      Point32 currPoint;
      currPoint.x = (range * cos(currAngle)) + translation.x;
      currPoint.y = (range * sin(currAngle)) + translation.y;
      currPoint.z = 0 + translation.z;
      // z (before translation) = height of points relative to laser scanner.
      // assuming a horizontal scanner, z (before translation) is always 0.
      returnVal.push_back(currPoint);
    }
  }
  return returnVal;
}

// general function to get info about a set of points for a given v and w
vector<CheckPointSrv> checkAllPoints(const vector<Point32>& points, float v, float w) {
  vector<CheckPointSrv> output;
  for (unsigned int i = 0; i < points.size(); i++) {
    CheckPointSrv srv;
    srv.request.P = points[i];
    srv.request.v = v;
    srv.request.w = w;
    checkPoint(srv.request, srv.response);
    output.push_back(srv);
  }
  return output;
}

// general function to get the point info with the min distance for a given v and w along a selected path
GetFreePathSrv::Response getShortestDistance(const vector<CheckPointSrv>& pointInfo, float maxDistance, bool alongPath) {
  // init output
  GetFreePathSrv::Response output;
  output.is_obstacle = false;
  output.free_path_length = maxDistance;

  // find min distance
  for (unsigned int i = 0; i < pointInfo.size(); i++) {
    CheckPointSrv::Response currPoint = pointInfo[i].response;

    // if the point is an obstacle, it lies along the trajectory of the robot
    if ((currPoint.is_obstacle == alongPath) && (currPoint.free_path_length < output.free_path_length)) {
      output.is_obstacle = currPoint.is_obstacle;
      output.free_path_length = currPoint.free_path_length;
    }
  }
  return output;
}

// Callback for "/COMPSCI403/GetFreePath"
bool getFreePath(GetFreePathSrv::Request& input, GetFreePathSrv::Response& output) {
  Point32 noTransform = constructPoint(0, 0, 0);
  vector<Point32> points = laserScanToPoints(input.laser_scan, noTransform);
  vector<CheckPointSrv> pointInfo = checkAllPoints(points, input.v, input.w);
  output = getShortestDistance(pointInfo, input.laser_scan.range_max, true);
  return true;
}

bool transformPoints(ObstacleLaserScanSrv::Request& input, ObstacleLaserScanSrv::Response& output) {
  output.S_prime = laserScanToPoints(input.S, input.T);
  return true;
}

bool hasEnoughRoomToStop(float v, float w, const vector<Point32>& points) {
  vector<CheckPointSrv> pointInfo = checkAllPoints(points, v, w);
  GetFreePathSrv::Response freePathInfo = getShortestDistance(pointInfo, currMaxDistance, true);
  if (!freePathInfo.is_obstacle) {
    return true;
  }

  // this check assumes a path of constant radius.
  // in order for that to be true, the ratio (v/w) must be the same throughout the maneuver
  // in order to keep it the same, v and w must change at the same rate.
  // therefore, we use the biggest acceleration that can be achieved
  // both linearly and angularly to calculate the stopping distance,
  // as this is the biggest acceleration that will keep a constant radius.
  float maxLinearAccelerationOfConstantRadius = std::min(maxLinearAcceleration, maxAngularAcceleration);
  float distanceNeededToStop = (-1 * (v * v)) / (2 * -maxLinearAccelerationOfConstantRadius);
  distanceNeededToStop += stopDistanceOffset;

  /*
  bool returnVal = (!srv.response.is_obstacle) || (distanceNeededToStop <= srv.response.free_path_length);
  if (!returnVal) {
    ROS_INFO("REJECTED V: %f, W: %f", v, w);
    ROS_INFO("OBSTACLE DETECTED: %d", srv.response.is_obstacle);
    ROS_INFO("STOPPING DISTANCE: %f", distanceNeededToStop);
    ROS_INFO("FPL: %f\n", srv.response.free_path_length);
  }*/
  return (distanceNeededToStop <= freePathInfo.free_path_length);
}

// objective function
float calculateScore(float v, float w, const vector<Point32>& points, bool print) {
  // get free path for the given v & w. (note that currScan is updated in publishToObstaclesTopic())
  // distance score (high score = longest free path)
  vector<CheckPointSrv> pointInfo = checkAllPoints(points, v, w);
  float freePathLength = getShortestDistance(pointInfo, currMaxDistance, true).free_path_length;
  float distanceToObstaclesScore = freePathLength * freePathWeight;

  // get the shortest distance of an obstacle not on the current path to the robot
  // clearance score (high score = most clearance)
    // get the shortest distance of an obstacle not on the current path to the robot
  // clearance score (high score = most clearance)
  float sizeBefore = pointInfo.size();
  if (w != 0) {
    // eliminate points that are beyond the FPL
    Point32 centerOfCurvature = constructPoint(0, (v / w), 0);
    float robotTurningRadius = fabs(centerOfCurvature.y);
    float freeAngle = freePathLength / robotTurningRadius;

    for (int i = pointInfo.size()-1; i >= 0; i--) {
      // calculate angle to point
      float pointTurningRadius = distanceFormula(centerOfCurvature, pointInfo[i].request.P);
      float euclideanDistance = distanceFormula(constructPoint(0, 0, 0), pointInfo[i].request.P);
      float angleToPoint = lawOfCosines(robotTurningRadius, pointTurningRadius, euclideanDistance);
      if (pointInfo[i].request.P.x < 0) {
        angleToPoint = (2 * k_pi) - angleToPoint;
      }

      if (angleToPoint > freeAngle) {
        pointInfo.erase(pointInfo.begin() + i);
      }
    }
  }
  float sizeAfter = pointInfo.size();

  if (print) {
    ROS_INFO("SIZE DIFFERENCE: %f", (sizeBefore - sizeAfter));
  }

  float clearance = getShortestDistance(pointInfo, currMaxDistance, false).free_path_length;
  if(clearance < .2) {
  	return -1;
  }
  float clearanceScore = clearance * clearanceWeight;

  // angle score (highest score is heading of 0, score gets worse as heading deviates from 0)
  float headingToTarget = atan2(target.y, target.x);
  float proposedHeading = constrainHeading(headingToTarget - w * velocityCommandUpdateRate);
  //float angleToTarget = (k_pi - fabs(proposedHeading));
  float angularScore = (k_pi - fabs(proposedHeading)) * angularWeight;

  // speed cost (high score = high speed)
  float speedScore = v * speedWeight;

  float totalScore = angularScore + distanceToObstaclesScore + speedScore + clearanceScore;

  if (print) {
    ROS_INFO("V: %f, W: %f", v, w);
    ROS_INFO("ANGULAR SCORE: %f", angularScore);
    ROS_INFO("DISTANCE SCORE: %f", distanceToObstaclesScore);
    ROS_INFO("FPL: %f", freePathLength);
    ROS_INFO("SPEED SCORE: %f", speedScore);
    ROS_INFO("CLEARANCE SCORE: %f", clearanceScore);
    ROS_INFO("TOTAL SCORE: %f\n", totalScore);

    Marker obstacles = getMarker(1, 0, 0);
    Marker freePoints = getMarker(0, 1, 0);
    for (unsigned int i = 0; i < pointInfo.size(); i++) {
      geometry_msgs::Point toPush;
      toPush.x = pointInfo[i].request.P.x;
      toPush.y = pointInfo[i].request.P.y;
      toPush.z = pointInfo[i].request.P.z;

      if (pointInfo[i].response.is_obstacle) {
        obstacles.points.push_back(toPush);
      } else {
        freePoints.points.push_back(toPush);
      }
    }
    rvizPublisher.publish(obstacles);
    //rvizPublisher.publish(freePoints);
  }
  return totalScore;
}

bool getCommandVelocity(GetCommandVelSrv::Request& input, GetCommandVelSrv::Response& output) {
  // calculate bounds of dynamic window
  float maxDeltaV = maxLinearAcceleration * velocityCommandUpdateRate;
  float maxDeltaW = maxAngularAcceleration * velocityCommandUpdateRate;
  float linearVelocityUpperBound = std::min(maxLinearVelocity, input.v_0 + maxDeltaV);
  float linearVelocityLowerBound = std::max(minLinearVelocity, input.v_0 - maxDeltaV);
  float angularVelocityUpperBound = std::min(maxAngularVelocity, input.w_0 + maxDeltaW);
  float angularVelocityLowerBound = std::max(minAngularVelocity, input.w_0 - maxDeltaW);

  // init output command to just be achieveable velocities closest to 0
  output.C_v = linearVelocityLowerBound;
  if (angularVelocityLowerBound <= 0 && 0 <= angularVelocityUpperBound) {
    output.C_w = 0;
  } else {
    output.C_w = (fabs(angularVelocityLowerBound) < fabs(angularVelocityUpperBound)) ? angularVelocityLowerBound : angularVelocityUpperBound;
  }

  // calculate initial score
  ROS_INFO("INITIAL SCORE CALCULATION");
  float maxScore = calculateScore(output.C_v, output.C_w, currPoints, true);

  // set dynamic window stepsize (resolution)
  float numOfSteps = 10.0; // actual number of discrete velocites checked should theoretically this value squared.
  // however, this wasn't alwasy observed, likely because ranges aren't just between integers.
  float linearVelocityStepSize = (linearVelocityUpperBound - linearVelocityLowerBound) / numOfSteps;
  float angularVelocityStepSize = (angularVelocityUpperBound - angularVelocityLowerBound) / numOfSteps;

  // iterate through all discretized combinations of V and W
  for (float currV = linearVelocityLowerBound; currV <= linearVelocityUpperBound; currV += linearVelocityStepSize) {
    for (float currW = angularVelocityLowerBound; currW <= angularVelocityUpperBound; currW += angularVelocityStepSize) {
      if (hasEnoughRoomToStop(currV, currW, currPoints)) {
        float score = calculateScore(currV, currW, currPoints, false);

        if (score > maxScore) {
          maxScore = score;
          output.C_v = currV;
          output.C_w = currW;
        }
      }
    }
  }
  ROS_INFO("MAX SCORE FOUND");
  calculateScore(output.C_v, output.C_w, currPoints, true); // this is here just to print individual scores
  return true;
}

void updateWorldView(const LaserScan& scan) {
  GetTransformationSrv srv1;
  getTransformClient.call(srv1);

  ObstacleLaserScanSrv srv2;
  srv2.request.S = scan;
  srv2.request.R = srv1.response.R;
  srv2.request.T = srv1.response.T;
  transformPoints(srv2.request, srv2.response);

  currPoints = srv2.response.S_prime;
  currMaxDistance = scan.range_max;
}

void updateVelocity() {
  GetCommandVelSrv srv;
  srv.request.v_0 = currLinearVelocity;
  srv.request.w_0 = currAngularVelocity;

  getCommandVelocity(srv.request, srv.response);

  geometry_msgs::Twist output;
  output.linear.x = srv.response.C_v;
  output.linear.y = 0;
  output.linear.z = 0;
  output.angular.x = 0;
  output.angular.y = 0;
  output.angular.z = srv.response.C_w;

  if (finished) {
    output.linear.x = 0;
    output.angular.z = 0;

    ROS_INFO("PLAYED TIL COMPLETION. COMPLETION IS HERE <3");
    driveMsgPublisher.publish(output);
    //ROS_INFO("DRIVE MESSAGE PUBLISHED:\nV = %f, W = %f\n\n\n\n", driveMsg.v, driveMsg.w);
    return;
  }

  driveMsgPublisher.publish(output);
  ROS_INFO("DRIVE MESSAGE PUBLISHED:\nV = %f, W = %f\n\n\n\n", srv.response.C_v, srv.response.C_w);

  // update state variables
  currLinearVelocity = srv.response.C_v;
  currAngularVelocity = srv.response.C_w;
  currHeading = constrainHeading(currHeading + (currAngularVelocity * velocityCommandUpdateRate));

  if (srv.response.C_w != 0) {
    float deltaAngle = -1 * srv.response.C_w * velocityCommandUpdateRate;
    // step 1: translate to the center
    target.x = target.x - 0;
    target.y = target.y - (srv.response.C_v / srv.response.C_w);

    // step 2: rotate by deltaAngle
    float xRotated = target.x * cos(deltaAngle) - target.y * sin(deltaAngle);
    float yRotated = target.x * sin(deltaAngle) + target.y * cos(deltaAngle);

    target.x = xRotated;
    target.y = yRotated;

    // step 3: translate back to where you were before
    target.x = target.x + 0;
    target.y = target.y + (srv.response.C_v / srv.response.C_w);
  }

  if(distanceFormula(target, constructPoint(0,0,0)) <= .5){
   finished = true;
 	}
}
void laserScanCallback(const LaserScan& scan) {
  updateWorldView(scan);
  updateVelocity();
}

void odometeryCallback(const Odometry& odom) {
  /*
  currLinearVelocity = odom.twist.twist.linear.x;
  currAngularVelocity = odom.twist.twist.angular.z;
  currHeading = constrainHeading(currHeading + (currAngularVelocity * velocityCommandUpdateRate));
  */
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "assignment5");
  ros::NodeHandle n;

  if (argc != 8) {
    ROS_INFO("LOL U DIDN'T GIVE ANY INPUT ARGS");
    ROS_INFO("CHECK MAIN TO SEE WHAT U NEED TO INPUT LOL");
    return 0;
  }
  angularWeight = float(atof(argv[1])); // 4 // 500   // 200
  freePathWeight = float(atof(argv[2])); // 1 // 1    // 1
  speedWeight = float(atof(argv[3])); // 1 // 150     // 50
  clearanceWeight = float(atof(argv[4])); // 0 // 10  // 1
  stopDistanceOffset = float(atof(argv[5]));         // 0.25
  target.x = float(atof(argv[6]));
  target.y = float(atof(argv[7]));


  // init services and subscribers
  checkPointServer = n.advertiseService("/COMPSCI403/CheckPoint", checkPoint);
  getFreePathServer = n.advertiseService("/COMPSCI403/GetFreePath", getFreePath);
  transformPointsServer = n.advertiseService("/COMPSCI403/ObstacleLaserScan", transformPoints);
  getCommandVelocityServer = n.advertiseService("/COMPSCI403/GetCommandVel", getCommandVelocity);

  laserScanSubscriber = n.subscribe("/COMPSCI403/LaserScan", 10000, laserScanCallback);
  odometrySubscriber = n.subscribe("/odom", 10000, odometeryCallback);
  getTransformClient = n.serviceClient<GetTransformationSrv>("/COMPSCI403/GetTransformation");

  driveMsgPublisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10000);

  rvizPublisher = n.advertise<Marker>("visualization_marker", 0);

  ros::spin();

  return 0;
}
