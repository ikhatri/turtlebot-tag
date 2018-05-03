#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include "cobot_simulator-master/cobot_msgs/msg_gen/cpp/include/cobot_msgs/CobotDriveMsg.h"
#include "compsci403_assignment4/ObstacleMsg.h"
#include "compsci403_assignment4/CheckPointSrv.h"
#include "compsci403_assignment4/GetFreePathSrv.h"
#include "compsci403_assignment4/GetCommandVelSrv.h"

#include <math.h> // sqrt

using cobot_msgs::CobotDriveMsg;
using compsci403_assignment4::ObstacleMsg;
using compsci403_assignment4::CheckPointSrv;
using compsci403_assignment4::GetFreePathSrv;
using compsci403_assignment4::GetCommandVelSrv;

using std::vector;
using geometry_msgs::Point32;
using sensor_msgs::LaserScan;
using visualization_msgs::Marker;

// Include any additional header or service/message files

// Declare class variables, subscribers, publishers, messages
ros::ServiceServer checkPointServer;
ros::ServiceServer getFreePathServer;
ros::ServiceServer getCommandVelocityServer;

ros::Subscriber laserScanSubscriber;

ros::Publisher obstaclePublisher;
ros::Publisher cobotDrivePublisher;
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

float lidarOffsetFromCenter = 0.145; // meters from center of robot (directly forward)
float lidarHeight = 0.23; // meters off of the ground
Point32 lidarToRobotTransform;

// State Variables
float currLinearVelocity = 0.0;
float currAngularVelocity = 0.0;
float currHeading = 0.0;
float desiredHeading = 0.0;
vector<Point32> currPoints;
float currMaxRange = 4.0;

// Weights for objective function
float angularWeight = 0;
float freePathWeight = 0;
float speedWeight = 0;
float clearanceWeight = 0;
float admissabilityOffset = 0;

float k_pi = 3.141592653589793;

Point32 target;
bool finished = false;

/* -----------------FUNCTIONS-----------------*/

// general helper
Point32 constructPoint(float x, float y, float z) {
  Point32 returnVal;
  returnVal.x = x;
  returnVal.y = y;
  returnVal.z = z;
  return returnVal;
}

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
Marker getMarker(float r, float g, float b, float id) {
  Marker marker;

  marker.header.frame_id = "/base_footprint";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = id;
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

void drawCircle(float centerX, float centerY, float radius, float r, float g, float b, float id) {
  // init marker
  Marker circle = getMarker(r, g, b, id);
  circle.scale.x = 0.03;
  circle.type = Marker::LINE_STRIP;
  circle.pose.position.x = centerX;
  circle.pose.position.y = centerY;
  circle.pose.position.z = 1;

  float tau = 2 * k_pi;
  float numOfSegments = 16.0;
  float stepSize = tau / numOfSegments;
  for (float theta = 0; theta <= (tau + stepSize); theta += stepSize) {
    geometry_msgs::Point p;
    p.x = radius * cos(theta);
    p.y = radius * sin(theta);
    p.z = 0;
    circle.points.push_back(p);
  }
  rvizPublisher.publish(circle);
}

void drawPoints(float r, float g, float b, float id, vector<Point32> points) {
  Marker marker = getMarker(r, g, b, id);
  for (unsigned int i = 0; i < points.size(); i++) {
    geometry_msgs::Point p;
    p.x = points[i].x;
    p.y = points[i].y;
    p.z = points[i].z;
    marker.points.push_back(p);
  }
  rvizPublisher.publish(marker);
}

// Define service and callback functions

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
  CheckPointSrv srv;
  srv.request.v = v;
  srv.request.w = w;

  for (unsigned int i = 0; i < points.size(); i++) {
    srv.request.P = points[i];
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
  Point32 noTransform = constructPoint(0.0, 0.0, 0.0);
  vector<Point32> points = laserScanToPoints(input.laser_scan, noTransform);
  vector<CheckPointSrv> pointInfo = checkAllPoints(points, input.v, input.w);
  output = getShortestDistance(pointInfo, input.laser_scan.range_max, true);
  return true;
}

bool hasEnoughRoomToStop(float v, float w, const vector<CheckPointSrv>& pointInfo) {
  GetFreePathSrv::Response freePathInfo = getShortestDistance(pointInfo, currMaxRange, true);
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
  distanceNeededToStop += admissabilityOffset;

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

// currently unused
float calculateCost(float v, float w, bool print) {
  /*
  // get free path for the given v & w. (note that currScan is updated in publishToObstaclesTopic())
  // distance cost (high cost = small distance)
  GetFreePathSrv srv;
  srv.request.laser_scan = currScan;
  srv.request.v = v;
  srv.request.w = w;
  getFreePathHelper(srv.request, srv.response, lidarToRobotTransform, true);
  float distanceToObstaclesCost = 1.0 / srv.response.free_path_length;

  // get the shortest distance of an obstacle not on the current path to the robot
  // here a large distance corresponds to a small cost,
  // and a small distance corresponds to a large cost
  getFreePathHelper(srv.request, srv.response, lidarToRobotTransform, false);
  float clearanceCost = 1.0 / srv.response.free_path_length;
  //distanceToObstaclesCost += 1.0 / srv.response.free_path_length;

  // angle cost
  float proposedHeading = currHeading + w * velocityCommandUpdateRate;
  float angularCost = fabs(proposedHeading);

  // speed cost
  float speedCost = maxLinearVelocity - v;

  if (print) {
    ROS_INFO("ANGULAR COST: %f", angularCost);
    ROS_INFO("DISTANCE COST: %f", distanceToObstaclesCost);
    ROS_INFO("SPEED COST: %f", speedCost);
    ROS_INFO("CLEARANCE COST: %f", clearanceCost);
  }
  return (angularWeight * angularCost) + (freePathWeight * distanceToObstaclesCost) + (speedWeight * speedCost) + (clearanceWeight * clearanceCost);
  */
  return 0;
}

// objective function
float calculateScore(float v, float w, vector<CheckPointSrv>& pointInfo, bool print, bool draw) {
  // get free path for the given v & w. (note that currScan is updated in publishToObstaclesTopic())
  // distance score (high score = longest free path)
  //vector<CheckPointSrv> pointInfo = checkAllPoints(points, v, w);
  float freePathLength = getShortestDistance(pointInfo, currMaxRange, true).free_path_length;
  float distanceToObstaclesScore = freePathLength * freePathWeight;

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
  float clearance = getShortestDistance(pointInfo, currMaxRange, false).free_path_length;
  if (clearance < 0.2) {
    return -1;
  }
  float clearanceScore = clearance * clearanceWeight;

  // angle score (highest score is heading of 0, score gets worse as heading deviates from 0)
  // negate the result from atan2
  float headingToTarget = atan2(target.y, target.x);
  float proposedHeading = constrainHeading(headingToTarget - w * velocityCommandUpdateRate);
  // we use (-w * velocityCommandUpdateRate becasue positive headingToTarget requires positive w,
  // and negative headingToTarget requires negative w).
  float angularScore = (k_pi - fabs(proposedHeading)) * angularWeight;

  // speed cost (high score = high speed)
  float speedScore = v * speedWeight;

  float totalScore = angularScore + distanceToObstaclesScore + speedScore + clearanceScore;

  if (print) {
    ROS_INFO("V: %f, W: %f", v, w);
    ROS_INFO("ANGULAR SCORE: %f", angularScore);
    ROS_INFO("PROPOSED HEADING: %f", proposedHeading);
    ROS_INFO("DISTANCE SCORE: %f", distanceToObstaclesScore);
    ROS_INFO("SPEED SCORE: %f", speedScore);
    ROS_INFO("CLEARANCE SCORE: %f", clearanceScore);
    ROS_INFO("TOTAL SCORE: %f\n", totalScore);
  }

  if (draw) {
    vector<Point32> obstacles;
    for (unsigned int i = 0; i < pointInfo.size(); i++) {
      if (pointInfo[i].response.is_obstacle) {
        obstacles.push_back(pointInfo[i].request.P);
      }
    }
    drawPoints(1.0, 0, 0, 0, obstacles);

    if (w != 0) {
      drawCircle(0, (v / w), fabs(v / w), 1, 1, 0, 1);
      drawCircle(0, (v / w), fabs(v / w)-robotRadius, 1, 1, 0, 2);
      drawCircle(0, (v / w), fabs(v / w)+robotRadius, 1, 1, 0, 3);
      drawCircle(target.x, target.y, robotRadius, 1, 0, 1, 10);
    }
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
  //ROS_INFO("INITIAL SCORE CALCULATION");
  vector<CheckPointSrv> pointInfo = checkAllPoints(currPoints, output.C_v, output.C_w);
  float maxScore = calculateScore(output.C_v, output.C_w, pointInfo, false, false);

  // set dynamic window stepsize (resolution)
  float numOfSteps = 15.0; // actual number of discrete velocites checked should theoretically this value squared.
  // however, this wasn't alwasy observed, likely because ranges aren't just between integers.
  float linearVelocityStepSize = (linearVelocityUpperBound - linearVelocityLowerBound) / numOfSteps;
  float angularVelocityStepSize = (angularVelocityUpperBound - angularVelocityLowerBound) / numOfSteps;

  // iterate through all discretized combinations of V and W
  for (float currV = linearVelocityLowerBound; currV <= linearVelocityUpperBound; currV += linearVelocityStepSize) {
    for (float currW = angularVelocityLowerBound; currW <= angularVelocityUpperBound; currW += angularVelocityStepSize) {
      pointInfo = checkAllPoints(currPoints, currV, currW);

      if (hasEnoughRoomToStop(currV, currW, pointInfo)) {
        float score = calculateScore(currV, currW, pointInfo, false, false);

        if (score > maxScore) {
          maxScore = score;
          output.C_v = currV;
          output.C_w = currW;
        }
      }
    }
  }
  ROS_INFO("MAX SCORE FOUND");
  pointInfo = checkAllPoints(currPoints, output.C_v, output.C_w);
  calculateScore(output.C_v, output.C_w, pointInfo, true, true); // this is here just to print individual scores
  return true;
}

// obstacle laser scan service callback
void publishToObstaclesTopic(const LaserScan& scan) {
  ObstacleMsg outputMsg;
  outputMsg.header = scan.header;
  outputMsg.header.frame_id = "/base_footprint";

  // get points as they viewed from the frame of the robot (update global copy)
  currPoints = laserScanToPoints(scan, lidarToRobotTransform);

  outputMsg.obstacle_points = currPoints;
  obstaclePublisher.publish(outputMsg);
}

void publishToCobotDriveTopic(const LaserScan& scan) {
  // get desired velocity from getCommandVelocity
  GetCommandVelSrv srv;
  srv.request.v_0 = currLinearVelocity;
  srv.request.w_0 = currAngularVelocity;
  getCommandVelocity(srv.request, srv.response);

  // publish desired velocity
  CobotDriveMsg driveMsg;
  driveMsg.v = srv.response.C_v;
  driveMsg.w = srv.response.C_w;
  if (finished) {
    driveMsg.v = 0;
    driveMsg.w = 0;
    ROS_INFO("PLAYED TIL COMPLETION. COMPLETION IS HERE <3");
    cobotDrivePublisher.publish(driveMsg);
    //ROS_INFO("DRIVE MESSAGE PUBLISHED:\nV = %f, W = %f\n\n\n\n", driveMsg.v, driveMsg.w);
    return;
  }
  cobotDrivePublisher.publish(driveMsg);
  ROS_INFO("DRIVE MESSAGE PUBLISHED:\nV = %f, W = %f\n\n\n\n", driveMsg.v, driveMsg.w);

  // updtate current values of v and w for next iteration
  currLinearVelocity = srv.response.C_v;
  currAngularVelocity = srv.response.C_w;
  currHeading = constrainHeading(currHeading + (srv.response.C_w * velocityCommandUpdateRate));
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

// callback for subcription to "/Cobot/Laser" topic
void laserScanCallback(const LaserScan& scan) {
  publishToObstaclesTopic(scan);
  publishToCobotDriveTopic(scan);
}

int main(int argc, char **argv) {
  // standard ROS init stuff
  ros::init(argc, argv, "assignment4");
  ros::NodeHandle n;

  // init member variables
  lidarToRobotTransform.x = lidarOffsetFromCenter;
  lidarToRobotTransform.y = 0;
  lidarToRobotTransform.z = lidarHeight;

  rvizPublisher = n.advertise<Marker>("visualization_marker", 0);

	// Perform operations defined in Assignment 4
  checkPointServer = n.advertiseService("/COMPSCI403/CheckPoint", checkPoint);
  getFreePathServer = n.advertiseService("/COMPSCI403/GetFreePath", getFreePath);
  getCommandVelocityServer = n.advertiseService("/COMPSCI403/GetCommandVel", getCommandVelocity);

  obstaclePublisher = n.advertise<ObstacleMsg>("/COMPSCI403/Obstacles", 10000);
  laserScanSubscriber = n.subscribe("/Cobot/Laser", 10000, laserScanCallback);

  cobotDrivePublisher = n.advertise<cobot_msgs::CobotDriveMsg>("/Cobot/Drive", 10000);

  if (argc != 8) {
    ROS_INFO("LOL U DIDN'T GIVE ANY INPUT ARGS");
    ROS_INFO("CHECK MAIN TO SEE WHAT U NEED TO INPUT LOL");
    return 0;
  }
  angularWeight = float(atof(argv[1])); // 4 // 500   // 200   // 1000           // 57
  freePathWeight = float(atof(argv[2])); // 1 // 1    // 1     // 1              // 0.1
  speedWeight = float(atof(argv[3])); // 1 // 150     // 50    // 1000 (or 250)  // 15
  clearanceWeight = float(atof(argv[4])); // 0 // 10  // 1    // 30              // 1
  //robotRadius = robotRadius * float(atof(argv[5]));
  admissabilityOffset = float(atof(argv[5]));        // 0.25  //0.25             // 4.2
  target.x = float(atof(argv[6]));
  target.y = float(atof(argv[7]));

	//ros::spin();

  ros::Rate loop(1.0 / velocityCommandUpdateRate);

  while (ros::ok()) {
    ros::spinOnce();
    loop.sleep();
  }

  return(0);
}
