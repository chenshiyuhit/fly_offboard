#ifndef FLYTRACK
#define FLYTRACK


#include "ros/ros.h"
#include "ros/time.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include "tf/transform_broadcaster.h"

#include <math.h>
#include <eigen3/Eigen/Eigen>

#include <stdio.h>



bool search_flag = true;
bool track_flag  = false;
bool lost_flag   = false;

float x;
float y;
float theta;

float uav_x;
float uav_y;
float uav_z;

geometry_msgs::PoseStamped pose_pub_;

using namespace Eigen;


#endif // FLYTRACK
