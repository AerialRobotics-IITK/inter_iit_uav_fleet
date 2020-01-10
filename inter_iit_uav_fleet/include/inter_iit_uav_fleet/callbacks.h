#ifndef INTER_IIT_UAV_FLEET_CALLBACKS_H
#define INTER_IIT_UAV_FLEET_CALLBACKS_H

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <inter_iit_uav_fleet/Poses.h>
#include <inter_iit_uav_fleet/reconfigConfig.h>
#include <inter_iit_uav_fleet/signal.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointReached.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <eigen3/Eigen/Eigen>

// input image
int imageID = 0;
cv::Mat src = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);

// storage variables
nav_msgs::Odometry mav_pose_, return_pose_, home_pose_;
inter_iit_uav_fleet::Poses obj_data;
mavros_msgs::WaypointReached prev_wp;
mavros_msgs::State mav_mode_;
geometry_msgs::PoseStamped home_msg_;
sensor_msgs::NavSatFix gps;

// camera parameters
Eigen::Matrix3d camMatrix, invCamMatrix, camToQuad, quadToCam;
Eigen::Vector3d tCam;

// hsv range variables
int HMax = 90, HMin = 70, SMax = 255, SMin = 0, VMax = 255, VMin = 0;

// flags
bool is_verbose = true;
bool debug = false;
bool isRectified = false;

// height params
double hover_height = 4.0;
double land_height = 0.41;
double descent_step = 0.4;

// rates
double hover_time = 5.0;
double transition_time = 5.0;
double exit_time = 20.0;

double loc_error = 0.2;
double gps_error = 30;

// detector switch
int execFlag = 1;

// number of objects to detect
int totalObjects = 4;

// callback for image subscriber
void imageCallback(const sensor_msgs::Image msg)
{
  if (execFlag == 1)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    imageID = msg.header.seq;
    src = cv_ptr->image;
  }
  return;
}

// callback for detector termination service
bool serviceCall(inter_iit_uav_fleet::signal::Request &req, inter_iit_uav_fleet::signal::Response &res)
{
  switch (req.signal)
  {
    case -1:
    case 0:
    case 1:
      execFlag = req.signal;
      switch (execFlag)
      {
        case 0:
          ROS_INFO("Paused");
          break;
        case 1:
          ROS_INFO("Starting");
          break;
        case -1:
          ROS_INFO("Exiting");
          break;
      }
      res.success = true;
      break;
    default:
      res.success = false;
      break;
  }
  return true;
}

// state variable callbacks
void mav_pose_cb_(const nav_msgs::Odometry &msg)
{
  mav_pose_ = msg;
}

void obj_cb_(const inter_iit_uav_fleet::Poses &msg)
{
  obj_data = msg;
}

void wp_reached_cb_(const mavros_msgs::WaypointReached &msg)
{
  prev_wp = msg;
}

void state_cb_(const mavros_msgs::State &msg)
{
  mav_mode_ = msg;
}

void gpsCallback(const sensor_msgs::NavSatFix &msg)
{
  gps = msg;
}

// callback for dynamic reconfigure
void cfgCallback(inter_iit_uav_fleet::reconfigConfig &config, uint32_t level)
{
  switch (level)
  {
    case 0:
      isRectified = config.is_rectified;
      ROS_INFO("Set isRectified to %d", isRectified);
      break;
    case 1:
      totalObjects = config.total_objects;
      ROS_INFO("Set numObjects to %d", totalObjects);
      break;

    case 12:
      gps_error = config.groups.params.gps_error;
      ROS_INFO("Set gps_error to %f", gps_error);
      break;
    case 13:
      loc_error = config.groups.params.loc_error;
      ROS_INFO("Set loc_error to %f", loc_error);
      break;
    case 14:
      hover_height = config.groups.params.hover_height;
      ROS_INFO("Set hover_height to %f", hover_height);
      break;
    case 15:
      land_height = config.groups.params.land_height;
      ROS_INFO("Set land_height to %f", land_height);
      break;
    case 16:
      descent_step = config.groups.params.descent_step;
      ROS_INFO("Set descent_step to %f", descent_step);
      break;
    case 17:
      hover_time = config.groups.params.hover_time;
      ROS_INFO("Set hover_time to %f", hover_time);
      break;
    case 37:
      exit_time = config.groups.params.exit_time;
      ROS_INFO("Set exit_time to %f", exit_time);
      break;
    case 39:
      transition_time = config.groups.params.transition_time;
      ROS_INFO("Set transition_time to %f", transition_time);
      break;

    case 19:
      is_verbose = config.groups.flags.is_verbose;
      ROS_INFO("Set is_verbose to %d", is_verbose);
      break;
    case 20:
      execFlag = config.groups.flags.exec;
      ROS_INFO("Set execFlag to %d", execFlag);
      break;
    case 21:
      debug = config.groups.flags.exec;
      ROS_INFO("Set debug to %d", debug);
      break;

    case 26:
      HMin = config.groups.hsv.h_min;
      ROS_INFO("Set HMin to %d", HMin);
      break;
    case 27:
      HMax = config.groups.hsv.h_max;
      ROS_INFO("Set HMax to %d", HMax);
      break;
    case 28:
      SMin = config.groups.hsv.s_min;
      ROS_INFO("Set SMin to %d", SMin);
      break;
    case 29:
      SMax = config.groups.hsv.s_max;
      ROS_INFO("Set SMax to %d", SMax);
      break;
    case 30:
      VMin = config.groups.hsv.v_min;
      ROS_INFO("Set VMin to %d", VMin);
      break;
    case 31:
      VMax = config.groups.hsv.v_max;
      ROS_INFO("Set VMax to %d", VMax);
      break;
  }

  return;
}

#endif
