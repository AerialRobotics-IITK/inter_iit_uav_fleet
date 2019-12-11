#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Eigen>

#include <inter_iit_uav_fleet/reconfigConfig.h>
#include <dynamic_reconfigure/server.h>

#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
// #include <inter_iit_uav_fleet/UTMPose.h>
#include <inter_iit_uav_fleet/Poses.h>
#include <inter_iit_uav_fleet/signal.h>

// input image
int imageID = 0;
cv::Mat src = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);

// storage variables
nav_msgs::Odometry mav_pose_, return_pose_, home_pose_;
// inter_iit_uav_fleet::UTMPose utm_pose_, home_pose_;
inter_iit_uav_fleet::Poses obj_data;
mavros_msgs::WaypointReached prev_wp;
mavros_msgs::State mav_mode_;
geometry_msgs::PoseStamped home_msg_;
sensor_msgs::NavSatFix gps;

//camera parameters
Eigen::Matrix3d camMatrix, invCamMatrix, camToQuad, quadToCam;
Eigen::Vector3d tCam;

// hsv range variables
int HMax=90, HMin=70, SMax=255, SMin=0, VMax=255, VMin=0;

// flags
bool verbose = true;
bool isRectified = false;

// height params
double hover_height = 4.0;
double land_height = 0.41;
double descent_step = 0.4;

// Rates
double hover_time = 5.0;
double transition_time = 5.0;
double exit_time = 20.0;

double loc_error = 0.2;
double gps_error = 30;

// detector switch
int execFlag = 1;

// number of objects to detect
int totalObjects = 4;

void imageCallback(const sensor_msgs::Image msg)
{
    if(execFlag == 1)
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

bool serviceCall(inter_iit_uav_fleet::signal::Request &req, inter_iit_uav_fleet::signal::Response &res)
{
    switch(req.signal){
        case -1:
        case 0:
        case 1: execFlag = req.signal;
                switch(execFlag){
                    case 0: ROS_INFO("Paused"); break;
                    case 1: ROS_INFO("Starting"); break;
                    case -1: ROS_INFO("Exiting"); break;
                }
                res.success = true;
                break;
        default: res.success = false;
                break;
    }
    return true;
}

// state variable callbacks
void mav_pose_cb_(const nav_msgs::Odometry &msg){mav_pose_ = msg;}
void obj_cb_(const inter_iit_uav_fleet::Poses &msg){obj_data = msg;}
// void utm_pose_cb_(const inter_iit_uav_fleet::UTMPose &msg){utm_pose_ = msg;}
void wp_reached_cb_(const mavros_msgs::WaypointReached &msg){prev_wp = msg;}
void state_cb_(const mavros_msgs::State &msg){mav_mode_ = msg;}
void gpsCallback(const sensor_msgs::NavSatFix& msg){gps = msg;}

void cfgCallback(inter_iit_uav_fleet::reconfigConfig &config, uint32_t level){
    switch(level){

        case 0: isRectified = config.is_rectified;
                ROS_INFO("Set isRectified to %d", isRectified); break;
        case 1: totalObjects = config.total_objects;
                ROS_INFO("Set numObjects to %d", totalObjects);break;

            // case 1: tCam(0) = config.groups.camera_translation.t_x;
            //         ROS_INFO("Set t_x to %f", tCam(0)); break;
            // case 2: tCam(1) = config.groups.camera_translation.t_y;
            //         ROS_INFO("Set t_y to %f", tCam(1)); break;
            // case 3: tCam(2) = config.groups.camera_translation.t_z;
            //         ROS_INFO("Set t_z to %f", tCam(2)); break;

            // case 4: quadToCam(0,0) = config.groups.camera_rotation.r_xx;
            //         ROS_INFO("Set r_xx to %f", quadToCam(0,0)); break;
            // case 5: quadToCam(0,1) = config.groups.camera_rotation.r_xy;
            //         ROS_INFO("Set r_xy to %f", quadToCam(0,1)); break;
            // case 6: quadToCam(0,2) = config.groups.camera_rotation.r_xz;
            //         ROS_INFO("Set r_xz to %f", quadToCam(0,2)); break;
            // case 7: quadToCam(1,0) = config.groups.camera_rotation.r_yx;
            //         ROS_INFO("Set r_yx to %f", quadToCam(1,0)); break;
            // case 8: quadToCam(1,1) = config.groups.camera_rotation.r_yy;
            //         ROS_INFO("Set r_yy to %f", quadToCam(1,1)); break;
            // case 9: quadToCam(1,2) = config.groups.camera_rotation.r_yz;
            //         ROS_INFO("Set r_yz to %f", quadToCam(1,2)); break;
            // case 10: quadToCam(2,0) = config.groups.camera_rotation.r_zx;
            //         ROS_INFO("Set r_zx to %f", quadToCam(2,0)); break;
            // case 11: quadToCam(2,1) = config.groups.camera_rotation.r_zy;
            //         ROS_INFO("Set r_zy to %f", quadToCam(2,1)); break;
            // case 12: quadToCam(2,2) = config.groups.camera_rotation.r_zz;
            //         ROS_INFO("Set r_zz to %f", quadToCam(2,2)); break;
    
	case 12: gps_error = config.groups.params.gps_error;
		ROS_INFO("Set gps_error to %f", gps_error); break;
        case 13: loc_error = config.groups.params.loc_error;
                 ROS_INFO("Set loc_error to %f", loc_error); break;
        case 14: hover_height = config.groups.params.hover_height;
                 ROS_INFO("Set hover_height to %f", hover_height); break;
        case 15: land_height = config.groups.params.land_height;
                 ROS_INFO("Set land_height to %f", land_height); break;
        case 16: descent_step = config.groups.params.descent_step;
                 ROS_INFO("Set descent_step to %f", descent_step); break;
        case 17: hover_time = config.groups.params.hover_time;
                 ROS_INFO("Set hover_time to %f", hover_time); break;
        case 37: exit_time = config.groups.params.exit_time;
                 ROS_INFO("Set exit_time to %f", exit_time); break;
        case 39: transition_time = config.groups.params.transition_time;
                 ROS_INFO("Set transition_time to %f", transition_time); break;

        case 19: verbose = config.groups.flags.verbose;
                 ROS_INFO("Set verbose to %d", verbose); break;
        case 20: execFlag = config.groups.flags.exec;
                 ROS_INFO("Set execFlag to %d", execFlag); break;
        
        case 26: HMin = config.groups.hsv.h_min;
                 ROS_INFO("Set HMin to %d", HMin); break;
        case 27: HMax = config.groups.hsv.h_max;
                 ROS_INFO("Set HMax to %d", HMax); break;
        case 28: SMin = config.groups.hsv.s_min;
                 ROS_INFO("Set SMin to %d",SMin ); break;
        case 29: SMax = config.groups.hsv.s_max;
                 ROS_INFO("Set SMax to %d", SMax); break;
        case 30: VMin = config.groups.hsv.v_min;
                 ROS_INFO("Set VMin to %d", VMin); break;
        case 31: VMax = config.groups.hsv.v_max;
                 ROS_INFO("Set VMax to %d", VMax); break;
    }

    return;
} 
