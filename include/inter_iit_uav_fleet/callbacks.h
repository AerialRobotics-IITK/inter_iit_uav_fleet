#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <inter_iit_uav_fleet/reconfigConfig.h>
#include <dynamic_reconfigure/server.h>

cv::Mat src = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);

void imageCallback(const sensor_msgs::Image msg)
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

    src = cv_ptr->image;

    return;
}

int HMax=90, HMin=70, SMax=255, SMin=0, VMax=255, VMin=0;

void cfgCallback(inter_iit_uav_fleet::reconfigConfig &config, uint32_t level){
    switch(level){

        // case 0: isRectified = config.is_rectified;
        //         ROS_INFO("Set isRectified to %d", isRectified); break;

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
    
        // case 13: minSize = config.groups.box.minSize;
        //          ROS_INFO("Set minSize to %d", minSize); break;
        // case 14: maxAreaIndex = config.groups.box.maxAreaIndex;
        //          ROS_INFO("Set maxAreaIndex to %f", maxAreaIndex); break;
        // case 15: maxEigenIndex = config.groups.box.maxEigenIndex;
        //          ROS_INFO("Set maxEigenIndex to %f", maxEigenIndex); break;
        // case 16: maxDiagIndex = config.groups.box.maxDiagIndex;
        //          ROS_INFO("Set maxDiagIndex to %f", maxDiagIndex); break;
        // case 17: centreCorrectIndex = config.groups.box.centreCorrectIndex;
        //          ROS_INFO("Set centreCorrectIndex to %f", centreCorrectIndex); break;
        // case 37: maxCentreDist = config.groups.box.maxCentreDist;
        //          ROS_INFO("Set maxCentreDist to %f", maxCentreDist); break;
        // case 39: maxStoreVel = config.groups.box.maxStoreVel;
        //          ROS_INFO("Set maxStoreVel to %f", maxStoreVel); break;

        // case 18: debug = config.groups.flags.debug;
        //          ROS_INFO("Set debug to %d", debug); break;
        // case 19: verbose = config.groups.flags.verbose;
        //          ROS_INFO("Set verbose to %d", verbose); break;
        // case 20: areaCheckFlag = config.groups.flags.areaCheck;
        //          ROS_INFO("Set areaCheckFlag to %d", areaCheckFlag); break;
        // case 21: eigenCheckFlag = config.groups.flags.eigenCheck;
        //          ROS_INFO("Set eigenCheckFlag to %d", eigenCheckFlag); break;
        // case 22: diagCheckFlag = config.groups.flags.diagCheck;
        //          ROS_INFO("Set diagCheckFlag to %d", diagCheckFlag); break;
        // case 23: sizeCheckFlag = config.groups.flags.sizeCheck;
        //          ROS_INFO("Set sizeCheckFlag to %d", sizeCheckFlag); break;
        // case 24: centreCorrect = config.groups.flags.centreCorrect;
        //          ROS_INFO("Set centreCorrect to %d", centreCorrect); break;

        // case 25: color_num = config.groups.hsv.color; break;
        
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

        // case 26: switch(color_num){
        //             case 0: RHMin = (config.groups.hsv.h_min + 90) % 180;
        //                     ROS_INFO("Set RHMin to %d", RHMin); break;
        //             case 1: BHMin = (config.groups.hsv.h_min + 90) % 180;
        //                     ROS_INFO("Set BHMin to %d", BHMin); break;
        //             case 2: YHMin = (config.groups.hsv.h_min + 90) % 180;
        //                     ROS_INFO("Set YHMin to %d", YHMin); break;
        //             case 3: OHMin = (config.groups.hsv.h_min + 90) % 180;
        //                     ROS_INFO("Set OHMin to %d", OHMin); break;
        //          } break;
        
        // case 27: switch(color_num){
        //             case 0: RHMax = (config.groups.hsv.h_max + 90) % 180;
        //                     ROS_INFO("Set RHMax to %d", RHMax); break;
        //             case 1: BHMax = (config.groups.hsv.h_max + 90) % 180;
        //                     ROS_INFO("Set BHMax to %d", BHMax); break;
        //             case 2: YHMax = (config.groups.hsv.h_max + 90) % 180;
        //                     ROS_INFO("Set YHMax to %d", YHMax); break;
        //             case 3: OHMax = (config.groups.hsv.h_max + 90) % 180;
        //                     ROS_INFO("Set OHMax to %d", OHMax); break;        
        //          } break;
        
        // case 28: switch(color_num){
        //             case 0: RSMin = config.groups.hsv.s_min;
        //                     ROS_INFO("Set RSMin to %d", RSMin); break;
        //             case 1: BSMin = config.groups.hsv.s_min;
        //                     ROS_INFO("Set BSMin to %d", BSMin); break;
        //             case 2: YSMin = config.groups.hsv.s_min;
        //                     ROS_INFO("Set YSMin to %d", YSMin); break;
        //             case 3: OSMin = config.groups.hsv.s_min;
        //                     ROS_INFO("Set OSMin to %d", OSMin); break;
        //          } break;
        
        // case 29: switch(color_num){
        //             case 0: RSMax = config.groups.hsv.s_max;
        //                     ROS_INFO("Set RSMax to %d", RSMax); break;
        //             case 1: BSMax = config.groups.hsv.s_max;
        //                     ROS_INFO("Set BSMax to %d", BSMax); break;
        //             case 2: YSMax = config.groups.hsv.s_max;
        //                     ROS_INFO("Set YSMax to %d", YSMax); break;
        //             case 3: OSMax = config.groups.hsv.s_max;
        //                     ROS_INFO("Set OSMax to %d", OSMax); break;
        //          } break;
        
        // case 30: switch(color_num){
        //             case 0: RVMin = config.groups.hsv.v_min;
        //                     ROS_INFO("Set RVMin to %d", RVMin); break;
        //             case 1: BVMin = config.groups.hsv.v_min;
        //                     ROS_INFO("Set BVMin to %d", BVMin); break;
        //             case 2: YVMin = config.groups.hsv.v_min;
        //                     ROS_INFO("Set YVMin to %d", YVMin); break;
        //             case 3: OVMin = config.groups.hsv.v_min;
        //                     ROS_INFO("Set OVMin to %d", OVMin); break;
        //          } break;
        
        // case 31: switch(color_num){
        //             case 0: RVMax = config.groups.hsv.v_max;
        //                     ROS_INFO("Set RVMax to %d", RVMax); break;
        //             case 1: BVMax = config.groups.hsv.v_max;
        //                     ROS_INFO("Set BVMax to %d", BVMax); break;
        //             case 2: YVMax = config.groups.hsv.v_max;
        //                     ROS_INFO("Set YVMax to %d", YVMax); break;
        //             case 3: OVMax = config.groups.hsv.v_max;
        //                     ROS_INFO("Set OVMax to %d", OVMax); break;
        //          } break;

        // case 32: redSize = config.groups.sizes.r_size;
        //          ROS_INFO("Set redSize to %f", redSize); break;
        // case 33: blueSize = config.groups.sizes.b_size;
        //          ROS_INFO("Set blueSize to %f", blueSize); break;
        // case 34: yellowSize = config.groups.sizes.y_size;
        //          ROS_INFO("Set yellowSize to %f", yellowSize); break;
        // case 38: orangeSize = config.groups.sizes.o_size;
        //          ROS_INFO("Set orangeSize to %f", orangeSize); break;
        // case 35: delSize = config.groups.sizes.del_size;
        //          ROS_INFO("Set delSize to %f", delSize); break;
        // case 36: minSizeHeight = config.groups.sizes.min_height;
        //          ROS_INFO("Set minSizeHeight to %f", minSizeHeight); break;


    }

    return;
} 