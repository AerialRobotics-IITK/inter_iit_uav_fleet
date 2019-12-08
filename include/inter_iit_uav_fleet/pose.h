#include <tf/tf.h>
#include <GeographicLib/AzimuthalEquidistant.hpp>

void findPose(const cv::Point& center, inter_iit_uav_fleet::Pose& box_pose)
{
    Eigen::Matrix3d scaleUp, quadToGlob;
    //Eigen::Quaterniond quadToGlob_eigen;

    tf::Quaternion q1(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    Eigen::Quaterniond quat = Eigen::Quaterniond(q1.w(), q1.x(), q1.y(), q1.z());
    
    quadToGlob = quat.normalized().toRotationMatrix();

    // for (int i=0;i<3;i++){
    //     for (int j=0;j<3;j++){
    //         quadToGlob2.m_el[i][j] = quadToGlob(i,j);
    //     }
    // }

    //quadToGlob.getRPY(roll,pitch,yaw);
    // yaw = -(CV_PI/180)*(heading.data<180 ? heading.data : heading.data-360);
    // GlobtoLatLong.setRPY(0,0,yaw);
    //quadToGlob.setRPY(roll,pitch,yaw);
    //quadToGlob.getRotation(quadToLatLonquat);
    
    //tf::quaternionTFToEigen(quadToLatLonquat,quadToLatLonquat_eigen);
    //quadToLatLon_eigen = quadToLatLonquat_eigen.normalized().toRotationMatrix();

    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {
            if(i==j) scaleUp(i,j) = odom.pose.pose.position.z;
            else scaleUp(i,j) = 0;
        }
    }

    Eigen::Vector3d imgVec(center.x,center.y,1);
    Eigen::Vector3d quadCoord = (camToQuad*scaleUp*invCamMatrix*imgVec) + tCam;

    Eigen::Vector3d globCoord = quadToGlob*quadCoord;
    GeographicLib::AzimuthalEquidistant obj;
    obj.Reverse(quad_GPS.latitude,quad_GPS.longitude,globCoord[0],globCoord[1],box_pose.position.x,box_pose.position.y);
    box_pose.position.z = 0;
}
