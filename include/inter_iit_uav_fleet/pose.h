#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#define R 6387100

void findPose(const cv::Point& center, inter_iit_uav_fleet::Pose& box_pose)
{
    Eigen::Matrix3d scaleUp, quadToGlob2;
    Eigen::Quaterniond quat2;
    tf::Matrix3x3 quadToGlob;
    tf::Quaternion quadToGlobquat;
    
    tfScalar roll,pitch,yaw;

    tf::Quaternion q1(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    Eigen::Quaternionf quat = Eigen::Quaternionf(q1.w(), q1.x(), q1.y(), q1.z());
    quadToGlob.setRotation(q1);

    // for (int i=0;i<3;i++){
    //     for (int j=0;j<3;j++){
    //         quadToGlob2.m_el[i][j] = quadToGlob(i,j);
    //     }
    // }

    quadToGlob.getRPY(roll,pitch,yaw);
    yaw = -(CV_PI/180)*(heading.data<180 ? heading.data : heading.data-360);
    quadToGlob.setRPY(roll,pitch,yaw);
    quadToGlob.getRotation(quadToGlobquat);
    
    tf::quaternionTFToEigen(quadToGlobquat,quat2);
    quadToGlob2 = quat2.normalized().toRotationMatrix();

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

    Eigen::Vector3d globCoord = quadToGlob2*quadCoord;
    box_pose.position.x = (180/CV_PI)*(globCoord[0])/R + quad_GPS.longitude;
    box_pose.position.y = (180/CV_PI)*(globCoord[1])/R + quad_GPS.latitude;

}