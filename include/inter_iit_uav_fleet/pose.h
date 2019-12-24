#include <tf/tf.h>
#include <GeographicLib/AzimuthalEquidistant.hpp>

//This function takes the center of the box and using the odometry and GPS location of quad, it calculates the GPS coordinates of the box
void findPose(const cv::Point &center, inter_iit_uav_fleet::Pose &box_pose)
{
    //Declaring variables
    Eigen::Matrix3d scaleUp, quadToGlob;

    //Calculating the rotation matrix to transform from quadrotor to global frame (quadToGlob)
    tf::Quaternion q1(mav_pose_.pose.pose.orientation.x, mav_pose_.pose.pose.orientation.y, mav_pose_.pose.pose.orientation.z, mav_pose_.pose.pose.orientation.w);
    Eigen::Quaterniond quat = Eigen::Quaterniond(q1.w(), q1.x(), q1.y(), q1.z());
    quadToGlob = quat.normalized().toRotationMatrix();

    //Setting up the scaleUp Matrix, which is a diagonal matrix with each diagonal element = height
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (i == j)
                scaleUp(i, j) = mav_pose_.pose.pose.position.z;
            else
                scaleUp(i, j) = 0;
        }
    }

    //Calculating the position of the box in the quadrotor fixed frame
    Eigen::Vector3d imgVec(center.x, center.y, 1);
    Eigen::Vector3d quadCoord = (camToQuad * scaleUp * invCamMatrix * imgVec) + tCam;

    //Calculating the position of the box in the Global frame attached to quad and level with the ground
    Eigen::Vector3d globCoord = quadToGlob * quadCoord;

    //Calculating and storing the GPS coordinates of the box center using the above calculated coordinates and GPS data of quadrotor
    GeographicLib::AzimuthalEquidistant obj;
    obj.Reverse(gps.latitude, gps.longitude, globCoord[0], globCoord[1], box_pose.position.x, box_pose.position.y);
    box_pose.position.z = 0;
}
