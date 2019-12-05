#include <inter_iit_uav_fleet/callbacks.h>
#include <inter_iit_uav_fleet/outlier_filter.h>
#include <inter_iit_uav_fleet/pose.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_detect");
    ros::NodeHandle nh, ph("~");

    nh.getParam("hsvMin/HMin", HMin);
    nh.getParam("hsvMin/SMin", SMin);
    nh.getParam("hsvMin/VMin", VMin);
    nh.getParam("hsvMax/HMax", HMax);
    nh.getParam("hsvMax/SMax", SMax);
    nh.getParam("hsvMax/VMax", VMax);
    nh.getParam("initial_latitude", lat0);
    nh.getParam("initial_longitude", long0);


    std::vector<double> tempList;
    cv::Mat intrinsic = cv::Mat_<double>(3,3);
    int tempIdx=0;
    

    nh.getParam("camera_matrix/data", tempList);
    tempIdx=0;
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            intrinsic.at<double>(i,j) = tempList[tempIdx++];
        }
    }

    nh.getParam("camera/translation", tempList);
    for (int i = 0; i < 3; i++)
    {
        tCam(i) = tempList[i];
    }

    nh.getParam("camera/rotation", tempList);
    tempIdx = 0;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            quadToCam(i,j) = tempList[tempIdx++];
        }
    }
    
    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            camMatrix(i,j) = intrinsic.at<double>(i,j);
        }
    }

    invCamMatrix = camMatrix.inverse();
    camToQuad = quadToCam.inverse();

    cv::Mat src_hsv, thresholded_hsv, thresholded_hsv1, drawing, drawing1;

    std::vector<std::vector<cv::Point>> list_contours;
    std::vector<std::vector<cv::Point>> list_corners;
    std::vector<std::vector<int>> hull;
    std::vector<cv::Point> corners;

    ros::Subscriber image_sub = ph.subscribe<sensor_msgs::Image>("image", 30, imageCallback);
    ros::Subscriber GPS_sub = ph.subscribe<sensor_msgs::NavSatFix>("GPS", 30, GPS_cb_);
    ros::Subscriber compass_sub = ph.subscribe<std_msgs::Float64>("compass", 30, compass_cb_);
    ros::Subscriber odom_sub = ph.subscribe<nav_msgs::Odometry>("odom", 30, odom_cb_);

    ros::Publisher thresh_imgPub = ph.advertise<sensor_msgs::Image>("thresh_image", 1);
    ros::Publisher contour_imgPub = ph.advertise<sensor_msgs::Image>("contours", 1);
    ros::Publisher marked_imgPub = ph.advertise<sensor_msgs::Image>("marked_image", 1);

    ros::ServiceServer exec_server = ph.advertiseService("terminate", serviceCall);

    dynamic_reconfigure::Server<inter_iit_uav_fleet::reconfigConfig> cfg_server;
    dynamic_reconfigure::Server<inter_iit_uav_fleet::reconfigConfig>::CallbackType call_f = boost::bind(&cfgCallback, _1, _2);
    cfg_server.setCallback(call_f); 


    ros::Rate loop_rate(10);

    while (ros::ok()  && !(execFlag == -1))
    {
        // while(imageID < 1 && !(execFlag == -1)){ if(execFlag == 1) ros::spinOnce(); }
                
        if(execFlag == 1)
        {
            // if(!isRectified) cv::undistort(src, src, camMat, distCoeffs);

            cv::GaussianBlur(src, src, cv::Size(3, 3), 0, 0);
            cv::cvtColor(src, src_hsv, CV_BGR2HSV);
            cv::inRange(src_hsv, cv::Scalar(HMin, SMin, VMin), cv::Scalar(HMax, SMax, VMax), thresholded_hsv);

            cv::morphologyEx(thresholded_hsv, thresholded_hsv1, CV_MOP_OPEN, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 1);
            cv::findContours(thresholded_hsv, list_contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

            drawing = cv::Mat::zeros(src.size(), CV_8UC3);
            hull.resize(list_contours.size());

            for (int i = 0; i < list_contours.size(); i++)
            {
                if (cv::contourArea(list_contours.at(i)) > 0.00025 * src.rows * src.cols)
                {
                    list_corners.clear();
                    corners.clear();
                    cv::drawContours(drawing, list_contours, i, cv::Scalar(255, 255, 255));
                    cv::convexHull(list_contours.at(i), hull.at(i));

                    outlier_filter(list_contours.at(i), hull.at(i), corners);
                    list_corners.push_back(corners);

                    if (!list_corners.at(0).empty() && list_corners.at(0).size() == 4);
                        cv::drawContours(src, list_corners, 0, cv::Scalar(255, 0, 0));
                }
            }

            sensor_msgs::ImagePtr marked_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();
            sensor_msgs::ImagePtr thresh_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", thresholded_hsv1).toImageMsg();
            sensor_msgs::ImagePtr contour_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawing).toImageMsg();

            marked_imgPub.publish(marked_msg);
            thresh_imgPub.publish(thresh_msg);
            contour_imgPub.publish(contour_msg);
        }

        loop_rate.sleep();
        ros::spinOnce();
    } 

    return 0;
}
