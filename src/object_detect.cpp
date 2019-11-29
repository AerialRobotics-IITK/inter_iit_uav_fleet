#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv-3.3.1-dev/opencv2/opencv.hpp>
#include </home/tanay/src/catkin_ws/src/inter_iit_uav_fleet/include/inter_iit_uav_fleet/outlier_filter.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_detect");
    ros::NodeHandle nh_private = ros::NodeHandle("~");

    int i, j;
    int h_low, s_low, v_low, h_high, s_high, v_high;

    nh_private.getParam("h_low", h_low);
    nh_private.getParam("s_low", s_low);
    nh_private.getParam("v_low", v_low);
    nh_private.getParam("h_high", h_high);
    nh_private.getParam("s_high", s_high);
    nh_private.getParam("v_high", v_high);
    
    cv::Mat src, src_hsv, thresholded_hsv, thresholded_hsv1, drawing, drawing1;

    std::vector<std::vector<cv::Point> > list_contours;
    std::vector<std::vector<cv::Point> > list_corners;
    std::vector<std::vector<int> > hull;
    std::vector<cv::Point> corners;

    cv::VideoCapture cap;
    cap.open(1);

    while(1)    
    {
        cap >> src;
        // src = cv::imread("/home/tanay/src/catkin_ws/src/inter_iit_uav_fleet/etc/image_2.png");
        cv::GaussianBlur(src, src, cv::Size(3, 3), 0, 0);
        cv::cvtColor(src, src_hsv, CV_BGR2HSV);
        cv::inRange(src_hsv, cv::Scalar(h_low, s_low, v_low), cv::Scalar(h_high, s_high, v_high), thresholded_hsv);
        
        cv::morphologyEx(thresholded_hsv, thresholded_hsv1, CV_MOP_OPEN, getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)), cv::Point(-1, -1), 1); 
        cv::findContours(thresholded_hsv, list_contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

        drawing = cv::Mat::zeros(src.size(), CV_8UC1);
        drawing1 = cv::Mat::zeros(src.size(), CV_8UC1);

        hull.resize(list_contours.size());

        for(i=0; i<list_contours.size(); i++)
        {
            if(cv::contourArea(list_contours.at(i))>0.0025*src.rows*src.cols)
            {
                list_corners.clear();
                corners.clear();
                cv::drawContours(drawing1, list_contours, i, cv::Scalar(255, 255, 255));
                cv::convexHull(list_contours.at(i), hull.at(i));

                outlier_filter(list_contours.at(i), hull.at(i), corners);
                list_corners.push_back(corners);

                if(!list_corners.at(0).empty() && list_corners.at(0).size()==4)
                    cv::drawContours(drawing, list_corners, 0, cv::Scalar(255, 255, 255));
            }
        }

        cv::imshow("In", src);
        cv::imshow("contours", drawing1);
        cv::imshow("Thresholded Image", thresholded_hsv);
        cv::imshow("Thresholded Image1", thresholded_hsv1);
        cv::imshow("Out", drawing);
        if((char)cv::waitKey(30)=='q')
            return 0;
    } 
    return 0;
}