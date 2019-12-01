#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv-3.3.1-dev/opencv2/opencv.hpp>
#include <inter_iit_uav_fleet/outlier_filter.h>

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_detect");
    ros::NodeHandle nh_private = ros::NodeHandle();

    int i, j;
    int h_low, s_low, v_low, h_high, s_high, v_high;

    nh_private.getParam("h_low", h_low);
    nh_private.getParam("s_low", s_low);
    nh_private.getParam("v_low", v_low);
    nh_private.getParam("h_high", h_high);
    nh_private.getParam("s_high", s_high);
    nh_private.getParam("v_high", v_high);

    cv::Mat src_hsv, thresholded_hsv, thresholded_hsv1, drawing, drawing1;

    std::vector<std::vector<cv::Point>> list_contours;
    std::vector<std::vector<cv::Point>> list_corners;
    std::vector<std::vector<int>> hull;
    std::vector<cv::Point> corners;

    ros::Subscriber image_sub = nh_private.subscribe<sensor_msgs::Image>("image", 30, imageCallback);
    ros::Publisher marked_imgPub = nh_private.advertise<sensor_msgs::Image>("marked_image", 1);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        cv::GaussianBlur(src, src, cv::Size(3, 3), 0, 0);
        cv::cvtColor(src, src_hsv, CV_BGR2HSV);
        cv::inRange(src_hsv, cv::Scalar(h_low, s_low, v_low), cv::Scalar(h_high, s_high, v_high), thresholded_hsv);

        cv::morphologyEx(thresholded_hsv, thresholded_hsv1, CV_MOP_OPEN, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 1);
        cv::findContours(thresholded_hsv, list_contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

        drawing = cv::Mat::zeros(src.size(), CV_8UC1);
        drawing1 = cv::Mat::zeros(src.size(), CV_8UC1);

        hull.resize(list_contours.size());

        for (i = 0; i < list_contours.size(); i++)
        {
            if (cv::contourArea(list_contours.at(i)) > 0.0025 * src.rows * src.cols)
            {
                list_corners.clear();
                corners.clear();
                cv::drawContours(drawing1, list_contours, i, cv::Scalar(255, 255, 255));
                cv::convexHull(list_contours.at(i), hull.at(i));

                outlier_filter(list_contours.at(i), hull.at(i), corners);
                list_corners.push_back(corners);

                if (!list_corners.at(0).empty() && list_corners.at(0).size() == 4);
                    cv::drawContours(src, list_corners, 0, cv::Scalar(255, 0, 0));
            }
        }

        // cv::imshow("In", src);
        // cv::imshow("contours", drawing1);
        // cv::imshow("Thresholded Image", thresholded_hsv);
        // cv::imshow("Thresholded Image1", thresholded_hsv1);
        // cv::imshow("Out", drawing);

        sensor_msgs::ImagePtr marked_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();
        marked_imgPub.publish(marked_msg);

        loop_rate.sleep();
    } 
    return 0;
}
