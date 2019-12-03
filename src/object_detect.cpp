
#include <inter_iit_uav_fleet/callbacks.h>
#include <inter_iit_uav_fleet/outlier_filter.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_detect");
    ros::NodeHandle nh_private = ros::NodeHandle("~");

    nh_private.getParam("HMin", HMin);
    nh_private.getParam("SMin", SMin);
    nh_private.getParam("VMin", VMin);
    nh_private.getParam("HMax", HMax);
    nh_private.getParam("SMax", SMax);
    nh_private.getParam("VMax", VMax);

    cv::Mat src_hsv, thresholded_hsv, thresholded_hsv1, drawing, drawing1;

    std::vector<std::vector<cv::Point>> list_contours;
    std::vector<std::vector<cv::Point>> list_corners;
    std::vector<std::vector<int>> hull;
    std::vector<cv::Point> corners;

    ros::Subscriber image_sub = nh_private.subscribe<sensor_msgs::Image>("image", 30, imageCallback);
    ros::Publisher thresh_imgPub = nh_private.advertise<sensor_msgs::Image>("thresh_image", 1);
    ros::Publisher contour_imgPub = nh_private.advertise<sensor_msgs::Image>("contours", 1);
    ros::Publisher marked_imgPub = nh_private.advertise<sensor_msgs::Image>("marked_image", 1);
    ros::Rate loop_rate(10);

    dynamic_reconfigure::Server<inter_iit_uav_fleet::reconfigConfig> cfg_server;
    dynamic_reconfigure::Server<inter_iit_uav_fleet::reconfigConfig>::CallbackType call_f = boost::bind(&cfgCallback, _1, _2);
    cfg_server.setCallback(call_f);

    while (ros::ok())
    {
        ros::spinOnce();
        cv::GaussianBlur(src, src, cv::Size(3, 3), 0, 0);
        cv::cvtColor(src, src_hsv, CV_BGR2HSV);
        cv::inRange(src_hsv, cv::Scalar(HMin, SMin, VMin), cv::Scalar(HMax, SMax, VMax), thresholded_hsv);

        cv::morphologyEx(thresholded_hsv, thresholded_hsv1, CV_MOP_OPEN, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 1);
        cv::findContours(thresholded_hsv, list_contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

        drawing = cv::Mat::zeros(src.size(), CV_8UC3);
        drawing1 = cv::Mat::zeros(src.size(), CV_8UC3);

        hull.resize(list_contours.size());

        for (int i = 0; i < list_contours.size(); i++)
        {
            if (cv::contourArea(list_contours.at(i)) > 0.00025 * src.rows * src.cols)
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
        sensor_msgs::ImagePtr thresh_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", thresholded_hsv1).toImageMsg();
        sensor_msgs::ImagePtr contour_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawing1).toImageMsg();

        marked_imgPub.publish(marked_msg);
        thresh_imgPub.publish(thresh_msg);
        contour_imgPub.publish(contour_msg);

        loop_rate.sleep();
    } 

    return 0;
}
