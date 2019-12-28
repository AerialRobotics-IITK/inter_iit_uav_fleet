#include <inter_iit_uav_fleet/callbacks.h>
#include <inter_iit_uav_fleet/outlier_filter.h>
#include <inter_iit_uav_fleet/pose.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_detect");
  ros::NodeHandle nh, ph("~");

  // get HSV threshold parameters
  nh.getParam("hsvMin/HMin", HMin);
  nh.getParam("hsvMin/SMin", SMin);
  nh.getParam("hsvMin/VMin", VMin);
  nh.getParam("hsvMax/HMax", HMax);
  nh.getParam("hsvMax/SMax", SMax);
  nh.getParam("hsvMax/VMax", VMax);

  // get camera parameters
  std::vector<double> tempList;
  cv::Mat intrinsic = cv::Mat_<double>(3, 3);
  int tempIdx = 0;

  nh.getParam("camera_matrix/data", tempList);
  tempIdx = 0;
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      intrinsic.at<double>(i, j) = tempList[tempIdx++];
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
      quadToCam(i, j) = tempList[tempIdx++];
    }
  }

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      camMatrix(i, j) = intrinsic.at<double>(i, j);
    }
  }

  // precalculate camera matrices for pose transformations
  invCamMatrix = camMatrix.inverse();
  camToQuad = quadToCam.inverse();

  // storage variables
  cv::Mat src_hsv, thresholded_hsv, thresholded_hsv1, drawing, drawing1;
  std::vector<std::vector<cv::Point>> list_contours;
  std::vector<std::vector<cv::Point>> list_corners;
  std::vector<std::vector<int>> hull;
  std::vector<cv::Point> corners;

  ros::Subscriber image_sub = ph.subscribe<sensor_msgs::Image>("image", 10, imageCallback);
  ros::Subscriber GPS_sub = ph.subscribe("GPS", 5, gpsCallback);
  ros::Subscriber mav_pose_sub_ = ph.subscribe("odom", 10, mav_pose_cb_);

  ros::Publisher thresh_imgPub = ph.advertise<sensor_msgs::Image>("thresh_image", 1);
  ros::Publisher contour_imgPub = ph.advertise<sensor_msgs::Image>("contours", 1);
  ros::Publisher marked_imgPub = ph.advertise<sensor_msgs::Image>("marked_image", 1);
  ros::Publisher obj_gpsPub = ph.advertise<inter_iit_uav_fleet::Poses>("obj_gps", 1);

  ros::ServiceServer exec_server = ph.advertiseService("terminate", serviceCall);

  // startup dynamic reconfigure server
  dynamic_reconfigure::Server<inter_iit_uav_fleet::reconfigConfig> cfg_server;
  dynamic_reconfigure::Server<inter_iit_uav_fleet::reconfigConfig>::CallbackType call_f =
      boost::bind(&cfgCallback, _1, _2);
  cfg_server.setCallback(call_f);

  ros::Rate loop_rate(10);

  inter_iit_uav_fleet::Poses pose_msg;

  while (ros::ok() && !(execFlag == -1))
  {
    // wait for image
    while (imageID < 1 && !(execFlag == -1))
    {
      if (execFlag == 1)
        ros::spinOnce();
    }

    if (execFlag == 1)
    {
      pose_msg.object_poses.clear();  // clear buffer

      // process image
      cv::GaussianBlur(src, src, cv::Size(3, 3), 0, 0);
      cv::cvtColor(src, src_hsv, CV_BGR2HSV);
      cv::inRange(src_hsv, cv::Scalar(HMin, SMin, VMin), cv::Scalar(HMax, SMax, VMax), thresholded_hsv);

      cv::morphologyEx(thresholded_hsv, thresholded_hsv1, CV_MOP_OPEN,
                       getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 1);
      cv::findContours(thresholded_hsv, list_contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

      drawing = cv::Mat::zeros(src.size(), CV_8UC3);
      hull.resize(list_contours.size());

      for (int i = 0; i < list_contours.size(); i++)
      {
        inter_iit_uav_fleet::Pose box_pose;
        if ((box_pose.area = cv::contourArea(list_contours.at(i))) > 0.00025 * src.rows * src.cols)  // area check
        {
          list_corners.clear();
          corners.clear();  // clear buffers
          if (debug)
            cv::drawContours(drawing, list_contours, i, cv::Scalar(255, 255, 255));

          cv::convexHull(list_contours.at(i), hull.at(i));

          outlier_filter(list_contours.at(i), hull.at(i), corners);  // extract corners from hull
          list_corners.push_back(corners);

          if (!list_corners.at(0).empty() && list_corners.at(0).size() == 4)  // quadrilateral check
          {
            if (debug)
              cv::drawContours(src, list_corners, 0, cv::Scalar(255, 0, 0));
            cv::Point center((list_corners.at(0).at(0).x + list_corners.at(0).at(1).x + list_corners.at(0).at(2).x +
                              list_corners.at(0).at(3).x) /
                                 4,
                             (list_corners.at(0).at(0).y + list_corners.at(0).at(1).y + list_corners.at(0).at(2).y +
                              list_corners.at(0).at(3).y) /
                                 4);
            box_pose.boxID = i;
            findPose(center, box_pose);  // calculate GPS coordinates
            pose_msg.object_poses.push_back(box_pose);
          }
        }
      }

      if (debug)
      {
        sensor_msgs::ImagePtr marked_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();
        sensor_msgs::ImagePtr thresh_msg =
            cv_bridge::CvImage(std_msgs::Header(), "mono8", thresholded_hsv1).toImageMsg();
        sensor_msgs::ImagePtr contour_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawing).toImageMsg();
        marked_imgPub.publish(marked_msg);
        thresh_imgPub.publish(thresh_msg);
        contour_imgPub.publish(contour_msg);
      }

      pose_msg.imageID = imageID;
      pose_msg.stamp = ros::Time::now();
      obj_gpsPub.publish(pose_msg);
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
