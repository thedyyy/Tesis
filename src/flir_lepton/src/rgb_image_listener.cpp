//  Img Publisher
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>

//ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// Third party
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <cv.h>
//#include <highgui.h>


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    //cv::Mat imgout = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat imgout;
    cv::flip(cv_bridge::toCvShare(msg, "bgr8")->image, imgout, -1);
    //cv::normalize(, imgout, 0, 255, cv::NORM_MINMAX, CV_8UC3);
    //cv::resize(imgout, imgout, cv::Size(), 4, 4, cv::INTER_CUBIC);
    //cv::applyColorMap(imgout, imgout, 2);//COLOR_MAP_JET
    cv::imshow("rgbImage", imgout);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rgb_image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("rgbImage");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("rgb_camera/image", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("rgbImage");
}
