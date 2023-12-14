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

int counter = 0;
std::string dir = "/home/benjamin/Desktop/imagenes_termales/";

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("C.1");
  try
  {
    cv::Mat imgout;
    ROS_INFO("C.2");
    cv::normalize(cv_bridge::toCvShare(msg, "mono16")->image, imgout, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    // Extract the width, height, and number of channels
    int width = imgout.cols;
    int height = imgout.rows;
    int channels = imgout.channels();

    // Print the information
    //std::cout << "Width: " << width << std::endl;
    //std::cout << "Height: " << height << std::endl;
    //std::cout << "Number of Channels: " << channels << std::endl;
    ROS_INFO("Width: %d",width);
    ROS_INFO("Height: %d",height);
    ROS_INFO("Number of Channels: %d",channels);

    //ROS_INFO("C.3");
    //cv::resize(imgout, imgout, cv::Size(), 4, 4, cv::INTER_CUBIC);
    //ROS_INFO("C.4");
    cv::applyColorMap(imgout, imgout, 2);//COLOR_MAP_JET
    //ROS_INFO("C.5");
    cv::imshow("ThermalImage", imgout);
    std::string filename = dir + std::to_string(counter) + ".jpg";
    cv::imwrite(filename, imgout);
    counter++;
    cv::waitKey(30);
    ROS_INFO("C.6");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ROS_INFO("A");
  ros::init(argc, argv, "thermal_image_listener");
  ros::NodeHandle nh;
  ROS_INFO("B");
  cv::namedWindow("ThermalImage");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  ROS_INFO("C");
  image_transport::Subscriber sub = it.subscribe("thermal_camera/image", 1, imageCallback);
  ros::spin();
  ROS_INFO("D");
  cv::destroyWindow("ThermalImage");
}
