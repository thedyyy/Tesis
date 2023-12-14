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

image_transport::Publisher pub;
sensor_msgs::ImagePtr msg_out;

void imageCallback(const sensor_msgs::ImageConstPtr& msg_in)
{
  try
  {
    cv::Mat gray;
    cv::normalize(cv_bridge::toCvShare(msg_in, "mono16")->image, gray, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    if(!gray.empty()) {
        msg_out = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray).toImageMsg();
        pub.publish(msg_out);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg_in->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thermal_image2gray");
  ros::NodeHandle nh;
  //cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  pub = it.advertise("thermal_camera/gray", 1);
  image_transport::Subscriber sub = it.subscribe("thermal_camera/image", 1, imageCallback);
  ros::spin();
}
