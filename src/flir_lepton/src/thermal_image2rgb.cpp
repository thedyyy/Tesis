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
    cv::Mat gray, rgb;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_in, "mono16");
    cv::normalize(cv_ptr->image, gray, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::applyColorMap(gray, rgb, 2);//COLOR_MAP_JET
    if(!rgb.empty()) {
        cv_ptr->image = rgb;
        cv_ptr->encoding = "bgr8";
        cv_ptr->header.frame_id = "thermal"; // Asigna el frame_id
        pub.publish(cv_ptr->toImageMsg());
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg_in->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  //std::cout << "thermal_image2rgb" << std::endl;
  ros::init(argc, argv, "thermal_image2rgb");
  ros::NodeHandle nh;
  //cv::namedWindow("ThermalImage");
  //cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  pub = it.advertise("thermal_camera/rgb", 1);
  image_transport::Subscriber sub = it.subscribe("thermal_camera/image", 1, imageCallback);
  ros::spin();
  //cv::destroyWindow("ThermalImage");
}
