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

image_transport::Publisher pub, pub2;
sensor_msgs::ImagePtr msg_out_bin, msg_out_gray;

int Threshold2Zero = 0;
int Threshold2ZeroInverted = 1;

std::string trackbar_min_value = "Min Value";
std::string trackbar_max_value = "Max Value";
std::string window_name = "ThermalImage";

//int max_bar_value = 65500;
int min_bar_value = 7500+590;
int max_bar_value = 8500-50;
int d_value = max_bar_value-min_bar_value;

int threshold_min_value = 0;
int threshold_max_value = d_value;

int threshold_min_value_ = threshold_min_value+min_bar_value;
int threshold_max_value_ = threshold_max_value+min_bar_value;

//Operador morfologico
const int morph_elem = 2;	//elipse
const int morph_size = 1;	//porte del filtro
int operation = 2;		//opening
cv::Mat element = cv::getStructuringElement( morph_elem, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );

/**
 * @function Threshold_Demo
 */
void Threshold_Demo( int, void* )
{
  threshold_min_value_ = threshold_min_value+min_bar_value;
  threshold_max_value_ = threshold_max_value+min_bar_value;
}

void LoopPixels(cv::Mat &img, cv::Mat &img_bin, cv::Mat &img_gray) {
    // Accept only char type matrices
    CV_Assert(img.depth() == CV_16U);
    img_bin = cv::Mat(img.size(), CV_8UC1);
    img_gray = cv::Mat(img.size(), CV_8UC1);
    double v, b;
    double f = double(255) / (threshold_max_value_ - threshold_min_value_);

    // Single colour
    cv::MatIterator_<ushort> it_in, end_in;
    cv::MatIterator_<uchar> it1_out, end1_out, it2_out, end2_out;

    for (it_in = img.begin<ushort>(), end_in = img.end<ushort>(), it1_out = img_gray.begin<uchar>(), end1_out = img_gray.end<uchar>(), it2_out = img_bin.begin<uchar>(), end2_out = img_bin.end<uchar>(); it_in != end_in; ++it_in, ++it1_out, ++it2_out)
    {
        if((*it_in) < threshold_min_value_ || (*it_in) > threshold_max_value_)
        {
            (*it1_out) = 0;
            (*it2_out) = 0;
        }
        else
        {
            v = ((*it_in) - threshold_min_value_) * f;
            (*it1_out) = (uchar)v;
            (*it2_out) = 255;
        }
    }
    /// Apply the specified morphology operation
    cv::morphologyEx( img_bin, img_bin, operation, element );
  
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg_in)
{
  try
  {
    /*cv::Mat image, bin, bin1, bin2, image2;
    cv_bridge::toCvShare(msg_in, "mono16")->image.convertTo(image, CV_64F);
    //cv::normalize(cv_bridge::toCvShare(msg_in, "mono16")->image, gray, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    image2 = (image - threshold_min_value_) * (255.0 / (threshold_max_value_ - threshold_min_value_));
    //cv::normalize(image2, image2, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    image2.convertTo(image2, CV_8UC1);
    

    cv::threshold( image, bin1, threshold_min_value_, 255, Threshold2Zero );
    cv::threshold( image, bin2, threshold_max_value_, 255, Threshold2ZeroInverted );
    bitwise_and(bin1,bin2,bin);
    //cv::normalize(bin, bin, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    bin.convertTo(bin, CV_8UC1);

    image2.copyTo(image2, bin);
    cv::resize(image2, image2, cv::Size(), 4, 4, cv::INTER_CUBIC);
    */
    cv::Mat image = cv_bridge::toCvShare(msg_in, "mono16")->image;
    cv::Mat gray, bin;
    LoopPixels(image, bin, gray);
    
    if(!image.empty()) {
        msg_out_bin = cv_bridge::CvImage(std_msgs::Header(), "mono8", bin).toImageMsg();
        pub.publish(msg_out_bin);
        msg_out_gray = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray).toImageMsg();
        pub2.publish(msg_out_gray);
        cv::resize(gray, gray, cv::Size(), 4, 4, cv::INTER_CUBIC);
        cv::resize(bin, bin, cv::Size(), 4, 4, cv::INTER_CUBIC);
        cv::imshow(window_name, gray);
        cv::waitKey(1);
    }
    //else std::cout << "rgb is empty" << std::endl;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg_in->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  //std::cout << "thermal_image2rgb" << std::endl;
  /// Create Trackbar to choose type of Threshold
  cv::namedWindow(window_name);
  cv::startWindowThread();
  cv::createTrackbar( trackbar_min_value,
                  window_name, &threshold_min_value,
                  d_value, Threshold_Demo );

  cv::createTrackbar( trackbar_max_value,
                  window_name, &threshold_max_value,
                  d_value, Threshold_Demo );

  /// Call the function to initialize
  Threshold_Demo( 0, 0 );

  ros::init(argc, argv, "thermal_image_threshold");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  pub = it.advertise("thermal_camera/bin", 1);
  pub2 = it.advertise("thermal_camera/imth", 1);
  image_transport::Subscriber sub = it.subscribe("thermal_camera/image", 1, imageCallback);
  ros::spin();
  cv::destroyWindow(window_name);
}
