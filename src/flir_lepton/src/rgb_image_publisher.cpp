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
#include <zmq.h>

#define DATA_SIZE 921600
#define DATA_SIZE_2 19200
#define HEIGHT 480
#define WIDTH  640
/*#define DATA_SIZE 115200
#define DATA_SIZE_2 57600
#define HEIGHT 120
#define WIDTH  160*/

using namespace std;
using namespace cv;

void toUInt16Array(unsigned char in[DATA_SIZE], unsigned short out[DATA_SIZE_2])
{
    for(int i=0; i<DATA_SIZE; i+=2)
    {
        out[i/2] = in[i];
        out[i/2] = (out[i/2]<<8) | in[i+1];
    }
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "rgb_image_publisher");    
    ros::NodeHandle nh;

    printf ("Conectando al servidor de la camara rgb…\n");
    void *context = zmq_ctx_new ();
    void *requester = zmq_socket (context, ZMQ_REQ);
    zmq_connect (requester, "tcp://192.168.1.9:5556");
    printf ("Conectado.\n");

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("rgb_camera/image", 1); //"camera/image"
    sensor_msgs::ImagePtr msg;
    
    while(1)
    {
        unsigned char buffer [DATA_SIZE];
        //unsigned short data [DATA_SIZE_2];
        zmq_send (requester, "1", 2, 0);
        zmq_recv (requester, buffer, DATA_SIZE, 0);
        //toUInt16Array(buffer, data);

        //Mat frame(HEIGHT, WIDTH, CV_16UC1, &data);
        Mat frame(HEIGHT, WIDTH, CV_8UC3, &buffer);

        if(!frame.empty()) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
        }
    }
    
    zmq_close (requester);
    zmq_ctx_destroy (context);
    return 0;
}
