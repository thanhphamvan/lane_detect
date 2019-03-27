#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>

#include <opencv2/highgui/highgui.hpp>
#include <vector>

#include "detectlane.h"
#include "carcontrol.h"

using namespace std;

DetectLane *detect;
CarControl *car;

bool RUN = false;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        Mat img = detect->update(cv_ptr->image);

        cv::imshow("View", img);
        waitKey(1);

        float error = detect->getErrorAngle();
        if (RUN) car->driverCar(error);
        else car->stop();
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void button1Press(const std_msgs::Bool::ConstPtr &msg)
{
    bool data = msg->data;
    if (data)
    {
        RUN = true;
    }
}

void sensorTrigger(const std_msgs::Bool::ConstPtr &msg)
{
    bool data = msg->data;
    if (!data)
    {
        RUN = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    cv::namedWindow("View");

    detect = new DetectLane();
    car = new CarControl();

    cv::startWindowThread();

    ros::NodeHandle nh1, nh2, nh3;
    image_transport::ImageTransport it(nh1);
    image_transport::Subscriber sub1 = it.subscribe("camera/rgb/image_raw", 1, imageCallback);

    ros::Subscriber sub2 = nh2.subscribe("/bt1_status", 10, button1Press);
    ros::Subscriber sub3 = nh2.subscribe("/ss_status", 10, sensorTrigger);

    ros::spin();

    cv::destroyAllWindows();
}
