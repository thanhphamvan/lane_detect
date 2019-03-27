#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <vector>

#include "detectlane.h"
#include "carcontrol.h"

using namespace std;

//DetectSign *signDetect;
DetectLane *detect;
CarControl *car;

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
        car->driverCar(error);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    cv::namedWindow("View");

    detect = new DetectLane();
    car = new CarControl();

    cv::startWindowThread();

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, imageCallback);

    ros::spin();

    cv::destroyAllWindows();
}
