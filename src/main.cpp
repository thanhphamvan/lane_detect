#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "lane_detect/SignInfo.h"

#include <opencv2/highgui/highgui.hpp>
#include <vector>

#include "detectlane.h"
#include "carcontrol.h"

using namespace std;

DetectLane *detect;
CarControl *car;

int countNoneSign = 0;
int signDir = 0;
int signSignal = 0;
int frameSkip = 0;

vector<lane_detect::SignInfo::ConstPtr> signs;

void sign_callback(const lane_detect::SignInfo::ConstPtr &msg)
{
    if (msg->sign_label == -1)
    {
        countNoneSign++;
        if (countNoneSign >= 3)
        {
            signDir = 0;
            signs.clear();
            countNoneSign = 0;
        }
    }
    else
    {
        signs.push_back(msg);
        countNoneSign = 0;
    }

    if (signs.size() > 3)
    {
        int dir = 0;
        for (int i = 0; i < signs.size(); i++)
        {
            dir += signs[i]->sign_label == 0 ? 1 : -1;
        }
        if (dir > 0)
        {
            signDir = 1;
            signSignal = 90;
        }
        else
        {
            signDir = -1;
            signSignal = 90;
        }
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    
    if (signSignal > 0) signSignal--;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        Mat img = detect->update(cv_ptr->image, signDir);

        cv::imshow("View", img);
        waitKey(1);

        float error = detect->getErrorAngle(signDir);

        car->driverCar(error, signSignal);
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
    cv::namedWindow("Canny");
    cv::namedWindow("Threshold");

    detect = new DetectLane();
    car = new CarControl();

    cv::startWindowThread();

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("Team1_image", 1, imageCallback);

    ros::Subscriber sign_subscriber = nh.subscribe("/sign_detect", 1, sign_callback);

    ros::spin();

    cv::destroyAllWindows();
}
