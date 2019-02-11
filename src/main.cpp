#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <vector>

#include "detectsign.h"
#include "detectlane.h"
#include "carcontrol.h"

using namespace std;

DetectSign *signDetect;
DetectLane *detect;
CarControl *car;

void drawSign(Mat &image)
{
    if (signDetect->currentSign == NULL) return;
    int label = signDetect->currentSign->signLabel;
    Rect rect = signDetect->currentSign->boundingBox;
    if (signDetect->signDir == -1) putText(image, "LEFT", Point(rect.x - 3, rect.y - 10), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 0, 0));
    else if (signDetect->signDir == 1) putText(image, "RIGHT", Point(rect.x - 3, rect.y - 10), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 0, 0));
    rectangle(image, rect, Scalar(0, 0, 255), 1);
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        Mat img = detect->update(cv_ptr->image, signDetect->signDir);

        signDetect->update(cv_ptr->image);

        drawSign(img);

        cv::imshow("View", img);
        waitKey(1);

        float error = detect->getErrorAngle(signDetect->signDir);

        car->driverCar(error, signDetect->signSignal);
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

    signDetect = new DetectSign();
    detect = new DetectLane();
    car = new CarControl();

    cv::startWindowThread();

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("Team1_image", 1, imageCallback);

    ros::spin();

    cv::destroyAllWindows();
}
