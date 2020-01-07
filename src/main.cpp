#include "core.h"
#include "abstract.h"

#include "lane_detection.h"
#include "car_controller.h"

tpv::LaneDetectorObject *detector;
tpv::CarControllerObject *controller;

// bool RUN = false;
bool RUN = true;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    TPV_CV_MAT mem_img;
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        detector->update(cv_ptr->image, mem_img);

        cv::imshow("View", mem_img);
        cv::waitKey(1);

        float error = detector->get_err_angle();
        if (RUN) {
            controller->drive(-error);
        }
        else {
            controller->stop();
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void on_Button1_Pressed(const std_msgs::Bool::ConstPtr &msg)
{
    bool data = msg->data;
    if (data)
    {
        RUN = true;
    }
}

void on_Button4_Pressed(const std_msgs::Bool::ConstPtr &msg)
{
    bool data = msg->data;
    if (data)
    {
        RUN = false;
    }
}

void on_Sensor_Triggered(const std_msgs::Bool::ConstPtr &msg)
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
    // cv::namedWindow("Debug - Binary");
    cv::namedWindow("Debug - Line");

    detector = new tpv::LaneDetectorObject();
    controller = new tpv::CarControllerObject();

    // cv::startWindowThread();

    detector->create_track_bars(); // only for dev

    ros::NodeHandle nh1, nh2;
    image_transport::ImageTransport it(nh1);
    // image_transport::Subscriber sub1 = it.subscribe("camera/rgb/image_raw", 1, imageCallback);
    image_transport::Subscriber sub1 = it.subscribe("team1/camera/rgb", 1, imageCallback);

    ros::Subscriber sub2 = nh2.subscribe("/bt1_status", 10, on_Button1_Pressed);
    ros::Subscriber sub3 = nh2.subscribe("/bt4_status", 10, on_Button4_Pressed);
    ros::Subscriber sub4 = nh2.subscribe("/ss_status",  10, on_Sensor_Triggered);

    //draft
    // ros::NodeHandle nh;
    // image_transport::ImageTransport it(nh);
    // image_transport::Subscriber sub = it.subscribe("team1/camera/rgb", 1, imageCallback);
    //end draft

    ros::spin();

    cv::destroyAllWindows();
}
