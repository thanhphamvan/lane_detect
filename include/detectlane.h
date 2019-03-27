#ifndef DETECTLANE_H
#define DETECTLANE_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace std;
using namespace cv;

class DetectLane
{
public:
    DetectLane();
    ~DetectLane();

    Mat update(const Mat &src);
    float getErrorAngle();
    
    static int WIDTH;
    static int HEIGHT;

private:

    Mat preProcess(const Mat &src);
    Mat binaryImage(const Mat &src);

    Point getPointInLine(Vec4i line, float y);
    vector<Vec4i> fitLane2Line(const Mat &src, float weight = 0);
    void groupLine(const vector<Vec4i> &lines);

    float errorAngle(const Point &dst);

    int minThreshold[3] = {0, 0, 170};
    int maxThreshold[3] = {179, 30, 255};

    int binaryThreshold = 180;

    int skyLine = 90;

    Vec4i leftLane;
    Vec4i rightLane;

    Vec4i preLane;

    static Vec4i nullLine;

    Point carPos;

    int laneWidth = 160;
};

#endif
