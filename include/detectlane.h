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

    Mat update(const Mat &src, int signDir);
    float getErrorAngle(int signDir);
    
    static int WIDTH;
    static int HEIGHT;

private:

    Mat preProcess(const Mat &src);
    Mat binaryImage(const Mat &src);

    Point getPointInLine(Vec4i line, float y);
    vector<Vec4i> fitLane2Line(const Mat &src, float weight = 0, int dir = 0);
    void groupLine(const vector<Vec4i> &lines, int signDir);

    float errorAngle(const Point &dst);

    int minThreshold[3] = {0, 0, 170};
    int maxThreshold[3] = {179, 30, 255};

    int minShadowTh[3] = {90, 43, 36};
    int maxShadowTh[3] = {120, 81, 171};

    int minLane[3] = {0, 0, 30};
    int maxLane[3] = {100, 60, 120}; 

    int binaryThreshold = 180;

    int skyLine = 50;

    Vec4i centerLane;
    Vec4i leftLane;
    Vec4i rightLane;

    Vec4i preLane;

    static Vec4i nullLine;

    Point carPos;

    int laneWidth = 100;
};

#endif
