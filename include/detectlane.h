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

    void update(const Mat &src);
    float getErrorAngle();
    
    static int WIDTH;
    static int HEIGHT;

    float preError;

private:

    Mat preProcess(const Mat &src);
    Mat binaryImage(const Mat &src);

    Point getPointInLine(Vec4f line, float y);
    vector<Vec4f> fitLane2Line(const Mat &src, float weight = 0);
    void groupLine(const vector<Vec4f> &lines);

    bool checkCrossRoad(const Mat &img, int dir);
    float errorAngle(const Point &dst);

    int minThreshold[3] = {0, 0, 180};
    int maxThreshold[3] = {179, 30, 255};

    int minShadowTh[3] = {90, 43, 36};
    int maxShadowTh[3] = {120, 81, 171};

    int binaryThreshold = 180;

    int skyLine = 85;

    Vec4f centerLane;
    Vec4f leftLane;
    Vec4f rightLane;

    static Vec4f nullLine;

    Point carPos;

    int laneWidth = 80;
};

#endif
