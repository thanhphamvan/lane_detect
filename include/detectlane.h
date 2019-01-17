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
    vector<Point2f> fitLane2Line(const Mat &src, float weight = 0);
    void groupLine(const vector<Point2f> &lines);

    bool checkCrossRoad(const Mat &img, int dir);
    float errorAngle(const Point &dst);

    Vec2f point2Line(Point2f pt);

    int minThreshold[3] = {0, 0, 180};
    int maxThreshold[3] = {179, 30, 255};

    int minShadowTh[3] = {90, 43, 36};
    int maxShadowTh[3] = {120, 81, 171};

    int binaryThreshold = 180;

    int skyLine = 85;

    Point2f centerLane;
    Point2f leftLane;
    Point2f rightLane;

    static Point2f nullPoint;

    Point carPos;

    int laneWidth = 80;
};

#endif
