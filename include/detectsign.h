#ifndef DETECTSIGN_H
#define DETECTSIGN_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace std;
using namespace cv;

struct SignInfo {
    int signLabel;
    Rect boundingBox;
};

class DetectSign
{
public:
    DetectSign();
    void update(const Mat &image);
    SignInfo *currentSign;
    int signSignal = 0;
    int signDir = 0;
private:
    vector<SignInfo> signs;
    int countNoneSign = 0;
    
    Mat colorThreshold(const Mat &image);
    Mat removeNoise(const Mat &blob);
    Mat signExtract(const Mat &blob, SignInfo &sign);

    int huMomentsClassify(const Mat &blob);

};

#endif