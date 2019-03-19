#include "detectlane.h"

void swap(int *a, int *b)
{
    int temp = *a;
    *a = *b;
    *b = temp;
}

float lineLength(Vec4i line)
{
    return sqrt((line[0] - line[2]) * (line[0] - line[2]) + (line[1] - line[3]) * (line[1] - line[3]));
}

float lineAngle(Vec4i line)
{
    float X = line[2] - line[0];
    float Y = line[3] - line[1];
    return atan2(Y, X) * 180 / CV_PI;
}

struct Dist
{
    bool operator()(Vec4i a, Vec4i b)
    {
        return abs(lineAngle(a) - lineAngle(b)) < 15;
    }
};

int DetectLane::WIDTH = 320;
int DetectLane::HEIGHT = 240;

Vec4i DetectLane::nullLine = Vec4i();

DetectLane::DetectLane()
{
    carPos.x = WIDTH / 2;
    carPos.y = HEIGHT;

    preLane = nullLine;
}

float DetectLane::errorAngle(const Point &dst)
{
    if (dst.x == carPos.x)
        return 0;
    if (dst.y == carPos.y)
        return (dst.x < carPos.x ? -90 : 90);

    double dx = dst.x - carPos.x;
    double dy = carPos.y - dst.y;

    if (dx < 0)
        return -atan(-dx / dy) * 180 / CV_PI;
    return atan(dx / dy) * 180 / CV_PI;
}

DetectLane::~DetectLane() {}

Mat DetectLane::update(const Mat &src)
{
    Mat output = src.clone();

    Mat binary = preProcess(src);

    vector<Vec4i> lines = fitLane2Line(binary, 10);

    groupLine(lines);

    if (leftLane != nullLine)
    {
        Point pt1 = getPointInLine(leftLane, HEIGHT);
        Point pt2 = getPointInLine(leftLane, HEIGHT / 2);
        line(output, pt1, pt2, Scalar(0, 0, 255), 2);
    }

    if (rightLane != nullLine)
    {
        Point pt1 = getPointInLine(rightLane, HEIGHT);
        Point pt2 = getPointInLine(rightLane, HEIGHT / 2);
        line(output, pt1, pt2, Scalar(0, 255, 0), 2);
    }

    if (centerLane != nullLine)
    {
        Point pt1 = getPointInLine(centerLane, HEIGHT);
        Point pt2 = getPointInLine(centerLane, HEIGHT / 2);
        line(output, pt1, pt2, Scalar(255, 0, 0), 2);
    }
    
    return output;
}

float DetectLane::getErrorAngle()
{
    Point dst(WIDTH / 2, HEIGHT / 2);
    int p1 = getPointInLine(leftLane, HEIGHT / 2).x;
    int p2 = getPointInLine(rightLane, HEIGHT / 2).x;
    int pc = getPointInLine(centerLane, HEIGHT / 2).x;
    int pr = WIDTH / 2;
    
    if (preLane != nullLine) pr = getPointInLine(preLane, HEIGHT / 2).x;

    if (leftLane != nullLine && rightLane != nullLine && centerLane != nullLine)
    {
        if (abs((p1 + p2) / 2 - pc) < 10)
        {
            dst.x = (p1 + p2) / 2;
        }
        else
        {
            dst.x = ((p1 + p2) / 2 + pr) / 2;
        }
    }
    else if (leftLane != nullLine && rightLane != nullLine)
    {
        if (abs((p1 + p2) / 2 - pr) < 20)
        {
            dst.x = (p1 + p2) / 2;
        }
        else
        {
            dst.x = ((p1 + p2) / 2 + pr) / 2;
        }
    }
    else if (rightLane != nullLine)
    {
        if (centerLane != nullLine && abs(pc - pr) < 20)
        {
            dst.x = pc;
        }
        else
        {
            dst.x = p2 - laneWidth / 2;
        }
    }
    else if (leftLane != nullLine)
    {
        if (centerLane != nullLine && abs(pc - pr) < 20)
        {
            dst.x = pc;
        }
        else
        {
            dst.x = p1 + laneWidth / 2;
        }
    }

    return errorAngle(dst);
}

Mat DetectLane::preProcess(const Mat &src)
{
    Mat binary;

    binary = binaryImage(src);

    return binary;
}

Point DetectLane::getPointInLine(Vec4i line, float y)
{
    return Point((y - line[1]) * (line[0] - line[2]) / (line[1] - line[3]) + line[0], y);
}

void DetectLane::groupLine(const vector<Vec4i> &lines)
{
    if (lines.size() == 0)
        return;

    vector<int> labels;

    int cnt = partition(lines, labels, Dist());

    int countLine[cnt];
    int idx[cnt];
    Vec4i mean[cnt];

    for (int i = 0; i < cnt; i++)
    {
        countLine[i] = 0;
        idx[i] = i;
    }

    for (int i = 0; i < labels.size(); i++)
    {
        countLine[labels[i]]++;
        mean[labels[i]] = Vec4i(0, 0, 0, 0);
    }

    for (int i = 0; i < cnt - 1; i++)
    {
        for (int j = i + 1; j < cnt; j++)
        {
            if (countLine[i] < countLine[j])
            {
                swap(countLine[i], countLine[j]);
                swap(idx[i], idx[j]);
            }
        }
    }

    for (int i = 0; i < lines.size(); i++)
    {
        mean[idx[labels[i]]] += lines[i];
    }

    for (int i = 0; i < cnt; i++)
    {
        mean[idx[i]] /= countLine[idx[i]];
    }

    leftLane = nullLine;
    centerLane = nullLine;
    rightLane = nullLine;

    if (cnt >= 2)
    {
        if (abs(lineAngle(mean[0])) > 5 && abs(lineAngle(mean[1])) > 5)
        {
            if (getPointInLine(mean[0], HEIGHT).x < getPointInLine(mean[1], HEIGHT).x)
            {
                leftLane = mean[0];
                rightLane = mean[1];
            }
            else
            {
                leftLane = mean[1];
                rightLane = mean[0];
            }
        }
    }
    else
    {
        if (getPointInLine(mean[0], HEIGHT).x < WIDTH / 2)
            leftLane = mean[0];
        else
            rightLane = mean[0];
    }        
}

vector<Vec4i> DetectLane::fitLane2Line(const Mat &src, float weight)
{
    vector<Vec4i> res;
    Mat debug = Mat::zeros(src.size(), CV_8UC1);

    vector<Vec4i> lines;
    HoughLinesP(src, lines, 1, CV_PI / 180, 35, 10, 3);

    for (int i = 0; i < lines.size(); i++)
    {
        float length = lineLength(lines[i]);
        float angle = lineAngle(lines[i]);

        if (abs(angle) < 5) continue;

        bool check = true;
        for (int j = 0; j < 2; j++)
        {
            if (lines[i][j * 2 + 1] < skyLine)
            {
                check = false;
                break;
            }
        }
        if (!check)
            continue;

        if (weight)
        {
            for (int w = 0; w < ceil(length / weight); w++)
            {
                res.push_back(lines[i]);
            }
        }
        else
        {
            res.push_back(lines[i]);
        }
    }

    return res;
}

Mat DetectLane::binaryImage(const Mat &src)
{
    Mat img, imgThresholded, imgShadow, imgHSV, cannyImg, gray, dst;

    img = src.clone();

    medianBlur(img, img, 3);

    cvtColor(img, imgHSV, COLOR_BGR2HSV);
    cvtColor(img, gray, COLOR_BGR2GRAY);

    inRange(imgHSV, Scalar(minThreshold[0], minThreshold[1], minThreshold[2]),
            Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]),
            imgThresholded);

    // if (leftLane != nullLine)
    // {
    //     Point pt1 = getPointInLine(leftLane, HEIGHT);
    //     Point pt2 = getPointInLine(leftLane, 100);
    //     line(imgThresholded, pt1, pt2, Scalar(255), 30);
    // }

    // if (rightLane != nullLine)
    // {
    //     Point pt1 = getPointInLine(rightLane, HEIGHT);
    //     Point pt2 = getPointInLine(rightLane, 100);
    //     line(imgThresholded, pt1, pt2, Scalar(255), 30);
    // }

    // if (centerLane != nullLine)
    // {
    //     Point pt1 = getPointInLine(centerLane, HEIGHT);
    //     Point pt2 = getPointInLine(centerLane, 100);
    //     line(imgThresholded, pt1, pt2, Scalar(255), 30);
    // }

    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

    Canny(gray, cannyImg, 30, 150);

    Mat lane = Mat::zeros(img.size(), CV_8UC1);

    cannyImg.copyTo(lane, imgThresholded);

    return lane;
}