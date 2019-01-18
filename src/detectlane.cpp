#include "detectlane.h"

struct Dist
{
    bool operator()(float a, float b)
    {
        return abs(a - b) < 15;
    }
};

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

int DetectLane::WIDTH = 320;
int DetectLane::HEIGHT = 240;

Vec4f DetectLane::nullLine = Vec4f();

DetectLane::DetectLane()
{

    carPos.x = WIDTH / 2;
    carPos.y = HEIGHT;

    preLane = nullLine;

    cvCreateTrackbar("LowH", "Threshold", &minLane[0], 179);
    cvCreateTrackbar("HighH", "Threshold", &maxLane[0], 179);

    cvCreateTrackbar("LowS", "Threshold", &minLane[1], 255);
    cvCreateTrackbar("HighS", "Threshold", &maxLane[1], 255);

    cvCreateTrackbar("LowV", "Threshold", &minLane[2], 255);
    cvCreateTrackbar("HighV", "Threshold", &maxLane[2], 255);
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

Mat DetectLane::update(const Mat &src, int signDir)
{
    Mat output = src.clone();

    detectCrossRoad(src, signDir);

    Mat binary = preProcess(src);

    vector<Vec4f> lines = fitLane2Line(binary, 10, signDir);

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

float DetectLane::getErrorAngle(int signDir)
{
    Point dst(WIDTH / 2, HEIGHT / 2);
    int p1 = getPointInLine(leftLane, HEIGHT / 2).x;
    int p2 = getPointInLine(rightLane, HEIGHT / 2).x;
    int pc = getPointInLine(centerLane, HEIGHT / 2).x;
    int pr = WIDTH / 2;
    
    if (preLane != nullLine) getPointInLine(preLane, HEIGHT / 2).x;


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
        if (p1 > p2)
        {
            if (signDir > 0)
            {
                dst.x = p1 + 50;
                ROS_INFO("ok");
            }
            else if (signDir < 0)
            {
                dst.x = p2 - 50;
                ROS_INFO("fail");
            }
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
            dst.x = p2 - 50;
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
            dst.x = p1 + 50;
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

Point DetectLane::getPointInLine(Vec4f line, float y)
{
    return Point((y - line[3]) * line[0] / line[1] + line[2], y);
}

void DetectLane::groupLine(const vector<Vec4f> &lines)
{
    if (lines.size() == 0)
        return;

    vector<int> labels;
    vector<float> angles;

    for (int i = 0; i < lines.size(); i++)
    {
        angles.push_back(atan2(lines[i][1], lines[i][0]) * 180 / CV_PI);
    }

    int cnt = partition(angles, labels, Dist());

    static int countFrame = 0;
    static vector<int> laneNumPerSecond;
    static int currentLaneNum = 2;

    countFrame++;

    if (countFrame >= 30)
    {
        int count = 0;
        for (int i = 0; i < countFrame; i++)
            count += laneNumPerSecond[i] == 3 ? 1 : -1;

        if (count > 0)
            currentLaneNum = 3;
        else
            currentLaneNum = 2;

        countFrame = 0;
        laneNumPerSecond.clear();
    }
    else
        laneNumPerSecond.push_back(cnt <= 2 ? 2 : 3);

    int countLine[cnt];
    int idx[cnt];
    Vec4f mean[cnt];

    for (int i = 0; i < cnt; i++)
    {
        countLine[i] = 0;
        idx[i] = i;
    }

    for (int i = 0; i < labels.size(); i++)
    {
        countLine[labels[i]]++;
        mean[labels[i]] = Vec4f(0, 0, 0, 0);
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

    if (currentLaneNum == 2)
    {
        if (cnt >= 2)
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
        else
        {
            if (getPointInLine(mean[0], HEIGHT).x < WIDTH / 2)
                leftLane = mean[0];
            else
                rightLane = mean[0];
        }
    }
    else
    {
        if (cnt >= 3)
        {
            for (int i = 0; i < 2; i++)
            {
                for (int j = i + 1; j < 3; j++)
                {
                    if (getPointInLine(mean[i], HEIGHT).x > getPointInLine(mean[j], HEIGHT).x)
                    {
                        Vec4f temp = mean[i];
                        mean[i] = mean[j];
                        mean[j] = temp;
                    }
                }
            }
            leftLane = mean[0];
            centerLane = mean[1];
            rightLane = mean[2];
        }
        else if (cnt == 2)
        {
            float p1 = getPointInLine(mean[0], HEIGHT).x;
            float p2 = getPointInLine(mean[1], HEIGHT).x;
            float pc = getPointInLine(preLane, HEIGHT).x;
            
            if (abs((p1 + p2) / 2 - pc) > 20)
            {
                if (p1 < p2)
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
            else
            {
                if (abs(p1 - pc) < abs(p2 - pc))
                {
                    centerLane = mean[0];
                    if (p2 > p1) rightLane = mean[1];
                    else leftLane = mean[1];
                }
                else
                {
                    centerLane = mean[1];
                    if (p2 > p1) rightLane = mean[0];
                    else leftLane = mean[0];
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
}

vector<Vec4f> DetectLane::detectCrossRoad(const Mat &src, int dir)
{
    vector<Vec4f> res;
    Mat imgHSV, imgThresholded;
    cvtColor(src, imgHSV, COLOR_BGR2HSV);

    inRange(imgHSV, Scalar(minLane[0], minLane[1], minLane[2]),
            Scalar(maxLane[0], maxLane[1], maxLane[2]),
            imgThresholded);


    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)));
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)));

    vector<vector<Point> > cnts;
    findContours(imgThresholded, cnts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    int maxArea = 0, idx = 0;

    for (int i = 0; i < cnts.size(); i++)
    {
        bool check = true;
        for (int j = 0; j < cnts[i].size(); j++)
        {
            if (cnts[i][j].y < skyLine)
            {
                check = false;
                break;
            }
        }

        if (!check) continue;

        if (contourArea(cnts[i]) > maxArea)
        {
            maxArea = contourArea(cnts[i]);
            idx = i;
        }
    }

    Mat lane = Mat::zeros(src.size(), CV_8UC1);

    drawContours(lane, cnts, idx, Scalar(255), CV_FILLED);

    imshow("Threshold", lane);

    return res;
}

vector<Vec4f> DetectLane::fitLane2Line(const Mat &src, float weight, int dir)
{
    vector<Vec4f> res;
    Mat debug = Mat::zeros(src.size(), CV_8UC1);

    vector<Vec4i> lines;
    HoughLinesP(src, lines, 1, CV_PI / 180, 35, 20, 5);

    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        line(debug, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 1, CV_AA);
    }

    imshow("HoughLine", debug);

    for (int i = 0; i < lines.size(); i++)
    {
        float length = lineLength(lines[i]);
        float angle = lineAngle(lines[i]);

        if (abs(angle) < 15) continue;

        if (dir != 0)
        {
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
        }

        vector<Point> points;
        points.push_back(Point(lines[i][0], lines[i][1]));
        points.push_back(Point(lines[i][2], lines[i][3]));

        Vec4f l;
        fitLine(points, l, 2, 0, 0.01, 0.01);

        if (weight)
        {
            float priority = 1.0;
            if (dir == 1)
            {
                if ((angle > 45 || (angle < 0 && angle > -45)) && lines[i][0] > WIDTH / 2 && lines[i][2] > WIDTH / 2)
                {
                    priority = 10.0f;
                }
            }
            for (int w = 0; w < ceil(length / weight) * priority; w++)
            {
                res.push_back(l);
            }
        }
        else
        {
            res.push_back(l);
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

    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

    Canny(gray, cannyImg, 50, 150);

    imshow("Canny", cannyImg);

    Mat lane = Mat::zeros(img.size(), CV_8UC1);

    cannyImg.copyTo(lane, imgThresholded);

    return lane;
}