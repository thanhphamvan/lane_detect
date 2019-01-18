#include "detectlane.h"

struct Dist {
    bool operator()(float a, float b) {
        return abs(a - b) < 15;
    }
};

void swap(int *a, int *b)
{
    int temp = *a;
    *a = *b;
    *b = temp;
}

int DetectLane::WIDTH = 320;
int DetectLane::HEIGHT = 240;

Vec4f DetectLane::nullLine = Vec4f();

DetectLane::DetectLane()
{

    carPos.x = 120;
    carPos.y = 500;

    cvCreateTrackbar("LowH", "Threshold", &minThreshold[0], 179);
    cvCreateTrackbar("HighH", "Threshold", &maxThreshold[0], 179);

    cvCreateTrackbar("LowS", "Threshold", &minThreshold[1], 255);
    cvCreateTrackbar("HighS", "Threshold", &maxThreshold[1], 255);

    cvCreateTrackbar("LowV", "Threshold", &minThreshold[2], 255);
    cvCreateTrackbar("HighV", "Threshold", &maxThreshold[2], 255);
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

void DetectLane::update(const Mat &src)
{
    Mat binary = preProcess(src);

    vector<Vec4f> lines = fitLane2Line(binary, 10);

    groupLine(lines);

    Mat lane = Mat::zeros(binary.size(), CV_8UC3);

    if (leftLane != nullLine)
    {
        Point pt1 = getPointInLine(leftLane, HEIGHT);
        Point pt2 = getPointInLine(leftLane, HEIGHT / 2);
        line(lane, pt1, pt2, Scalar(0, 0, 255), 2);
    }

    if (rightLane != nullLine)
    {
        Point pt1 = getPointInLine(rightLane, HEIGHT);
        Point pt2 = getPointInLine(rightLane, HEIGHT / 2);
        line(lane, pt1, pt2, Scalar(0, 255, 0), 2);
    }

    if (centerLane != nullLine)
    {
        Point pt1 = getPointInLine(centerLane, HEIGHT);
        Point pt2 = getPointInLine(centerLane, HEIGHT / 2);
        line(lane, pt1, pt2, Scalar(255, 0, 0), 2);
    }

    imshow("Lane", lane);
}

float DetectLane::getErrorAngle()
{
    return 0;
}

bool DetectLane::checkCrossRoad(const Mat &src, int dir)
{
    Mat labelImage, stats, centroids;

    int nLabels = connectedComponentsWithStats(src, labelImage, stats, centroids, 8, CV_32S);
    int maxArea = 0, idx = 0;

    for (int label = 1; label < nLabels; label++)
    {
        if (stats.at<int>(label, CC_STAT_AREA) > maxArea)
        {
            maxArea = stats.at<int>(label, CC_STAT_AREA);
            idx = label;
        }
    }

    compare(labelImage, idx, src, CMP_EQ);

    return true;
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
    if (lines.size() == 0) return;

    vector<int> labels;
    vector<float> angles;

    for (int i = 0; i < lines.size(); i++)
    {
        angles.push_back(atan2(lines[i][0], lines[i][1]) * 180 / CV_PI);
    }

    int cnt = partition(angles, labels, Dist());

    // ROS_INFO("%d", cnt);

    static int countFrame = 0;
    static vector<int> laneNumPerSecond;
    static int currentLaneNum = 2;

    countFrame++;

    if (countFrame >= 30)
    {
        int count = 0;
        for (int i = 0; i < countFrame; i++) count += laneNumPerSecond[i] == 3 ? 1 : -1;

        if (count > 0) currentLaneNum = 3;
        else currentLaneNum = 2;

        countFrame = 0;
        laneNumPerSecond.clear();
    }
    else laneNumPerSecond.push_back(cnt <= 2 ? 2 : 3);

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
        mean[labels[i]] = Vec4f();
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
            if (getPointInLine(mean[0], HEIGHT).x < WIDTH / 2) leftLane = mean[0];
            else rightLane = mean[0];
        }
    }
    else
    {
        if (cnt >= 3)
        {
            int line1 = getPointInLine(mean[0], HEIGHT).x;
            int line2 = getPointInLine(mean[1], HEIGHT).x;
            int line3 = getPointInLine(mean[2], HEIGHT).x;

            int arr[3] = {line1, line2, line3};
            int idx[3] = {1, 2, 3};
            for (int i = 0; i < 2; i++)
            {
                for (int j = i + 1; j < 3; j++)
                {
                    if (arr[i] > arr[j])
                    {
                        swap(arr[i], arr[j]);
                        swap(idx[i], idx[j]);
                    }
                }
            }

            for (int i = 0; i < 3; i++)
            {
                if (idx[i] == 0) leftLane = mean[i];
                else if (idx[i] == 1) centerLane = mean[i];
                else rightLane = mean[i];
            }
        }
    }
}

vector<Vec4f> DetectLane::fitLane2Line(const Mat &src, float weight)
{
    vector<std::vector<Point>> cnts;
    vector<Vec4i> hierarchy;
    vector<Vec4f> res;
    Mat debug = Mat::zeros(src.size(), CV_8UC1);

    vector<Vec4i> lines;
    HoughLinesP(src, lines, 1, CV_PI/180, 35, 20, 5);

    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line(debug, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 1, CV_AA);
    }
    imshow("HoughLine", debug);

    findContours(src, cnts, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    int cntsN = cnts.size();

    for (int i = 0; i < cntsN; i++)
    {
        Rect rect = boundingRect(cnts[i]);
        float length = sqrt(rect.width * rect.width + rect.height * rect.height);
        if (length > 15)
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
            if (!check)
                continue;

            Vec4f l;
            fitLine(cnts[i], l, 2, 0, 0.01, 0.01);
            float angle = atan(l[0]/l[1]) * 180 / CV_PI;

            if (weight)
            {
                for (int w = 0; w < ceil(length / weight); w++)
                {
                    res.push_back(l);    
                }
            }
            else
            {
                res.push_back(l);    
            }
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