#include "detectlane.h"

int min(int a, int b)
{
    return a < b ? a : b;
}

int DetectLane::WIDTH = 320;
int DetectLane::HEIGHT = 240;

Point2f DetectLane::nullPoint = Point2f();

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

    vector<Point2f> lines = fitLane2Line(binary, 10);

    // groupLine(lines);
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

    imshow("Binary", binary);

    return binary;
}

struct Dist {
    bool operator()(const Point& a, const Point &b) {
        return abs(a.y - b.y) < 20;
    }
};

void swap(int *a, int *b)
{
    int temp = *a;
    *a = *b;
    *b = temp;
}

Vec2f DetectLane::point2Line(Point2f pt)
{
    float x = pt.x;
	float alpha = pt.y;

	float slope = tan(alpha * CV_PI / 180);
	float intercept = HEIGHT - slope * x;
    
	return Vec2f(slope, intercept);
}

void DetectLane::groupLine(const vector<Point2f> &lines)
{
    if (lines.size() == 0) return;

    vector<int> labels;
    int cnt = partition(lines, labels, Dist());

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
    Point2f mean[cnt];

    for (int i = 0; i < cnt; i++)
    {
        countLine[i] = 0;
        idx[i] = i;
    }

    for (int i = 0; i < labels.size(); i++)
    {
        countLine[labels[i]]++;
        mean[i] = Point2f(0, 0);
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

    leftLane = nullPoint;
    centerLane = nullPoint;
    rightLane = nullPoint;
    
    if (currentLaneNum == 2)
    {
        if (cnt >= 2)
        {
            if (mean[0].x < mean[1].x)
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
            if (mean[0].x < WIDTH / 2) leftLane = mean[0];
            else rightLane = mean[0];
        }
    }
    else
    {
        if (cnt >= 3)
        {

        }
    }
}

vector<Point2f> DetectLane::fitLane2Line(const Mat &src, float weight)
{
    vector<std::vector<Point>> cnts;
    vector<Vec4i> hierarchy;
    vector<Point2f> res;
    Mat debug = Mat::zeros(src.size(), CV_8UC1);

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

            Vec4f line1;
            fitLine(cnts[i], line1, 2, 0, 0.01, 0.01);
            float angle = atan(line1[0] / line1[1]) * 180 / CV_PI;
            float point = (HEIGHT - line1[3]) * line1[0] / line1[1] + line1[2];

            Vec2f v = point2Line(Point2f(point, angle));
            Point pt1 = Point((120 - v[1]) / v[0], 120);
            Point pt2 = Point((240 - v[1]) / v[0], 240);

            line(debug, pt1, pt2, Scalar(255), 3, CV_AA);

            if (weight)
            {
                for (int w = 0; w < ceil(length / weight); w++)
                {
                    res.push_back(Point2f(point, angle));    
                }
            }
            else
            {
                res.push_back(Point2f(point, angle));    
            }
        }
    }

    imshow("Debug", debug);

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

    Mat lane = Mat::zeros(img.size(), CV_8UC1);

    cannyImg.copyTo(lane, imgThresholded);

    return lane;
}