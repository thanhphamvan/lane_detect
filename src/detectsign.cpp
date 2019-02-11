#include "detectsign.h"

double max(double a, double b)
{
    return a > b ? a : b;
}

DetectSign::DetectSign()
{
}

void DetectSign::update(const Mat &image)
{
    if (signSignal != 0) signSignal--;
    SignInfo sign;

    Mat blob = colorThreshold(image);
    Mat imgRemoveNoise = removeNoise(blob);
    Mat signImage = signExtract(imgRemoveNoise, sign);
    int signLabel = huMomentsClassify(signImage);
    if (signLabel != -1)
    {
        sign.signLabel = signLabel;
        currentSign = &sign;
        signSignal = 3;
        if (signLabel > 0)
        {
            signs.push_back(sign);

            int dir = 0;
            for (int i = 0; i < signs.size(); i++)
            {
                if (signs[i].signLabel == 1) dir += 1;
                else if (signs[i].signLabel == 2) dir += -1;
            }
            if (dir > 0)
            {
                signDir = 1;
                signSignal = 90;
            }
            else
            {
                signDir = -1;
                signSignal = 90;
            }
        }

    }
    else
    {
        currentSign = NULL;
        countNoneSign++;
        if (countNoneSign >= 3)
        {
            signDir = 0;
            signs.clear();
            countNoneSign = 0;
        }
    }
}

Mat DetectSign::colorThreshold(const Mat &image)
{
    Mat imgHSV, imgThresholded;
    cvtColor(image, imgHSV, CV_BGR2HSV);
    inRange(imgHSV, Scalar(100, 150, 0), Scalar(140, 255, 255), imgThresholded);
    return imgThresholded;
}

Mat DetectSign::removeNoise(const Mat &blob)
{
    Mat labels, stats, centroids, imgBlob;
    Mat imgRemoveNoise = Mat::zeros(blob.size(), CV_8UC1);
    vector<int> regions;

    int numLabels = connectedComponentsWithStats(blob, labels, stats, centroids, 4);

    for (int i = 1; i < numLabels; i++)
    {
        if (stats.at<int>(i, 4) >= 10)
        {
            regions.push_back(i);
        }
    }

    for (int r = 0; r < imgRemoveNoise.rows; r++)
    {
        for (int c = 0; c < imgRemoveNoise.cols; c++)
        {
            for (int i = 0; i < regions.size(); i++)
            {
                if (labels.at<int>(r, c) == regions[i])
                {
                    imgRemoveNoise.at<uchar>(r, c) = 255;
                }
            }
        }
    }

    return imgRemoveNoise;
}

Mat DetectSign::signExtract(const Mat &blob, SignInfo &sign)
{
    vector<vector<Point>> contours;
    Mat imgBlob = blob.clone();
    morphologyEx(imgBlob, imgBlob, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    findContours(imgBlob, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    Mat res = Mat::zeros(blob.size(), CV_8UC1);

    if (contours.size() != 0)
    {
        Rect crop;
        int largestArea = 0;
        int largestAreaID = 0;
        for (int i = 0; i < contours.size(); i++)
        {
            if (contourArea(contours[i]) > largestArea)
            {
                largestArea = contourArea(contours[i]);
                largestAreaID = i;
            }
        }

        if (largestArea >= 10)
        {
            Mat signMask = Mat::zeros(blob.size(), CV_8UC1);
            Mat arrowMask = Mat::zeros(blob.size(), CV_8UC1);
            imgBlob = blob.clone();

            vector<vector<Point>> hull(1);
            convexHull(contours[largestAreaID], hull[0]);
            drawContours(signMask, hull, 0, Scalar(255), CV_FILLED);
            drawContours(signMask, hull, 0, Scalar(255), 2);
            drawContours(imgBlob, hull, 0, Scalar(255), 2);

            crop = boundingRect(contours[largestAreaID]);
            signMask = signMask(crop);
            arrowMask = imgBlob(crop);
            bitwise_xor(signMask, arrowMask, res);

            sign.boundingBox = crop;
        }
    }

    return res;
}

int DetectSign::huMomentsClassify(const Mat &blob)
{
    static double rightSign[7] = {1.47571148e-03,
                                  8.04467035e-07,
                                  1.06030043e-09,
                                  4.90356123e-10,
                                  3.50997732e-19,
                                  2.93231799e-13,
                                  4.26152460e-20};
    Moments mm = moments(blob, false);

    double huMoments[7], error = 0;
    HuMoments(mm, huMoments);

    for (int i = 0; i < 6; i++)
        error += abs(rightSign[i] - huMoments[i]);

    if (error < 0.0001)
    {
        return huMoments[6] > 0 ? 1 : 2;
    }
    else if (error < 0.001)
    {
        return 0;
    }

    return -1;
}