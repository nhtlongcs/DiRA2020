#include "detectsign.h"
#include <opencv2/opencv.hpp>
#include <vector>
using namespace cv;
using namespace std;

void DetectSign::updateRGB(const cv::Mat& rgb)
{
    this->rgb = rgb.clone();
}

int DetectSign::detect() {
    if (rgb.empty())
    {
        return 0;
    }

    Mat blue;
    int turnFactor = 0;

    cvtColor(rgb, blue, cv::COLOR_BGR2HSV);
    inRange(blue, Scalar(minBlue[0], minBlue[1], minBlue[2]), Scalar(maxBlue[0], maxBlue[1], maxBlue[2]), blue);
    imshow("blue", blue);

    Mat canvas = Mat::zeros(rgb.size(), CV_8UC3);
    vector<vector<Point>> contours;
    findContours(blue, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
        
        vector<vector<Point>> contoursPoly((int)contours.size());
        vector<Rect> boundRect((int)contours.size());

        for (int i = 0; i < (int)contours.size(); ++i) {
            approxPolyDP(contours[i], contoursPoly[i], 3, true);
            boundRect[i] = boundingRect(contoursPoly[i]);    
        }

        int maxArea = 0, bestRect;
        for (int i = 0; i < (int)boundRect.size(); ++i) {
            int boundRect_W = abs(boundRect[i].tl().x - boundRect[i].br().x);
            int boundRect_H = abs(boundRect[i].tl().y - boundRect[i].br().y);
            if (boundRect_W * boundRect_H > maxArea) {
                maxArea = boundRect_W * boundRect_H;
                bestRect = i;
            }
        }

        if (maxArea < 125) goto skip;

        Point topLeft = boundRect[bestRect].tl();
        Point bottomRight = boundRect[bestRect].br();
        rectangle(canvas, topLeft, bottomRight, Scalar(0, 0, 255), 2, 8, 0);

        int rectW = abs(topLeft.x - bottomRight.x) * 4;
        int rectH = abs(topLeft.y - bottomRight.y) * 4;
        Mat zoom(rectW, rectH, CV_8UC1);
        Point2f boundingBox[4], zoomBox[4];
        
        boundingBox[0] = topLeft;
        boundingBox[1] = Point(bottomRight.x, topLeft.y);
        boundingBox[2] = bottomRight;
        boundingBox[3] = Point(topLeft.x, bottomRight.y);

        zoomBox[0] = Point(0, 0);
        zoomBox[1] = Point(rectW, 0);
        zoomBox[2] = Point(rectW, rectH);
        zoomBox[3] = Point(0, rectH);

        Mat M = getPerspectiveTransform(boundingBox, zoomBox);
        warpPerspective(blue, zoom, M, zoom.size(), INTER_LINEAR, BORDER_CONSTANT);

        int cntLeftOnes = 0, cntRightOnes = 0;
        for (int i = 0; i < rectW; ++i)
            for (int j = 0; j < rectH; ++j) 
                if ((int)zoom.at<uchar>(i, j) == 255) {
                    if (i < rectW / 2) ++cntLeftOnes;
                        else ++cntRightOnes;
                }

        cntLeftOnes = cntLeftOnes * cntLeftOnes;
        cntRightOnes = cntRightOnes * cntRightOnes;
        turnFactor = cntLeftOnes > cntRightOnes ? 1 : -1;   
    }

    skip: 
    addWeighted(rgb, 0.5, canvas, 1, 1, canvas);
    imshow("SignsDetector", canvas);
    return turnFactor;
}