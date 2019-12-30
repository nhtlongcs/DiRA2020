#include "detectlane.h"
#include "laneline.h"
#include "utils.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

constexpr const char* CONF_BIRDVIEW_WINDOW = "Birdview";

DetectLane::DetectLane()
: leftLane{nullptr}
, rightLane{nullptr}
, frameCount{0}
, sumLaneWidth{0}
// , midLane{nullptr}
{
    
    // setUseOptimized(true);
    // setNumThreads(4);
 
    // cvCreateTrackbar("Hough", "Threshold", &hough_lowerbound, max_houghThreshold);
    // cvCreateTrackbar("Hough_minLinlength", "Threshold", &minLinlength, 150);

    // cv::namedWindow("Threshold");
    // cv::createTrackbar("MinShadow H", "Threshold", &minLaneInShadow[0], 255);
    // cv::createTrackbar("MinShadow S", "Threshold", &minLaneInShadow[1], 255);
    // cv::createTrackbar("MinShadow V", "Threshold", &minLaneInShadow[2], 255);
    // cv::createTrackbar("MaxShadow H", "Threshold", &maxLaneInShadow[0], 255);
    // cv::createTrackbar("MaxShadow S", "Threshold", &maxLaneInShadow[1], 255);
    // cv::createTrackbar("MaxShadow V", "Threshold", &maxLaneInShadow[2], 255);
    // cv::createTrackbar( "Min Threshold:", "Threshold", &lowThreshold, 15);

    cv::namedWindow(CONF_BIRDVIEW_WINDOW);
    cv::createTrackbar("Use Birdview", CONF_BIRDVIEW_WINDOW, &usebirdview, 1);
    cv::createTrackbar("Birdwidth", CONF_BIRDVIEW_WINDOW, &birdwidth, 400);
    cv::createTrackbar("Birdheight", CONF_BIRDVIEW_WINDOW, &birdheight, 400);
    cv::createTrackbar("Skyline", CONF_BIRDVIEW_WINDOW, &skyline, 200);
    cv::createTrackbar("OffsetLeft", CONF_BIRDVIEW_WINDOW, &offsetLeft, 200);
    cv::createTrackbar("OffsetRight", CONF_BIRDVIEW_WINDOW, &offsetRight, 200);
    
    leftLane = std::make_shared<LeftLane>();
    rightLane = std::make_shared<RightLane>();

    // midLane = new MidLane(*leftLane, *rightLane);
}

DetectLane::~DetectLane(){
} 

void DetectLane::updateBinary(const cv::Mat& src)
{
    this->binary = src.clone();
}

void DetectLane::updateDepth(const cv::Mat& src)
{
    this->depth = src.clone();
}

void DetectLane::updateRGB(const cv::Mat& src)
{
    this->rgb = src.clone();
}

bool DetectLane::isNeedRedetect(cv::Point leftBegin, cv::Point rightBegin) const
{
    return abs(leftBegin.x - rightBegin.x) < 50;
}

void DetectLane::detect()
{
    if (this->rgb.empty())
    {
        return;
    }

    if(this->binary.empty())
    {
        return;
    }

    cv::imshow("RGB", this->rgb);

    // Mat binary = preprocess(this->rgb);

    // Mat shadowMask = shadow(this->rgb);
    // bitwise_or(binary, shadowMask, binary);
    // imshow("binary", this->binary);

    if (usebirdview)
    {
        cv::Mat M;
        imshow("binary", this->binary);

        // cv::Mat temp = this->binary(cv::Rect{0,this->binary.rows/2, this->binary.cols, this->binary.rows/2});
        this->binary(cv::Rect{0, 0, this->binary.cols, this->binary.rows/2}) = cv::Scalar{0};
        imshow("temp", this->binary);

        this->birdview = birdviewTransformation(this->binary, birdwidth, birdheight, skyline, offsetLeft, offsetRight, M);
        imshow("birdview", this->birdview);
    }
    else
    {
        birdview = this->binary;
    }

    // birdview = this->binary;

    // Mat morphBirdview = morphological(birdview);

    birdview(cv::Rect(0,0,birdview.cols, birdview.rows / 3)) = cv::Scalar{0};

    leftLane->update(birdview);
    rightLane->update(birdview);

    if (leftLane->isFound() && rightLane->isFound())
    {
        cv::Point leftBegin, rightBegin;
        leftLane->getBeginPoint(leftBegin);
        rightLane->getBeginPoint(rightBegin);

        if (isNeedRedetect(leftBegin, rightBegin))
        {
            leftLane->reset();
            rightLane->reset();
        }
        else
        {
            frameCount++;
            sumLaneWidth += abs(leftBegin.x - rightBegin.x);
        }
    }

}

void DetectLane::show(const cv::Point* drivePoint) const
{
    if (birdview.empty())
    {
        return;
    }
    cv::Mat birdviewColor;
    cv::cvtColor(birdview, birdviewColor, cv::COLOR_GRAY2BGR);
    leftLane->show(birdviewColor);
    rightLane->show(birdviewColor);

    if (drivePoint)
    {
        cv::circle(birdviewColor, *drivePoint, 25, cv::Scalar{0, 255, 255}, -1);
    }

    cv::imshow("Lanes", birdviewColor);
}

Mat DetectLane::shadow(const Mat& src) {
    Mat shadow, hsv;
    cvtColor(src, hsv, cv::COLOR_BGR2HSV);
    inRange(hsv, Scalar(minLaneInShadow[0], minLaneInShadow[1], minLaneInShadow[2]),
                    Scalar(maxLaneInShadow[1], maxLaneInShadow[1], maxLaneInShadow[2]), shadow);
    return shadow;
}


Point DetectLane::Hough(const Mat& img, const Mat& src) {
    vector<Vec4f> lines;
    Mat HoughTransform = Mat::zeros(src.size(), CV_8UC3);
    float cntL = 0.0, cntR = 0.0;
    float leftSlope_average = 0.0;
    float rightSlope_average = 0.0;
    float leftYintercept_average = 0.0;
    float rightYintercept_average = 0.0;

    HoughLinesP(img, lines, 1, CV_PI / 180, votes, minLinlength, maxLineGap);

    for (int i = 0; i < (int)lines.size(); ++i) {
        Vec4f t = lines[i];
        float slope = (t[1] - t[3]) / (t[0] - t[2]);
        float Yintercept = t[1] - slope * t[0];
        
        if (slope < 0.0) { 
            leftSlope_average += slope, leftYintercept_average += Yintercept, cntL += 1.0;
        }
        else {
            rightSlope_average += slope, rightYintercept_average += Yintercept, cntR += 1.0;
        }
    }

    if (cntL > 0) {
        leftSlope_average /= cntL;
        leftYintercept_average /= cntL;
        drawLine(leftSlope_average, leftYintercept_average, HoughTransform);
    }

    if (cntR > 0) {
        rightSlope_average /= cntR;
        rightYintercept_average /= cntR;
        drawLine(rightSlope_average, rightYintercept_average, HoughTransform);
    }

    float midY = offsetY;
    float midX = offsetX;

    if (fabs(rightSlope_average) < 0.05) cntR = 0;
    if (fabs(leftSlope_average) < 0.05) cntL = 0;
    
    if (cntL == 0.0 && cntR > 0) {
        midX = (midY - rightYintercept_average) / rightSlope_average;
    } 
    else if (cntL > 0 && cntR == 0.0) {
        midX = (midY - leftYintercept_average) / leftSlope_average;
    }
    else if (cntL > 0 && cntR > 0) {
        midX = (((midY - leftYintercept_average) / leftSlope_average) + ((midY - rightYintercept_average) / rightSlope_average)) / 2.0;
    }

    addWeighted(src, 0.5, HoughTransform, 1, 1, HoughTransform);
    circle(HoughTransform, Point(midX, midY), 3, Scalar(0, 0, 255), -1);
    circle(HoughTransform, Point(offsetX, offsetY), 3, Scalar(0, 255, 0), -1);
    imshow("HoughLines", HoughTransform);

    return Point(midX, midY);
}

void DetectLane::drawLine(float slope, float y_intercept, Mat &HoughTransform) {
    float y0 = 240.0;
    float y1 = 1.0;
    float x0 = (y0 - y_intercept) / slope;
    float x1 = (y1 - y_intercept) / slope;
    line(HoughTransform, Point(x0, y0), Point(x1, y1), Scalar(0, 255, 0), 2, LINE_AA);
}

Mat DetectLane::ROI(const Mat &src) {
    //Crop region of interest (ROI)
    int W = src.size().width;
    int H = src.size().height;
    Mat RegionOfInterest;
    Mat mask = Mat::zeros(src.size(), CV_8UC1);

    vector<Point> vertices;
    vertices.push_back(Point(0, skyline));
    vertices.push_back(Point(W, skyline));
    vertices.push_back(Point(W, H));
    vertices.push_back(Point(0, H));
    
    fillConvexPoly(mask, vertices, Scalar(255, 255, 255), 8, 0);
    bitwise_and(src, mask, RegionOfInterest);
    
    //imshow("RegionOfInterest", RegionOfInterest);
    return RegionOfInterest;
}

Mat DetectLane::morphological(const Mat& img) {
    Mat dst;

    dilate(img, dst, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

    return dst;
}

Mat DetectLane::preprocess(const Mat& src) { 
    Mat hsv, binary;
    
    cvtColor(src, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(minThreshold[0], minThreshold[1], minThreshold[2]), 
                 Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]), binary);

    return binary;
}


int DetectLane::getLaneWidth() const
{
    if (this->frameCount == 0)
    {
        return 0;
    }
    return this->sumLaneWidth / this->frameCount;
}

std::shared_ptr<LaneLine> DetectLane::getLeftLane() const
{
    return this->leftLane;
}

std::shared_ptr<LaneLine> DetectLane::getRightLane() const
{
    return this->rightLane;
}