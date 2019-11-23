#include "detectlane.h"
#include "laneline.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

DetectLane::DetectLane()
: leftLane{nullptr}
, rightLane{nullptr}
, laneWidth{0}
, frameCount{0}
// , midLane{nullptr}
{
    
    // setUseOptimized(true);
    // setNumThreads(4);
 
    // cvCreateTrackbar("Hough", "Threshold", &hough_lowerbound, max_houghThreshold);
    // cvCreateTrackbar("Hough_minLinlength", "Threshold", &minLinlength, 150);
    cv::createTrackbar("MinShadow H", "Threshold", &minLaneInShadow[0], 255);
    cv::createTrackbar("MinShadow S", "Threshold", &minLaneInShadow[1], 255);
    cv::createTrackbar("MinShadow V", "Threshold", &minLaneInShadow[2], 255);
    cv::createTrackbar("MaxShadow H", "Threshold", &maxLaneInShadow[0], 255);
    cv::createTrackbar("MaxShadow S", "Threshold", &maxLaneInShadow[1], 255);
    cv::createTrackbar("MaxShadow V", "Threshold", &maxLaneInShadow[2], 255);
    cv::createTrackbar( "Min Threshold:", "Threshold", &lowThreshold, 15);

    leftLane = std::make_shared<LeftLane>();
    rightLane = std::make_shared<RightLane>();

    // midLane = new MidLane(*leftLane, *rightLane);
}

DetectLane::~DetectLane(){
} 

void DetectLane::updateDepth(const cv::Mat& src)
{
    this->depth = src.clone();
}

void DetectLane::updateRGB(const cv::Mat& src)
{
    this->rgb = src.clone();
}

void DetectLane::detect()
{
    if (this->rgb.empty())
    {
        return;
    }

    cv::imshow("RGB", this->rgb);

    Mat binary = preprocess(this->rgb);

    Mat shadowMask = shadow(this->rgb);
    bitwise_or(binary, shadowMask, binary);
    imshow("binary", binary);
    
    cv::Mat birdview = birdviewTransformation(binary);
    // Mat morphBirdview = morphological(birdview);

    leftLane->update(birdview);
    rightLane->update(birdview);

    cv::Mat birdviewColor;
    cv::cvtColor(birdview, birdviewColor, cv::COLOR_GRAY2BGR);

    show(birdviewColor);
}

void DetectLane::show(cv::Mat& colorBirdview) const
{
    leftLane->show(colorBirdview);
    rightLane->show(colorBirdview);
    cv::imshow("Lanes", colorBirdview);
}

void DetectLane::processDepth() {
    // if (this->depth.empty())
    // {
    //     return;
    // }
    // imshow("depth", this->depth);


    // Mat samples(this->depth.rows * this->depth.cols, 3, CV_32F);
    // for( int y = 0; y < this->depth.rows; y++ )
    //     for( int x = 0; x < this->depth.cols; x++ )
    //     for( int z = 0; z < 3; z++)
    //         samples.at<float>(y + x*this->depth.rows, z) = this->depth.at<Vec3b>(y,x)[z];

    // int clusterCount = lowThreshold;
    // Mat labels;
    // int attempts = 5;
    // Mat centers;
    // kmeans(samples, clusterCount, labels, TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.0001), attempts, KMEANS_PP_CENTERS, centers );


    // Mat new_image( this->depth.size(), this->depth.type() );
    // for( int y = 0; y < this->depth.rows; y++ )
    //     for( int x = 0; x < this->depth.cols; x++ )
    //     { 
    //     int cluster_idx = labels.at<int>(y + x*this->depth.rows,0);
    //     new_image.at<Vec3b>(y,x)[0] = centers.at<float>(cluster_idx, 0);
    //     new_image.at<Vec3b>(y,x)[1] = centers.at<float>(cluster_idx, 1);
    //     new_image.at<Vec3b>(y,x)[2] = centers.at<float>(cluster_idx, 2);
    //     }

    // Mat birdviewdepth = birdviewTransformation(new_image);
    // imshow("birdviewdepth",birdviewdepth);
    // imshow( "clustered image", new_image );
 

}

Point DetectLane::calculateError(cv::Point carPos) {
    
    cv::Point beginLeft, beginRight;
    if (leftLane->getBeginPoint(beginLeft) && rightLane->getBeginPoint(beginRight))
    {
        if (abs(beginLeft.x - beginRight.x) < 20)
        {
            leftLane->reset();
            rightLane->reset();
            return carPos;
        } else
        {
            cv::Point midPoint = (beginLeft + beginRight) / 2;
            if (frameCount < 20)
            {
                laneWidth = abs(beginLeft.x - beginRight.x);
            }
            return midPoint;
        }
    } else if (leftLane->getBeginPoint(beginLeft))
    {
        return {beginLeft.x + laneWidth / 2, beginLeft.y};
    } else if (rightLane->getBeginPoint(beginRight))
    {
        return {beginRight.x - laneWidth / 2, beginRight.y};
    } else
    {
        return carPos;
    }
    

    // Point currentCarPosition = Hough(ROI(binary), this->rgb);

    // currentCarPosition.x += turn * 120;

    // return currentCarPosition;
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

Mat DetectLane::birdviewTransformation(const Mat& src) {
    int W = src.size().width;
    int H = src.size().height;

    Point2f srcVertices[4];
    srcVertices[0] = Point(0, skyline);
    srcVertices[1] = Point(W, skyline);
    srcVertices[2] = Point(0, H);
    srcVertices[3] = Point(W, H);
 
    Point2f dstVertices[4];
    dstVertices[0] = Point(0, 0);
    dstVertices[1] = Point(birdwidth, skyline);
    dstVertices[2] = Point(skyline, birdheight);
    dstVertices[3] = Point(birdwidth - skyline, birdheight);

    Mat M = getPerspectiveTransform(srcVertices, dstVertices);
    Mat resultBirdview(birdwidth, birdheight, CV_8UC3);
    warpPerspective(src, resultBirdview, M, resultBirdview.size(), INTER_LINEAR, BORDER_CONSTANT);

    // imshow("resultBirdview", resultBirdview);
    return resultBirdview;
}
