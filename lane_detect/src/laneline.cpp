#include <algorithm>
#include "laneline.h"
#include "utils.h"
using namespace cv;

LaneLine::LaneLine()
: isFound_{false}
{
}

LaneLine::~LaneLine()
{
}

void LaneLine::update(const cv::Mat& lineImage)
{
    this->lineImage = lineImage.clone();

    updateListPoint();
}

bool LaneLine::init()
{
}

bool LaneLine::isFound() const
{
    return this->isFound_;
}

void LaneLine::show(cv::Mat& drawImage) const
{
    if (isFound_ == true)
    {
        cv::Scalar color = getLaneColor();

        circle(drawImage, beginPoint, 1, color, 20, 8, 0);
        for (int i = 0; i < listPoint.size(); i++)
        {
            circle(drawImage, listPoint[i], 1, color, 8, 8, 0);
            for (int y = 0; y < drawImage.rows; y++)
            {
                int xDraw = getXByY(*lineParams, y);
                Point drawLine = Point(xDraw, y);
                circle(drawImage, drawLine, 1, color, 1, 1, 0);
            }
        }
    }
}

bool LaneLine::isOutOfImage(cv::Point point) const
{
    return point.x < 0 || point.x >= lineImage.cols || point.y < 0 || point.y >= lineImage.rows;
}

cv::Rect LaneLine::getROITracking(cv::Point centerPoint)
{
    return cv::Rect{centerPoint.x - W_TRACKING/2, centerPoint.y - H_TRACKING/2, W_TRACKING, H_TRACKING};
}

bool LaneLine::findPointHasBiggestValueInBin(const cv::Mat& trackingImage, cv::Point& returnPoint, int& returnValue) const
{
    int maxCount = 0;
    cv::Point maxPoint;

    const int BIN_WIDTH = trackingImage.cols / N_BINS;
    for(int bin = 0; bin < N_BINS; bin++)
    {
        cv::Rect binROI{bin * BIN_WIDTH, 0, BIN_WIDTH, trackingImage.rows};
        cv::Mat binImage = trackingImage(binROI);
        int nonZero = countNonZero(binImage);
        
        if(nonZero >= maxCount)
        {
            maxCount = nonZero;
            maxPoint = Point(bin * BIN_WIDTH / 2, binImage.rows / 2);
        }
    }

    if (maxCount == 0)
    {
        return false;
    }

    returnPoint = maxPoint;
    returnValue = maxCount;
    return true;
}

void LaneLine::updateListPointUp()
{
    Point nextPoint = Point{beginPoint.x,beginPoint.y-H_TRACKING};
    listPoint.clear();
    
    while(!isOutOfImage(nextPoint))
    {
        Rect roiTracking = getROITracking(nextPoint);
        cv::Mat trackingImage = lineImage(roiTracking);

        int maxNonZero = 0;
        Point maxPoint;
        bool found = findPointHasBiggestValueInBin(trackingImage, maxPoint, maxNonZero);
        if (!found)
        {
            break;
        }
        
        nextPoint = maxPoint;
        listPoint.insert(listPoint.begin(), nextPoint);
        nextPoint.y -= H_TRACKING;
    }
}

void LaneLine::updateListPointDown()
{
    Point nextPoint = Point{beginPoint.x,beginPoint.y + H_TRACKING};
    listPoint.clear();
    
    while(!isOutOfImage(nextPoint))
    {
        Rect roiTracking = getROITracking(nextPoint);
        cv::Mat trackingImage = lineImage(roiTracking);

        int maxNonZero = 0;
        Point maxPoint;
        bool found = findPointHasBiggestValueInBin(trackingImage, maxPoint, maxNonZero);
        if (!found)
        {
            break;
        }
        
        nextPoint = maxPoint;
        listPoint.insert(listPoint.begin(), nextPoint);
        nextPoint.y += H_TRACKING;
    }
}

void LaneLine::updateListPoint()
{
    if (!isFound_)
    {
        return;
    }

    updateListPointUp();
    updateListPointDown();

    isFound_ = listPoint.size() > 3;

    if (isFound_)
    {
        beginPoint = listPoint[3]; 
        endPoint = listPoint.back();

        lineParams = calcLineParams(listPoint) ;
        for(int i=0; i<listPoint.size();i++)
        {
            listPoint[i].x = getXByY(*lineParams, listPoint[i].y);
        }
    }
    else
    {
        beginPoint = Point{-1,-1};
        endPoint = Point{-1, -1};
        listPoint.clear();
    }
}

//////////////////////////////////////////

bool LeftLane::init()
{
    for(int center_y = lineImage.rows-1-H_TRACKING/2; center_y>H_TRACKING/2; center_y -= H_TRACKING)
    {
        int maxValue = 0;
        cv::Point maxPoint;

        for(int center_x=W_TRACKING/2; center_x<lineImage.cols/2; center_x+=W_TRACKING)
        {
            cv::Rect roiTracking = getROITracking(Point(center_x, center_y));
            cv::Mat trackingImage = lineImage(roiTracking);

            cv::Point maxPointResult;
            int maxValueResult = 0;
            bool found = findPointHasBiggestValueInBin(trackingImage, maxPointResult, maxValueResult);
            if (found)
            {
                maxPoint = maxPointResult + Point{roiTracking.x, roiTracking.y};
                maxValue = maxValueResult;
            }
        }
        
        if(maxValue > 0) {
            beginPoint = maxPoint;
            isFound_ = true;
            return true;
        }
    }

    return false;
}

int LeftLane::getType() const
{
    return LEFT;
}

cv::Scalar LeftLane::getLaneColor() const
{
    return cv::Scalar{0, 255, 0}; // Green
}


//////////////////////////////////////////

bool RightLane::init()
{
    for(int center_y = lineImage.rows-1-H_TRACKING/2; center_y>H_TRACKING/2; center_y -= H_TRACKING)
    {
        int maxNonZero = 0;
        Point maxPoint;

        for(int center_x=lineImage.cols/2+W_TRACKING/2; center_x<lineImage.cols-W_TRACKING/2; center_x+=W_TRACKING)
        {
            cv::Rect roiTracking = getROITracking(Point(center_x, center_y));
            cv::Mat trackingImage = lineImage(roiTracking);

            const int BIN_WIDTH = trackingImage.cols / N_BINS;
            for(int bin = 0; bin < N_BINS; bin++)
            {
                cv::Rect binROI{bin * BIN_WIDTH, 0, BIN_WIDTH, trackingImage.rows};
                cv::Mat binImage = trackingImage(binROI);
                int nonZero = countNonZero(binImage);
                
                if(nonZero>=maxNonZero)
                {
                    maxNonZero = nonZero;
                    maxPoint = Point(
                        roiTracking.x + bin * BIN_WIDTH / 2,
                        roiTracking.y + binImage.rows / 2
                    );
                }
            }                                                  
        }
        
        if(maxNonZero > 0) {
            beginPoint = maxPoint;
            isFound_ = true;
            return true;
        }
    }

    return false;
}

int RightLane::getType() const
{
    return RIGHT;
}

cv::Scalar RightLane::getLaneColor() const
{
    return cv::Scalar{255, 0, 0};
}