#include <algorithm>
#include "laneline.h"
#include "utils.h"

LaneLine::LaneLine(bool debug)
: isDebug{debug}
, lineParams{nullptr}
{
}

LaneLine::~LaneLine()
{
}

void LaneLine::update(const cv::Mat& lineImage)
{
    this->lineImage = lineImage.clone();
    lineImageRect = cv::Rect{ 0, 0, lineImage.cols, lineImage.rows};

    if (isDebug)
    {
        cv::cvtColor(lineImage, debugImage, cv::COLOR_GRAY2BGR);
    }

    if (isFound())
    {
        track();
    }
    else
    {
        reset();
        detect();
    }
}

void LaneLine::reset()
{
    listPoint.clear();
    lineParams = nullptr;
}

void LaneLine::track()
{
    auto iterPoint = listPoint.begin();
    while (iterPoint != listPoint.end())
    {
        if (!updateNewCenter(lineImage, *iterPoint, nullptr))
        {
            iterPoint = listPoint.erase(iterPoint);
        } else
        {
            iterPoint++;
        }
    }

    if (listPoint.size() > minPoint)
    {
        lineParams = calcLineParams(listPoint);
    }

    listPoint = getPointsFromParams(lineParams);
}

void LaneLine::detect()
{
    cv::Point beginPoint;
    if (findBeginPoint(beginPoint))
    {
        std::vector<cv::Point> points;
        std::vector<cv::Point>&& pointsUp = findPoints(beginPoint, 1);
        std::vector<cv::Point>&& pointsDown = findPoints(beginPoint, -1);
        std::move(pointsUp.begin(), pointsUp.end(), std::back_inserter(points));
        std::move(pointsDown.begin(), pointsDown.end(), std::back_inserter(points));

        if (points.size() > minPoint)
        {
            lineParams = calcLineParams(points);
            listPoint = getPointsFromParams(lineParams);
        }
    }
}

LineParams LaneLine::getLineParams() const
{
    return *(this->lineParams);
}

void LaneLine::setLineParams(std::shared_ptr<LineParams> params)
{
    this->lineParams = params;
}

cv::Rect LaneLine::getDetectBeginPointRegion() const
{
    return cv::Rect{0, 0, lineImage.cols, lineImage.rows};
}

std::vector<cv::Point> LaneLine::getPointsFromParams(const std::shared_ptr<LineParams> params) const
{
    std::vector<cv::Point> points;
    if (params)
    {
        for (int y = 0; y < lineImage.rows; y++)
        {
            cv::Point point{getXByY(*lineParams, y), y};
            if (!isOutOfImage(point))
            {
                points.push_back(point);
            }
        }
    }
    return points;
}

bool LaneLine::isFound() const
{
    return listPoint.size() > minPoint;
}

void LaneLine::show(cv::Mat& drawImage) const
{
    if (isFound())
    {
        const cv::Scalar&& color = getLaneColor();
        cv::Point beginPoint;
        getBeginPoint(beginPoint);
        circle(drawImage, beginPoint , 20, color, -1, 8, 0);
        for (const auto& point : listPoint)
        {
            circle(drawImage, point, 5, color, -1, 1, 0);
        }
    }
}

bool LaneLine::getBeginPoint(cv::Point& returnPoint) const
{
    if (isFound() && listPoint.size() > beginPointIndex)
    {
        returnPoint = *(listPoint.rbegin() + beginPointIndex);
        return true;
    }
    return false;
}

bool LaneLine::isOutOfImage(const cv::Point& point) const
{
    return point.x < 0 || point.x >= lineImage.cols || point.y < 0 || point.y >= lineImage.rows;
}

cv::Rect LaneLine::getROITracking(cv::Point centerPoint) const
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
            maxPoint = cv::Point2i{int((bin + 0.5) * BIN_WIDTH), int(binImage.rows / 2)};
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

bool LaneLine::findBeginPoint(cv::Point& returnPoint) const
{
    cv::Rect&& regionRect = getDetectBeginPointRegion();

    const cv::Mat region = lineImage(regionRect);

    for(int center_y = region.rows-1-H_TRACKING/2; center_y>H_TRACKING/2; center_y -= H_TRACKING)
    {
        int maxNonZero = 0;
        cv::Point maxPoint;

        for(int center_x=W_TRACKING/2 + 1; center_x<region.cols-W_TRACKING/2 - 1; center_x+=W_TRACKING)
        {
            cv::Point center{center_x, center_y};
            int nonZeroValue = 0;
            updateNewCenter(region, center, &nonZeroValue);
            if (nonZeroValue > maxNonZero)
            {
                maxNonZero = nonZeroValue;
                maxPoint = center;
            }
        }
        
        if(maxNonZero > maxNonZeroThreshold) {
            returnPoint = regionRect.tl() + maxPoint;
            return true;
        }
    }

    return false;
}

std::vector<cv::Point> LaneLine::findPoints(const cv::Point& beginPoint, int direct) const
{
    std::vector<cv::Point> result;
    cv::Point nextPoint{beginPoint.x, beginPoint.y + direct * H_TRACKING / 2};

    while (true)
    {
        if (!updateNewCenter(lineImage, nextPoint, nullptr))
        {
            break;
        }
        result.push_back(nextPoint);
        nextPoint.y += direct * H_TRACKING / 2;
    }
    return result;
}

bool LaneLine::updateNewCenter(const cv::Mat& region, cv::Point& center, int * const nonZeroValue) const
{
    const cv::Rect roiTracking = getROITracking(center);

    // if (isDebug)
    // {
    //     cv::rectangle(debugImage, roiTracking, getLaneColor(), 3);
    //     cv::imshow("Debug", debugImage);
    //     cv::waitKey(0);
    // }

    if ((roiTracking & lineImageRect) != roiTracking)
    {
        return false;
    }
    const cv::Mat trackingImage = region(roiTracking);

    cv::Point maxPointResult;
    int value = 0;
    bool found = findPointHasBiggestValueInBin(trackingImage, maxPointResult, value);
    if (found)
    {
        center = roiTracking.tl() + maxPointResult;
        if (nonZeroValue)
        {
            *nonZeroValue = value;
        }
    }
    return found;
}

//////////////////////////////////////////

cv::Rect LeftLane::getDetectBeginPointRegion() const
{
    return cv::Rect{0, 0, lineImage.cols / 2, lineImage.rows};
}

cv::Scalar LeftLane::getLaneColor() const
{
    return cv::Scalar{0, 255, 0}; // Green
}


//////////////////////////////////////////

cv::Rect RightLane::getDetectBeginPointRegion() const
{
    return cv::Rect{lineImage.cols / 2, 0, lineImage.cols / 2, lineImage.rows};
}

cv::Scalar RightLane::getLaneColor() const
{
    return cv::Scalar{255, 0, 0};
}

///////////////////////////////////////////////

MidLane::MidLane(LaneLine& leftLane, LaneLine& rightLane)
: LaneLine::LaneLine{true}
, leftLane{leftLane}
, rightLane{rightLane}
, laneSize{0}
{
}

void MidLane::detect()
{
    // NOTE: left and right lanes should be updated before midlane!!!
    if (!leftLane.isFound() && !rightLane.isFound())
    {
        // I'm blind!
        return;
    }

    if (leftLane.isFound() && rightLane.isFound())
    {
        detectIfHaveBothLanes();
    } else if (laneSize != 0)
    {
        lineParams = std::make_shared<LineParams>();
        if (leftLane.isFound())
        {
            const auto& leftParams = leftLane.getLineParams();
            std::copy(leftParams.begin(), leftParams.end(), (*lineParams).begin());
            (*lineParams)[2] += laneSize;
        } else
        {
            const auto& rightParams = rightLane.getLineParams();
            std::copy(rightParams.begin(), rightParams.end(), (*lineParams).begin());
            (*lineParams)[2] -= laneSize;
        }
    } else
    {
        // Cannot recover lane due to unknow lane size
        return;
    }

    if (lineParams)
    {
        listPoint = getPointsFromParams(lineParams);
    }

}

void MidLane::detectIfHaveBothLanes()
{
    const auto& leftParams = leftLane.getLineParams();
    const auto& rightParams = rightLane.getLineParams();
    lineParams = std::make_shared<LineParams>();
    for (size_t i = 0; i < (*lineParams).size(); i++)
    {
        (*lineParams)[i] = (leftParams[i] + rightParams[i]) / 2;
    }

    cv::Point leftBegin, rightBegin;
    leftLane.getBeginPoint(leftBegin);
    rightLane.getBeginPoint(rightBegin);
    laneSize = rightBegin.x - leftBegin.x;
}

cv::Scalar MidLane::getLaneColor() const
{
    return cv::Scalar{0, 0, 255};
}

// std::vector<cv::Point> MidLane::findListPoint() const
// {
//     // bool leftFound = leftLane.getBeginPoint(leftPoint);
//     // bool rightFound = rightLane.getBeginPoint(rightPoint);
//     // if (leftFound &&)
// }

