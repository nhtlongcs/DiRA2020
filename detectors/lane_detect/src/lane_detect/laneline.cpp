#include <algorithm>
#include "lane_detect/laneline.h"
#include "common/libcommon.h"

LaneLine::LaneLine(bool debug)
    : isDebug{debug}, lineParams{nullptr}, count_detect{0}
{
}

LaneLine::~LaneLine()
{
}

void LaneLine::update(const cv::Mat &lineImage)
{
    this->lineImage = lineImage.clone();
    lineImageRect = cv::Rect{0, 0, lineImage.cols, lineImage.rows};

    // if (isDebug)
    // {
    //     cv::cvtColor(lineImage, debugImage, cv::COLOR_GRAY2BGR);
    // }

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
    auto iter = listPoint.begin();
    while (iter != listPoint.end())
    {
        if (!updateNewCenter(lineImage, *iter, nullptr))
        {
            iter = listPoint.erase(iter);
        }
        else
        {
            iter++;
        }
    }

    if (listPoint.size() > minPointTrack)
    {
        lineParams = calcLineParams(listPoint);
        listPoint = getPointsFromParams(lineParams);
    }
    else
    {
        reset();
        detect();
    }

    // for (const auto& point : listPoint)
    // {
    //     cv::circle(debugImage, point, 3, getLaneColor(), -1);
    // }
    // cv::imshow("debug tracking", debugImage);
    // cv::waitKey(1);
}

void LaneLine::detect()
{
    count_detect++;
    cv::Point beginPoint;
    if (findBeginPoint(beginPoint))
    {
        std::vector<cv::Point> points;
        std::vector<cv::Point> &&pointsUp = findPoints(beginPoint, 1);
        std::vector<cv::Point> &&pointsDown = findPoints(beginPoint, -1);
        std::move(pointsUp.begin(), pointsUp.end(), std::back_inserter(points));
        std::move(pointsDown.begin(), pointsDown.end(), std::back_inserter(points));

        if (points.size() > minPointDetect)
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

// void LaneLine::setLineParams(std::shared_ptr<LineParams> params)
// {
//     this->lineParams = params;
// }

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
    return listPoint.size() > minPointTrack && lineParams != nullptr;
}

std::vector<cv::Point> LaneLine::calcGradient(const std::vector<cv::Point> &points) const
{
    std::vector<cv::Point> gradients;
    for (int i = 0; i < listPoint.size() - 1; i++)
    {
        gradients.push_back(listPoint[i + 1] - listPoint[i]);
    }
    return gradients;
}

void LaneLine::show(cv::Mat &drawImage, bool showDetectRegion) const
{
    if (isFound())
    {
        const cv::Scalar &&color = getLaneColor();
        cv::Point beginPoint;
        getBeginPoint(beginPoint);
        circle(drawImage, beginPoint, 20, color, -1, 8, 0);
        showLinePoints(drawImage);

        if (showDetectRegion)
        {
            const cv::Rect &&regionRect = getDetectBeginPointRegion();
            cv::Mat region = drawImage(regionRect);
            cv::Mat colorMap{regionRect.size(), CV_8UC3, getLaneColor()};
            cv::addWeighted(region, 0.5, colorMap, 0.5, 1, region);
        }
    }
}

void LaneLine::getMask(cv::Mat &birdviewInputBackground) const
{
    for (size_t i = 0; i < listPoint.size(); ++i)
    {
        circle(birdviewInputBackground, listPoint[i], 5, cv::Scalar{255}, -1, 1, 0);
    }
}

bool LaneLine::getBeginPoint(cv::Point &returnPoint) const
{
    if (isFound() && listPoint.size() > beginPointIndex)
    {
        returnPoint = *(listPoint.rbegin() + beginPointIndex);
        return true;
    }
    return false;
}

bool LaneLine::getDrivePoint(cv::Point &returnPoint) const
{
    if (isFound() && listPoint.size() > drivePointIndex)
    {
        returnPoint = *(listPoint.rbegin() + drivePointIndex);
        return true;
    }
    return false;
}

bool LaneLine::isOutOfImage(const cv::Point &point) const
{
    return point.x < 0 || point.x >= lineImage.cols || point.y < 0 || point.y >= lineImage.rows;
}

cv::Rect LaneLine::getROITracking(cv::Point centerPoint) const
{
    return cv::Rect{centerPoint.x - W_TRACKING / 2, centerPoint.y - H_TRACKING / 2, W_TRACKING, H_TRACKING};
}

bool LaneLine::findPointHasBiggestValueInBin(const cv::Mat &trackingImage, cv::Point &returnPoint, int &returnValue) const
{
    int maxCount = 0;
    cv::Point maxPoint;

    const int BIN_WIDTH = trackingImage.cols / N_BINS;
    for (int bin = 0; bin < N_BINS; bin++)
    {
        cv::Rect binROI{bin * BIN_WIDTH, 0, BIN_WIDTH, trackingImage.rows};
        cv::Mat binImage = trackingImage(binROI);
        int nonZero = countNonZero(binImage);

        if (nonZero >= maxCount)
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

std::vector<cv::Point> LaneLine::getPoints() const
{
    return this->listPoint;
}

void LaneLine::swap(std::shared_ptr<LaneLine> other)
{
    std::vector<cv::Point> tmpListPoint = this->listPoint;
    listPoint = other->listPoint;
    other->listPoint = tmpListPoint;

    std::shared_ptr<LineParams> tmpLineParams = this->lineParams;
    this->lineParams = other->lineParams;
    other->lineParams = tmpLineParams;
}

void LaneLine::setFindBeginPointRegion(int offset, int width)
{
    this->offsetX = offset;
    this->width = width;
}

bool LaneLine::findBeginPoint(cv::Point &returnPoint) const
{
    cv::Rect &&regionRect = getDetectBeginPointRegion();
    ROS_INFO("findBeginPoint, region = %d, %d, %d, %d", regionRect.x, regionRect.y, regionRect.width, regionRect.height);

    const cv::Mat region = lineImage(regionRect);

    for (int center_y = region.rows - 1 - H_TRACKING / 2; center_y > H_TRACKING / 2; center_y -= H_TRACKING)
    {
        int maxNonZero = 0;
        cv::Point maxPoint;

        for (int center_x = W_TRACKING / 2 + 1; center_x < region.cols - W_TRACKING / 2 - 1; center_x += W_TRACKING)
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

        if (maxNonZero > maxNonZeroThreshold)
        {
            returnPoint = regionRect.tl() + maxPoint;
            return true;
        }
    }

    return false;
}

std::vector<cv::Point> LaneLine::findPoints(const cv::Point &beginPoint, int direct) const
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
        nextPoint.y += direct * H_TRACKING / 3;
    }
    return result;
}

bool LaneLine::updateNewCenter(const cv::Mat &region, cv::Point &center, int *const nonZeroValue) const
{
    const cv::Rect roiTracking = getROITracking(center);

    // if (isDebug)
    // {
    //     cv::rectangle(debugImage, roiTracking, getLaneColor(), 1);
    //     cv::imshow("Debug", debugImage);
    //     cv::waitKey(1);
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

std::vector<cv::Point> LaneLine::moveByGradient(int distance) const
{
    std::vector<cv::Point> points = getPointsFromParams(lineParams);
    std::vector<cv::Point> gradients = calcGradient(points);
    for (int i = 0; i < points.size() - 1; i++)
    {
        points[i] += calcPerpendicular(gradients[i]) * distance;
    }
    return points;
}

bool LaneLine::recover(const std::shared_ptr<LaneLine> &lane, int laneWidth)
{
    if (!lane->isFound() || laneWidth < 0)
    {
        return false;
    }

    // if (isDebug)
    // {
    //     lane->show(debugImage);
    //     cv::imshow("Debug", debugImage);
    //     cv::waitKey(1);
    // }

    std::vector<cv::Point> &&movedPoints = lane->moveByGradient(laneWidth);

    // if (isDebug)
    // {
    //     for (auto& p : movedPoints)
    //     {
    //         cv::circle(debugImage, p, 5, cv::Scalar{0,255,255}, -1);
    //     }
    //     cv::imshow("Debug", debugImage);
    //     cv::waitKey(1);
    // }

    auto newParams = calcLineParams(movedPoints);
    if (newParams)
    {
        lineParams = newParams;
        listPoint = getPointsFromParams(lineParams);
        return isFound();
    }
    else
    {
        return false;
    }
}

// bool LaneLine::getDrivePoint(std::shared_ptr<LaneLine> other, cv::Point& returnPoint) const
// {
//     if (this->isFound() && other->isFound())
//     {
//         LineParams midLine;
//         std::shared_ptr<LineParams> leftParams = other->getLineParams();
//         for (size_t i = 0; i < midLine.size(); i++)
//         {
//             midLine[i] = (*leftParams)[i] + (*lineParams)[i]) / 2;
//         }

//         int y = drivePointIndex;
//         int x = getXByY(midLine, y);
//         returnPoint = cv::Point{x, y};
//         return true;
//     } else if (this->isFound())
//     {

//     }
// }

//////////////////////////////////////////

cv::Rect LeftLane::getDetectBeginPointRegion() const
{
    int real_width = std::min(width, lineImage.cols - offsetX - 1);
    return cv::Rect{offsetX, 0, real_width, lineImage.rows};
}

cv::Scalar LeftLane::getLaneColor() const
{
    return cv::Scalar{0, 255, 0}; // Green
}

void LeftLane::printDetectCount() const
{
    std::cout << "Left lane detect count: " << count_detect << std::endl;
}

void LeftLane::showLinePoints(cv::Mat &drawImage) const
{
    for (size_t i = 0; i < listPoint.size(); i += 20)
    {
        circle(drawImage, listPoint[i], 5, getLaneColor(), -1, 1, 0);
    }
}

cv::Point LeftLane::calcPerpendicular(const cv::Point &point) const
{
    return cv::Point{point.y, -point.x} / sqrt(point.y * point.y + point.x * point.x);
}

//////////////////////////////////////////

cv::Rect RightLane::getDetectBeginPointRegion() const
{
    int x = std::max(0, lineImage.cols - offsetX - width - 1);
    int real_width = std::min(width, lineImage.cols - x);
    return cv::Rect{x, 0, real_width, lineImage.rows};
}

cv::Scalar RightLane::getLaneColor() const
{
    return cv::Scalar{255, 0, 0};
}

void RightLane::printDetectCount() const
{
    std::cout << "Right lane detect count: " << count_detect << std::endl;
}

void RightLane::showLinePoints(cv::Mat &drawImage) const
{
    for (size_t i = 7; i < listPoint.size(); i += 20)
    {
        circle(drawImage, listPoint[i], 5, getLaneColor(), -1, 1, 0);
    }
}

cv::Point RightLane::calcPerpendicular(const cv::Point &point) const
{
    return {-point.y, point.x};
}
///////////////////////////////////////////////

// MidLane::MidLane(LaneLine& leftLane, LaneLine& rightLane)
// : LaneLine::LaneLine{true}
// , leftLane{leftLane}
// , rightLane{rightLane}
// , laneSize{0}
// {
// }

// void MidLane::detect()
// {
//     // NOTE: left and right lanes should be updated before midlane!!!
//     if (!leftLane.isFound() && !rightLane.isFound())
//     {
//         // I'm blind!
//         return;
//     }

//     if (leftLane.isFound() && rightLane.isFound())
//     {
//         detectIfHaveBothLanes();
//     } else if (laneSize != 0)
//     {
//         lineParams = std::make_shared<LineParams>();
//         if (leftLane.isFound())
//         {
//             const auto& leftParams = leftLane.getLineParams();
//             std::copy(leftParams.begin(), leftParams.end(), (*lineParams).begin());
//             (*lineParams)[2] += laneSize;
//         } else
//         {
//             const auto& rightParams = rightLane.getLineParams();
//             std::copy(rightParams.begin(), rightParams.end(), (*lineParams).begin());
//             (*lineParams)[2] -= laneSize;
//         }
//     } else
//     {
//         // Cannot recover lane due to unknow lane size
//         return;
//     }

//     if (lineParams)
//     {
//         listPoint = getPointsFromParams(lineParams);
//     }

// }

// void MidLane::detectIfHaveBothLanes()
// {
//     const auto& leftParams = leftLane.getLineParams();
//     const auto& rightParams = rightLane.getLineParams();
//     lineParams = std::make_shared<LineParams>();
//     for (size_t i = 0; i < (*lineParams).size(); i++)
//     {
//         (*lineParams)[i] = (leftParams[i] + rightParams[i]) / 2;
//     }

//     cv::Point leftBegin, rightBegin;
//     leftLane.getBeginPoint(leftBegin);
//     rightLane.getBeginPoint(rightBegin);
//     laneSize = rightBegin.x - leftBegin.x;
// }

// cv::Scalar MidLane::getLaneColor() const
// {
//     return cv::Scalar{0, 0, 255};
// }

// std::vector<cv::Point> MidLane::findListPoint() const
// {
//     // bool leftFound = leftLane.getBeginPoint(leftPoint);
//     // bool rightFound = rightLane.getBeginPoint(rightPoint);
//     // if (leftFound &&)
// }
