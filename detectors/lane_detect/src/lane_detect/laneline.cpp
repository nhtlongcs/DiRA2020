#include <algorithm>
#include "lane_detect/laneline.h"
#include "common/libcommon.h"

LaneLine::LaneLine()
    : lineParams{nullptr}
{
}

void LaneLine::update(const cv::Mat &lineImage)
{
    this->lineImage = lineImage.clone();
    lineImageRect = cv::Rect{0, 0, lineImage.cols, lineImage.rows};

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
    lineParams = nullptr;
}

void LaneLine::track()
{
    // TODO: fix me
    std::vector<cv::Point> trackingPoints;
    for (int y = lineImage.rows - 1; y > 0; y -= 5)
    {
        int x = getXByY(*lineParams, y * 1.0);
        cv::Point center{x, y};
        int nonZeroValue = 0;
        if (updateNewCenter(lineImage, center, &nonZeroValue))
        {
            trackingPoints.push_back(center);
        } else if (nonZeroValue < 10)
        {
            // Lost tracking
            break;
        }
    }

    if (trackingPoints.size() >= minPointTrack)
    {
        lineParams = calcLineParams(trackingPoints);
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
        }
    }
}

LineParams LaneLine::getLineParams() const
{
    return *(this->lineParams);
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
    return lineParams != nullptr;
}

std::vector<cv::Point> LaneLine::calcGradient(const std::vector<cv::Point> &points) const
{
    std::vector<cv::Point> gradients;
    for (int i = 0; i < points.size() - 1; i++)
    {
        gradients.push_back(points[i + 1] - points[i]);
    }
    return gradients;
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

void LaneLine::swap(std::shared_ptr<LaneLine> other)
{
    std::swap(other->lineParams, this->lineParams);
}

void LaneLine::setFindBeginPointRegion(int offset, int width)
{
    this->offsetX = offset;
    this->width = width;
}

bool LaneLine::findBeginPoint(cv::Point &returnPoint) const
{
    cv::Rect &&regionRect = getDetectBeginPointRegion();

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
    if (!lane->isFound() || laneWidth <= 0)
    {
        return false;
    }

    std::vector<cv::Point> &&movedPoints = lane->moveByGradient(laneWidth);

    auto newParams = calcLineParams(movedPoints);
    if (newParams)
    {
        lineParams = newParams;
        return true;
    }
    else
    {
        return false;
    }
}

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

void LeftLane::showLinePoints(cv::Mat &drawImage) const
{
    for (int y = 0; y < drawImage.rows; y += 20)
    {
        const cv::Point point{getXByY(*lineParams, y * 1.0), y};
        circle(drawImage, point, 5, getLaneColor(), -1, 1, 0);
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

void RightLane::showLinePoints(cv::Mat &drawImage) const
{
    for (int y = 7; y < drawImage.rows; y += 20)
    {
        const cv::Point point{getXByY(*lineParams, y * 1.0), y};
        circle(drawImage, point, 5, getLaneColor(), -1, 1, 0);
    }
}

cv::Point RightLane::calcPerpendicular(const cv::Point &point) const
{
    return {-point.y, point.x};
}