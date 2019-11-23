#ifndef LANELINE_H
#define LANELINE_H
#include <array>
#include <memory>

#include <opencv2/opencv.hpp>
#include "utils.h"

#define H_TRACKING 16
#define W_TRACKING 25
#define N_BINS 10


class LaneLine
{
public:
    LaneLine(bool debug=true);
    ~LaneLine();

    void update(const cv::Mat& lineImage);
    bool isFound() const;
    bool isOutOfImage(const cv::Point& point) const;
    void show(cv::Mat& drawImage) const;

    virtual void setLineParams(std::shared_ptr<LineParams> params, size_t laneSize);
    LineParams getLineParams() const;
    bool getBeginPoint(cv::Point& returnPoint) const;
protected:
    void track();
    virtual void reset();
    virtual void detect();


    virtual cv::Rect getDetectBeginPointRegion() const;
    std::vector<cv::Point> getPointsFromParams(const std::shared_ptr<LineParams> params) const;
    

    virtual cv::Scalar getLaneColor() const = 0;
    cv::Rect getROITracking(cv::Point centerPoint) const;

    bool findPointHasBiggestValueInBin(const cv::Mat& trackingImage, cv::Point& returnPoint, int& returnValue) const;
    bool updateNewCenter(const cv::Mat& region, cv::Point& center, int * const nonZeroValue) const;
    
    bool findBeginPoint(cv::Point& returnPoint) const;
    std::vector<cv::Point> findPoints(const cv::Point& beginPoint, int direct) const;
    
protected:
    cv::Mat debugImage;
    cv::Mat lineImage;
    cv::Rect lineImageRect;
    std::vector<cv::Point> listPoint;
    std::shared_ptr<LineParams> lineParams;
    bool isDebug;

    // const size_t maxNonZeroThreshold = W_TRACKING / N_BINS * H_TRACKING;
    const size_t maxNonZeroThreshold = 10;
    const size_t minPoint = 10;
    const size_t beginPointIndex = 20;
};

class LeftLane : public LaneLine
{
public:
    virtual cv::Scalar getLaneColor() const override;
    virtual cv::Rect getDetectBeginPointRegion() const override;

protected:
};

class RightLane : public LaneLine
{
public:
    virtual cv::Scalar getLaneColor() const override;
    virtual cv::Rect getDetectBeginPointRegion() const override;
private:
};

class MidLane : public LaneLine
{
public:
    MidLane(LaneLine& leftLane, LaneLine& rightLane);
    virtual cv::Scalar getLaneColor() const override;

protected:
    virtual void detect() override;
    void detectIfHaveBothLanes();


private:
    size_t laneSize;
    LaneLine& leftLane;
    LaneLine& rightLane;
};

#endif