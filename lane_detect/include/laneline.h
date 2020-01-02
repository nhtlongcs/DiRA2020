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
    void show(cv::Mat& drawImage, bool showDetectRegion = true) const;
    void getMask(cv::Mat& birdviewInputBackground) const;
    void setFindBeginPointRegion(int offset, int width);

    // virtual void setLineParams(std::shared_ptr<LineParams> params, size_t laneSize);
    LineParams getLineParams() const;
    bool getBeginPoint(cv::Point& returnPoint) const;
    // bool getDrivePoint(std::shared_ptr<LaneLine> other, cv::Point& returnPoint) const;
    bool getDrivePoint(cv::Point& returnPoint) const;

    std::vector<cv::Point> getPoints() const;

    void swap(std::shared_ptr<LaneLine> other);

    virtual void reset();
    bool recover(const std::shared_ptr<LaneLine>& lane, int laneWidth);
protected:
    std::vector<cv::Point> moveByGradient(int distance) const;
    std::vector<cv::Point> calcGradient(const std::vector<cv::Point>& points) const;
    virtual cv::Point calcPerpendicular(const cv::Point& point) const = 0;
    void track();
    virtual void detect();
    virtual void printDetectCount() const = 0;
    virtual void showLinePoints(cv::Mat& drawImage) const = 0;

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
    const size_t minPointTrack = 70;
    const size_t minPointDetect = 10;
    const size_t beginPointIndex = 40;
    const size_t drivePointIndex = beginPointIndex + 20;

    int offsetX = 0;
    int width = 160;
    int count_detect;
};

class LeftLane : public LaneLine
{
public:
    virtual cv::Scalar getLaneColor() const override;
    virtual cv::Rect getDetectBeginPointRegion() const override;
    virtual void printDetectCount() const;
    virtual cv::Point calcPerpendicular(const cv::Point& point) const override;
protected:
    virtual void showLinePoints(cv::Mat& drawImage) const override;
};

class RightLane : public LaneLine
{
public:
    virtual cv::Scalar getLaneColor() const override;
    virtual cv::Rect getDetectBeginPointRegion() const override;
    virtual void printDetectCount() const;
    virtual cv::Point calcPerpendicular(const cv::Point& point) const override;
protected:
    virtual void showLinePoints(cv::Mat& drawImage) const override;
private:
};

// class MidLane : public LaneLine
// {
// public:
//     MidLane(LaneLine& leftLane, LaneLine& rightLane);
//     virtual cv::Scalar getLaneColor() const override;

// protected:
//     virtual void detect() override;
//     void detectIfHaveBothLanes();


// private:
//     size_t laneSize;
//     LaneLine& leftLane;
//     LaneLine& rightLane;
// };

#endif