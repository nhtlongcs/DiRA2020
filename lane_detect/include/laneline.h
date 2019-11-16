#ifndef LANELINE_H
#define LANELINE_H
#include <array>
#include <memory>

#include <opencv2/opencv.hpp>
#include "utils.h"

#define LEFT 1
#define RIGHT 2

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
    virtual bool init() = 0;
    virtual int getType() const = 0;
    void show(cv::Mat& drawImage) const;
    
protected:
    virtual cv::Scalar getLaneColor() const = 0;
    bool isOutOfImage(cv::Point point) const;
    cv::Rect getROITracking(cv::Point centerPoint);
    bool findPointHasBiggestValueInBin(const cv::Mat& trackingImage, cv::Point& returnPoint, int& returnValue) const;
    
    void updateListPoint();
    void updateListPointUp();
    void updateListPointDown();

protected:
    cv::Mat debugImage;
    cv::Mat lineImage;
    cv::Point beginPoint;
    cv::Point endPoint;
    std::vector<cv::Point> listPoint;
    std::shared_ptr<LineParams> lineParams;
    bool isFound_, debug;
};

class LeftLane : public LaneLine
{
public:
    virtual bool init() override;
    virtual int getType() const override;
    virtual cv::Scalar getLaneColor() const override;

protected:
};

class RightLane : public LaneLine
{
public:
    virtual bool init() override;
    virtual int getType() const override;
    virtual cv::Scalar getLaneColor() const override;

private:
};

#endif