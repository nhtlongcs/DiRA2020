#ifndef LANELINE_H
#define LANELINE_H
#include <array>
#include <memory>

#include <opencv2/opencv.hpp>
#include <common/libcommon.h>

#define H_TRACKING 16
#define W_TRACKING 25
#define N_BINS 10

class LaneLine
{
public:
    LaneLine(cv::Mat& debugImage);

    void update(const cv::Mat &lineImage);
    bool isFound() const;
    bool isOutOfImage(const cv::Point &point) const;
    void setFindBeginPointRegion(int offset, int width);

    virtual void showLinePoints(cv::Mat &drawImage) const = 0;
    LineParams getLineParams() const;

    void swap(std::shared_ptr<LaneLine> other);

    virtual void reset();
    bool recoverFrom(const LaneLine &lane, int laneWidth);

    virtual std::string getName() const = 0;
    int getConfScore() const
    {
        return this->confident_score;
    }
    void visualizeTrackingWindow(cv::Mat& parent, cv::Point& center) const;
    virtual void visualizeTrackingPoint(cv::Mat& image, const std::vector<cv::Point>& points) = 0;
    void setLineParams(const LineParams& another);
protected:
    std::vector<cv::Point> moveByGradient(int distance) const;
    std::vector<cv::Point> calcGradient(const std::vector<cv::Point> &points) const;
    virtual cv::Point calcPerpendicular(const cv::Point &point) const = 0;
    void track();
    void detect();

    virtual cv::Rect getDetectBeginPointRegion() const;
    std::vector<cv::Point> getPointsFromParams(const std::shared_ptr<LineParams> params) const;

    virtual cv::Scalar getLaneColor() const = 0;

    bool updateNewCenter(const cv::Mat &region, cv::Point &center, int *const nonZeroValue) const;

    bool findBeginPoint(cv::Point &returnPoint) const;
    std::vector<cv::Point> findPoints(const cv::Point &beginPoint, int direct) const;

protected:
    cv::Mat& debugImage;
    cv::Mat lineImage;
    cv::Rect lineImageRect;
    std::shared_ptr<LineParams> lineParams;

    // const size_t maxNonZeroThreshold = W_TRACKING / N_BINS * H_TRACKING;
    const size_t maxNonZeroThreshold = 10;
    const size_t minPointTrack = 10;
    const size_t minPointDetect = 5;
    int confident_score=-1;
    int offsetX = 0;
    int width = 160;
};

class LeftLane : public LaneLine
{
public:
    LeftLane(cv::Mat& debugImage);
    virtual cv::Scalar getLaneColor() const override;
    virtual cv::Rect getDetectBeginPointRegion() const override;
    virtual cv::Point calcPerpendicular(const cv::Point &point) const override;
    virtual void showLinePoints(cv::Mat &drawImage) const override;
    virtual void visualizeTrackingPoint(cv::Mat& image, const std::vector<cv::Point>& points) override;
    virtual std::string getName() const override
    {
        return "Left";
    }

    // bool findBeginPoint(cv::Point &returnPoint) const;

    // bool updateNewCenter(const cv::Mat &region, cv::Point &center, int *const nonZeroValue) const;

};

class RightLane : public LaneLine
{
public:
    RightLane(cv::Mat& debugImage);
    virtual cv::Scalar getLaneColor() const override;
    virtual cv::Rect getDetectBeginPointRegion() const override;
    virtual cv::Point calcPerpendicular(const cv::Point &point) const override;
    virtual void showLinePoints(cv::Mat &drawImage) const override;
    virtual void visualizeTrackingPoint(cv::Mat& image, const std::vector<cv::Point>& points) override;

    virtual std::string getName() const override
    {
        return "Right";
    }
private:
};

#endif