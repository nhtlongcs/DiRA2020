#ifndef LANE_DETECT_H
#define LANE_DETECT_H

#include <memory>
#include "opencv2/core.hpp"
#include <dynamic_reconfigure/server.h>
#include <image_transport/publisher.h>
#include <image_transport/image_transport.h>
#include "lane_detect/LaneConfig.h"

class LaneLine;

class LaneDetect
{
public:
    LaneDetect();
    ~LaneDetect();

    void update();

    void detect(int turningDirect);
    void show(const cv::Point &carPos, const cv::Point *drivePoint = nullptr) const;
    int whichLane(const cv::Mat &objectMask) const;

    bool isAbleToTurn(cv::Mat depth) const;

    int getLaneWidth() const;
    std::shared_ptr<LaneLine> getLeftLane() const;
    std::shared_ptr<LaneLine> getRightLane() const;

    cv::Mat birdviewTransform(cv::Mat inputImage, cv::Mat &resultM) const;
    cv::Mat extractFeatureY(cv::Mat img) const;

public:
    void configlaneCallback(lane_detect::LaneConfig &config, uint32_t level);
    void updateBinaryCallback(const sensor_msgs::ImageConstPtr &msg);
    void updateRGBCallback(const sensor_msgs::ImageConstPtr &msg);

private:
    bool isWrongLane() const;
    cv::Mat preprocess(const cv::Mat &src);
    cv::Mat shadow(const cv::Mat &src);
    cv::Mat morphological(const cv::Mat &img);
    cv::Mat ROI(const cv::Mat &src);

    void drawLine(float slope, float yintercept, cv::Mat &HoughTransform);
    cv::Point Hough(const cv::Mat &img, const cv::Mat &src);

    bool isNeedRedetect(cv::Point leftBegin, cv::Point rightBegin) const;
    std::shared_ptr<LaneLine> leftLane;
    std::shared_ptr<LaneLine> rightLane;

    cv::Mat binary;
    cv::Mat rgb;
    cv::Mat debug;
    cv::Mat birdview;
    cv::Mat birdviewTransformMatrix;

    int roadInside_min[3] = {0, 0, 0};
    int roadInside_max[3] = {179, 40, 140};

    int minThreshold[3] = {0, 0, 180};
    int maxThreshold[3] = {179, 30, 255};

    int minLaneInShadow[3] = {90, 35, 95};
    int maxLaneInShadow[3] = {180, 117, 158};

    int initLaneWidth = 50;
    int dropTop = 80;

    int lowThreshold = 0;
    int highThreshold = 155;
    int votes = 60;
    int minLinlength = 60;
    int maxLineGap = 5;
    size_t sumLaneWidth = 0;
    size_t frameCount = 0;

    bool usebirdview = true;
    bool showDetectRegion = true;

    int offsetLeft = 100;
    int offsetRight = 100;

    int offsetX = 160;
    int offsetY = 180;
    int birdwidth = 320;
    int birdheight = 240;
    int skyline = 120;

private:
    ros::NodeHandle _nh;
    image_transport::Subscriber _binaryImageSub;
    image_transport::ImageTransport _binaryImageTransport;

    dynamic_reconfigure::Server<lane_detect::LaneConfig> _configServer;

    image_transport::ImageTransport _debugImage;
    image_transport::Publisher _birdviewPublisher;
    image_transport::Publisher _lanePublisher;
    image_transport::Publisher _houghPublisher;
};
#endif
