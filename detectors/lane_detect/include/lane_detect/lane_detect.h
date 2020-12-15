#ifndef LANE_DETECT_H
#define LANE_DETECT_H

#include <memory>
#include "opencv2/core.hpp"
#include <dynamic_reconfigure/server.h>
#include <image_transport/publisher.h>
#include <image_transport/image_transport.h>
#include "lane_detect/LaneConfig.h"
#include "lane_detect/laneline.h"
#include "cds_msgs/IsTurnable.h"
#include "cds_msgs/ResetLane.h"
#include "cds_msgs/RecoverLane.h"


class LaneDetect
{
public:
    LaneDetect(bool isDebug);
    ~LaneDetect();
    void detect();
    void redetect();

public:
    void configCallback(lane_detect::LaneConfig &config, uint32_t level);
    void updateDepthCallback(const sensor_msgs::ImageConstPtr& msg);
    void updateLaneSegCallback(const sensor_msgs::ImageConstPtr &msg);
    void updateRoadSegCallback(const sensor_msgs::ImageConstPtr &msg);

    bool isTurnable(cds_msgs::IsTurnableRequest& req, cds_msgs::IsTurnableResponse& res);
    bool resetLaneSrv(cds_msgs::ResetLaneRequest& req, cds_msgs::ResetLaneResponse& res);
    bool recoverLaneSrv(cds_msgs::RecoverLaneRequest& req, cds_msgs::RecoverLaneResponse& res);

private:
    void publishMessage() const;

    cv::Mat birdviewTransform(cv::Mat inputImage, cv::Mat &resultM) const;
    cv::Mat extractFeatureY(cv::Mat img) const;
    cv::Mat preprocess(const cv::Mat &src);
    cv::Mat shadow(const cv::Mat &src);
    cv::Mat morphological(const cv::Mat &img);
    cv::Mat ROI(const cv::Mat &src);

    void drawLine(float slope, float yintercept, cv::Mat &HoughTransform);
    cv::Point Hough(const cv::Mat &img, const cv::Mat &src);

    bool isNeedRedetect(const LaneLine& left, const LaneLine& right) const;
    bool isCorrect(LaneLine* lane, const cv::Mat& roadSeg, int direct) const;

    LeftLane left;
    RightLane right;

    cv::Mat binary;
    cv::Mat birdview;
    cv::Mat birdviewTransformMatrix;
    cv::Mat debugImage;
    cv::Mat depth;
    cv::Mat roadSeg;

    int roadInside_min[3] = {0, 0, 0};
    int roadInside_max[3] = {179, 40, 140};

    int minThreshold[3] = {0, 0, 180};
    int maxThreshold[3] = {179, 30, 255};

    int minLaneInShadow[3] = {90, 35, 95};
    int maxLaneInShadow[3] = {180, 117, 158};

    bool isDebug {false};
    int countRedetectLane = 0;
    int maxCountRedetectLane = 10;

    int initLaneWidth = 50;
    int dropTop = 80;

    int lowThreshold = 0;
    int highThreshold = 155;
    int votes = 60;
    int minLinlength = 60;
    int maxLineGap = 5;

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
    ros::Publisher _lanePub;
    ros::ServiceServer _isTurnableSrv;
    ros::ServiceServer _resetLaneSrv;
    ros::ServiceServer _recoverLaneSrv;

    image_transport::ImageTransport _image_transport;
    image_transport::Subscriber _laneSegSub;
    image_transport::Subscriber _roadSegSub;
    image_transport::Subscriber _depthImageSub;

    dynamic_reconfigure::Server<lane_detect::LaneConfig> _configServer;
};
#endif
