#include <common/libcommon.h>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <geometry_msgs/Point32.h>

#include "cds_msgs/lane.h"
#include "lane_detect/lane_detect.h"
#include "lane_detect/laneline.h"
#include "lane_detect/LaneConfig.h"

using namespace cv;
using namespace std;

constexpr const char *CONF_BIRDVIEW_WINDOW = "Birdview";
constexpr const char *LANE_WINDOW = "LaneDetect";

LaneDetect::LaneDetect(bool isDebug)
    : frameCount{0}, sumLaneWidth{0}, _nh{"lane_detect"}, _image_transport{_nh}, _configServer{_nh}, isDebug{isDebug}
    , left{debugImage}, right{debugImage}
{
    _isTurnableSrv = _nh.advertiseService("IsTurnable", &LaneDetect::isTurnable, this);

    std::string lane_seg_topic, depth_topic, lane_output_topic, transport_hint_str;
    ROS_ASSERT(ros::param::get("/lane_segmentation_topic", lane_seg_topic));
    ROS_ASSERT(ros::param::get("/lane_detect_topic", lane_output_topic));
    ROS_ASSERT(ros::param::get("/depth_topic", depth_topic));
    ROS_ASSERT(ros::param::get("/transport_hint", transport_hint_str));

    _lanePub = _nh.advertise<cds_msgs::lane>(lane_output_topic, 1);
    image_transport::TransportHints transport_hint{transport_hint_str};
    _binaryImageSub = _image_transport.subscribe(lane_seg_topic, 1, &LaneDetect::updateBinaryCallback, this, transport_hint);
    _depthImageSub = _image_transport.subscribe(depth_topic, 1, &LaneDetect::updateDepthCallback, this, transport_hint);
    _configServer.setCallback(std::bind(&LaneDetect::configCallback, this, std::placeholders::_1, std::placeholders::_2));
}

LaneDetect::~LaneDetect()
{
    cv::destroyAllWindows();
}

void LaneDetect::configCallback(lane_detect::LaneConfig &config, uint32_t level)
{
    usebirdview = config.use_birdview;
    showDetectRegion = config.show_detect_region;
    dropTop = config.drop_top;
    initLaneWidth = config.init_lane_width;
    birdwidth = config.birdwidth;
    birdheight = config.birdheight;
    skyline = config.skyline;
    offsetLeft = config.offset_birdview_left;
    offsetRight = config.offset_birdview_right;
    left.setFindBeginPointRegion(config.offset_left, config.left_width);
    right.setFindBeginPointRegion(config.offset_right, config.right_width);
}

bool LaneDetect::isNeedRedetect(const LaneLine& left, const LaneLine& right) const
{
    if (left.isFound() && right.isFound())
    {
        std::vector<int> diff;
        const auto &leftParams = left.getLineParams();
        const auto &rightParams = right.getLineParams();
        for (size_t y = 0; y < birdview.rows; y++)
        {
            int xleft = getXByY(leftParams, y * 1.0);
            int xright = getXByY(rightParams, y * 1.0);
            diff.push_back(abs(xleft - xright));
        }
        return std::any_of(diff.begin(), diff.end(), [this](const int &amount) {
            return amount < this->initLaneWidth;
        });
    }
    return false;
}

void LaneDetect::detect()
{
    cv::imshow("Receive segment", this->binary);
    cv::waitKey(1);

    this->birdview = birdviewTransformation(this->binary, birdwidth, birdheight, skyline, offsetLeft, offsetRight, birdviewTransformMatrix);

    birdview(cv::Rect(0, 0, birdview.cols, dropTop)) = cv::Scalar{0};

    if (isDebug)
    {
        cv::cvtColor(this->birdview, debugImage, cv::COLOR_GRAY2BGR);
    }

    left.update(birdview);
    right.update(birdview);

    if (left.isFound() && right.isFound())
    {
        // TODO: reset if both 
        // std::cout << '\t' << "LeftConfScore: " << left.getConfScore() << " RightConfScore " << right.getConfScore() << std::endl;
        if (isNeedRedetect(left, right))
        {
            // std::cout << "Redetect!!!" << std::endl;

            {
                // Plan 1: Reset 1 lane
                if (left.getConfScore() < right.getConfScore())
                {
                    // std::cout << "Reset LEFT" << std::endl;
                    left.reset();
                    // left.update(birdview);
                    if (left.recoverFrom(right, 90))
                    {
                        countRedetectLane = 0;
                        // std::cout << "Recovered left from right!" <<  std::endl;
                    } else
                    {
                        countRedetectLane++;
                        // std::cout << "Cannot recover left from right!" <<  std::endl;
                    }
                } else if (right.getConfScore() < left.getConfScore())
                {
                    // std::cout << "Reset RIGHT" << std::endl;
                    right.reset();
                    // right.update(birdview);
                    if (right.recoverFrom(left, 90))
                    {
                        // std::cout << "Recovered right from left" << std::endl;
                        countRedetectLane = 0;
                    } else
                    {
                        countRedetectLane++;
                        // std::cout << "Cannot recover right from left" << std::endl;
                    }
                }
            }

            // std::cout << "Count Reset = " << countRedetectLane << std::endl;
            {
                if (countRedetectLane < maxCountRedetectLane)
                {
                    if (left.getConfScore() < right.getConfScore())
                    {
                        // std::cout << "Reset LEFT" << std::endl;
                        left.reset();
                    } else if (right.getConfScore() < left.getConfScore())
                    {
                        // std::cout << "Reset RIGHT" << std::endl;
                        right.reset();
                    }
                }
                else
                {
                    // std::cout << "Plan 2" << std::endl;
                    countRedetectLane = 0; // reset counting
                    // Plan 2: Reset 2 lanes
                    cv::Mat newLaneDebug;
                    cv::cvtColor(this->birdview, newLaneDebug, cv::COLOR_GRAY2BGR);

                    LeftLane newLeft{newLaneDebug};
                    RightLane newRight{newLaneDebug};

                    newLeft.update(birdview);
                    newRight.update(birdview);
                    if (newLeft.isFound() && newRight.isFound())
                    {
                        if (isNeedRedetect(newLeft, newRight))
                        {
                            // std::cout << "Plan 2 still need redetect!!!" << std::endl;
                            if (left.getConfScore() < right.getConfScore())
                            {
                                // std::cout << "Reset LEFT" << std::endl;
                                left.reset();
                            } else if (right.getConfScore() < left.getConfScore())
                            {
                                // std::cout << "Reset RIGHT" << std::endl;
                                right.reset();
                            }
                        } else
                        {
                            // std::cout << "Use new lanes" << std::endl;
                            left.setLineParams(newLeft.getLineParams());
                            right.setLineParams(newRight.getLineParams());
                        }
                    } else if (newLeft.isFound())
                    {
                        left.setLineParams(newLeft.getLineParams());
                        right.reset();
                    } else if (newRight.isFound())
                    {
                        left.reset();
                        right.setLineParams(newRight.getLineParams());
                    } else
                    {
                        left.reset();
                        right.reset();
                        // std::cout << "BOTH LANE NOT FOUND!" << std::endl;
                    }
                }
            }

        }
    }

    if (isDebug)
    {
        if (left.isFound())
        {
            left.showLinePoints(debugImage);
        }
        if (right.isFound())
        {
            right.showLinePoints(debugImage);
        }
        cv::imshow(LANE_WINDOW, debugImage);
        cv::waitKey(1);
        // openCVVisualize();
    }
}

cv::Mat LaneDetect::extractFeatureY(cv::Mat img) const
{
    cv::Mat binaryTemp = img.clone();

    cv::Mat skel(binaryTemp.size(), CV_8UC1, cv::Scalar(0));
    {
        cv::Mat temp;
        cv::Mat eroded;

        cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

        bool done;
        do
        {
            cv::erode(binaryTemp, eroded, element);
            cv::dilate(eroded, temp, element); // temp = open(binaryTemp)
            cv::subtract(binaryTemp, temp, temp);
            cv::bitwise_or(skel, temp, skel);
            eroded.copyTo(binaryTemp);

            done = (cv::countNonZero(binaryTemp) == 0);
        } while (!done);
    }
    return skel;
}
bool LaneDetect::isTurnable(cds_msgs::IsTurnableRequest& req, cds_msgs::IsTurnableResponse& res)
{
    if (this->binary.empty())
    {
        return false;
    }

    ROS_DEBUG("Service IsTurnable is called");

    // cv::imshow("binary", binary);

    cv::Mat nearRegionMask;
    cv::inRange(depth, cv::Scalar{lowThreshold * 1.0}, cv::Scalar{highThreshold * 1.0}, nearRegionMask);
    // cv::imshow("nearRegionMask", nearRegionMask);

    cv::Mat binaryNearRegion = binary & nearRegionMask;
    // cv::imshow("binaryNearRegion", binaryNearRegion);

    // cv::Mat sobely;
    // cv::Mat skel = extractFeatureY(roadSegmentation);
    // cv::imshow("Sobely", skel);

    // cv::Sobel(skel, sobely, CV_8U, 0, 1);
    // cv::imshow("Sobely", binary);
    int cnt = 0;
    std::vector<cv::Vec4f> lines;
    cv::HoughLinesP(binaryNearRegion, lines, 1, CV_PI / 180, votes, minLinlength, maxLineGap);
    // cv::HoughLinesP(skel, lines, 1, CV_PI / 180, votes, minLinlength, maxLineGap);

    float Slope_average = 0;
    for (const cv::Vec4f &line : lines)
    {
        float slope = 0;
        if (abs(line[0] - line[2]) < 0.01f)
        {
            // vertical line, tan = inf
            slope = 0;
            continue;
        }
        else
        {
            slope = (line[1] - line[3]) / (line[0] - line[2]);
        }

        // std::cout << slope << std::endl;
        if (fabs(slope) < 0.1)
            cnt++;
    }
    // std::cout << cnt << std::endl;
    if (cnt > 4)
    {
        res.ok = true;
    }
    else
    {
        res.ok = false;
    }
    return true;
}

cv::Mat LaneDetect::birdviewTransform(cv::Mat inputImage, cv::Mat &resultM) const
{
    cv::Mat birdviewResult = birdviewTransformation(inputImage, birdwidth, birdheight, skyline, offsetLeft, offsetRight, resultM);
    return birdviewResult;
}

Mat LaneDetect::shadow(const Mat &src)
{
    Mat shadow, hsv;
    cvtColor(src, hsv, cv::COLOR_BGR2HSV);
    inRange(hsv, Scalar(minLaneInShadow[0], minLaneInShadow[1], minLaneInShadow[2]),
            Scalar(maxLaneInShadow[1], maxLaneInShadow[1], maxLaneInShadow[2]), shadow);
    return shadow;
}

Point LaneDetect::Hough(const Mat &img, const Mat &src)
{
    vector<Vec4f> lines;
    Mat HoughTransform = Mat::zeros(src.size(), CV_8UC3);
    float cntL = 0.0, cntR = 0.0;
    float leftSlope_average = 0.0;
    float rightSlope_average = 0.0;
    float leftYintercept_average = 0.0;
    float rightYintercept_average = 0.0;

    HoughLinesP(img, lines, 1, CV_PI / 180, votes, minLinlength, maxLineGap);

    for (int i = 0; i < (int)lines.size(); ++i)
    {
        Vec4f t = lines[i];
        float slope = (t[1] - t[3]) / (t[0] - t[2]);
        float Yintercept = t[1] - slope * t[0];

        if (slope < 0.0)
        {
            leftSlope_average += slope, leftYintercept_average += Yintercept, cntL += 1.0;
        }
        else
        {
            rightSlope_average += slope, rightYintercept_average += Yintercept, cntR += 1.0;
        }
    }

    if (cntL > 0)
    {
        leftSlope_average /= cntL;
        leftYintercept_average /= cntL;
        drawLine(leftSlope_average, leftYintercept_average, HoughTransform);
    }

    if (cntR > 0)
    {
        rightSlope_average /= cntR;
        rightYintercept_average /= cntR;
        drawLine(rightSlope_average, rightYintercept_average, HoughTransform);
    }

    float midY = offsetY;
    float midX = offsetX;

    if (fabs(rightSlope_average) < 0.05)
        cntR = 0;
    if (fabs(leftSlope_average) < 0.05)
        cntL = 0;

    if (cntL == 0.0 && cntR > 0)
    {
        midX = (midY - rightYintercept_average) / rightSlope_average;
    }
    else if (cntL > 0 && cntR == 0.0)
    {
        midX = (midY - leftYintercept_average) / leftSlope_average;
    }
    else if (cntL > 0 && cntR > 0)
    {
        midX = (((midY - leftYintercept_average) / leftSlope_average) + ((midY - rightYintercept_average) / rightSlope_average)) / 2.0;
    }

    addWeighted(src, 0.5, HoughTransform, 1, 1, HoughTransform);
    circle(HoughTransform, Point(midX, midY), 3, Scalar(0, 0, 255), -1);
    circle(HoughTransform, Point(offsetX, offsetY), 3, Scalar(0, 255, 0), -1);

    return Point(midX, midY);
}

void LaneDetect::drawLine(float slope, float y_intercept, Mat &HoughTransform)
{
    float y0 = 240.0;
    float y1 = 1.0;
    float x0 = (y0 - y_intercept) / slope;
    float x1 = (y1 - y_intercept) / slope;
    line(HoughTransform, Point(x0, y0), Point(x1, y1), Scalar(0, 255, 0), 2, LINE_AA);
}

Mat LaneDetect::ROI(const Mat &src)
{
    //Crop region of interest (ROI)
    int W = src.size().width;
    int H = src.size().height;
    Mat RegionOfInterest;
    Mat mask = Mat::zeros(src.size(), CV_8UC1);

    vector<Point> vertices;
    vertices.push_back(Point(0, skyline));
    vertices.push_back(Point(W, skyline));
    vertices.push_back(Point(W, H));
    vertices.push_back(Point(0, H));

    fillConvexPoly(mask, vertices, Scalar(255, 255, 255), 8, 0);
    bitwise_and(src, mask, RegionOfInterest);

    //imshow("RegionOfInterest", RegionOfInterest);
    return RegionOfInterest;
}

Mat LaneDetect::morphological(const Mat &img)
{
    Mat dst;

    dilate(img, dst, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

    return dst;
}

Mat LaneDetect::preprocess(const Mat &src)
{
    Mat hsv, binary;

    cvtColor(src, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(minThreshold[0], minThreshold[1], minThreshold[2]),
            Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]), binary);

    return binary;
}

void LaneDetect::updateBinaryCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        if (!cv_ptr->image.empty())
        {
            this->binary = cv_ptr->image.clone();
            detect();
            publishMessage();
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }
}

void LaneDetect::updateDepthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        if (!cv_ptr->image.empty())
        {
            this->depth = cv_ptr->image.clone();
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }
}

void LaneDetect::publishMessage() const
{
    cds_msgs::lane lane_msg;
    if (left.isFound())
    {
        const auto &params = left.getLineParams();
        lane_msg.left_params.insert(lane_msg.left_params.begin(), params.begin(), params.end());
    }
    if (right.isFound())
    {
        const auto &params = right.getLineParams();
        lane_msg.right_params.insert(lane_msg.right_params.begin(), params.begin(), params.end());
    }
    _lanePub.publish(lane_msg);
}