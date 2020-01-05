#include "detectlane.h"
#include "laneline.h"
#include "utils.h"
#include "lane_detect/laneConfig.h"
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

constexpr const char *CONF_BIRDVIEW_WINDOW = "Birdview";

DetectLane::DetectLane()
    : leftLane{nullptr}, rightLane{nullptr}, frameCount{0}, sumLaneWidth{0}, _nh{"lanedetect"}, _configServer{_nh}, _debugImage{_nh}
// , midLane{nullptr}
{
    leftLane = std::make_shared<LeftLane>();
    rightLane = std::make_shared<RightLane>();

    _configServer.setCallback(boost::bind(&DetectLane::configlaneCallback, this, _1, _2));

    _birdviewPublisher = _debugImage.advertise("/debug/lane/birdview", 1, false);
    _lanePublisher = _debugImage.advertise("/debug/lane/lane", 1, false);
    _houghPublisher = _debugImage.advertise("/debug/lane/hough", 1, false);
    // setUseOptimized(true);
    // setNumThreads(4);

    // cvCreateTrackbar("Hough", "Threshold", &hough_lowerbound, max_houghThreshold);
    // cvCreateTrackbar("Hough_minLinlength", "Threshold", &minLinlength, 150);

    cv::namedWindow("Threshold");
    cv::createTrackbar( "Min Threshold:", "Threshold", &highThreshold, 255);
    cv::createTrackbar( "Max Threshold:", "Threshold", &lowThreshold, 255);

    // cv::namedWindow("Threshold");
    // cv::createTrackbar("MinShadow H", "Threshold", &minLaneInShadow[0], 255);
    // cv::createTrackbar("MinShadow S", "Threshold", &minLaneInShadow[1], 255);
    // cv::createTrackbar("MinShadow V", "Threshold", &minLaneInShadow[2], 255);
    // cv::createTrackbar("MaxShadow H", "Threshold", &maxLaneInShadow[0], 255);
    // cv::createTrackbar("MaxShadow S", "Threshold", &maxLaneInShadow[1], 255);
    // cv::createTrackbar("MaxShadow V", "Threshold", &maxLaneInShadow[2], 255);
    // cv::createTrackbar( "Min Threshold:", "Threshold", &lowThreshold, 15);

    // cv::namedWindow(CONF_BIRDVIEW_WINDOW);
    // cv::createTrackbar("Use Birdview", CONF_BIRDVIEW_WINDOW, &usebirdview, 1);
    // cv::createTrackbar("InitLanewidth", CONF_BIRDVIEW_WINDOW, &initLaneWidth, 200);
    // cv::createTrackbar("Birdwidth", CONF_BIRDVIEW_WINDOW, &birdwidth, 400);
    // cv::createTrackbar("Birdheight", CONF_BIRDVIEW_WINDOW, &birdheight, 400);
    // cv::createTrackbar("Skyline", CONF_BIRDVIEW_WINDOW, &skyline, 200);
    // cv::createTrackbar("OffsetLeft", CONF_BIRDVIEW_WINDOW, &offsetLeft, 200);
    // cv::createTrackbar("OffsetRight", CONF_BIRDVIEW_WINDOW, &offsetRight, 200);

    // midLane = new MidLane(*leftLane, *rightLane);
}

DetectLane::~DetectLane()
{
}

void DetectLane::configlaneCallback(lane_detect::laneConfig &config, uint32_t level)
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
    leftLane->setFindBeginPointRegion(config.offset_left, config.left_width);
    rightLane->setFindBeginPointRegion(config.offset_right, config.right_width);
}

void DetectLane::updateBinary(const cv::Mat &src)
{
    this->binary = src.clone();
}

void DetectLane::updateRGB(const cv::Mat &src)
{
    this->rgb = src.clone();
}

bool DetectLane::isNeedRedetect(cv::Point leftBegin, cv::Point rightBegin) const
{
    if (leftLane->isFound() && rightLane->isFound())
    {
        std::vector<int> diff;
        const auto& leftParams = leftLane->getLineParams();
        const auto& rightParams = rightLane->getLineParams();
        for (size_t y = 0; y < birdview.rows; y++)
        {
            int xleft = getXByY(leftParams, y * 1.0);
            int xright = getXByY(rightParams, y * 1.0);
            diff.push_back(abs(xleft - xright));
        }
        return std::any_of(diff.begin(), diff.end(), [this](const int& amount) {
            return amount < this->initLaneWidth;
        });
    }
    return false;
}

void DetectLane::detect(int turningDirect)
{
    if (this->rgb.empty())
    {
        return;
    }

    if (this->binary.empty())
    {
        return;
    }

    // showImage("RGB", this->rgb);

    // Mat binary = preprocess(this->rgb);

    // Mat shadowMask = shadow(this->rgb);
    // bitwise_or(binary, shadowMask, binary);
    // showImage("binary", this->binary);

    if (usebirdview)
    {
        // showImage("binary", this->binary);
        this->birdview = birdviewTransformation(this->binary, birdwidth, birdheight, skyline, offsetLeft, offsetRight, birdviewTransformMatrix);

        // showImage(CONF_BIRDVIEW_WINDOW, this->birdview);
    }
    else
    {
        birdview = this->binary;
    }

    // birdview = this->binary;

    // Mat morphBirdview = morphological(birdview);

    birdview(cv::Rect(0, 0, birdview.cols, dropTop)) = cv::Scalar{0};
    showImage(_birdviewPublisher, "mono8", birdview);

    leftLane->update(birdview);
    rightLane->update(birdview);

    if (leftLane->isFound() && rightLane->isFound())
    {
        cv::Point leftBegin, rightBegin;
        leftLane->getBeginPoint(leftBegin);
        rightLane->getBeginPoint(rightBegin);

        if (leftBegin.x > 160)
        {
            ROS_INFO("Invalid left");
            leftLane->reset();
        }

        if (rightBegin.x < 100)
        {
            ROS_INFO("Invalid right");
            rightLane->reset();
        }

        if (isNeedRedetect(leftBegin, rightBegin))
        {
            // ROS_INFO("LaneWidth < %d. Redetect", initLaneWidth);
            if (turningDirect == 1)
            {
                leftLane->reset();
            } else if (turningDirect == -1)
            {
                rightLane->reset();
            } else {
                leftLane->reset();
                rightLane->reset();
            }
            frameCount = 0;
            sumLaneWidth = 0;
        }
        else
        {
            frameCount++;
            sumLaneWidth += abs(leftBegin.x - rightBegin.x);
        }
    } else if (leftLane->isFound())
    {
        ROS_INFO_ONCE("\033[1;31mLoss right lane\033[0m");

    } else if (rightLane->isFound())
    {
        ROS_INFO_ONCE("\033[1;32mLoss left lane\033[0m");
        
    }
}
cv::Mat DetectLane::extractFeatureY(cv::Mat img) const
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
bool DetectLane::isAbleToTurn(cv::Mat depth) const
{
    if (this->binary.empty())
    {
        return false;
    }

    // cv::imshow("binary", binary);

    cv::Mat nearRegionMask;
    cv::inRange(depth, cv::Scalar{lowThreshold}, cv::Scalar{highThreshold}, nearRegionMask);
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
        if (fabs(slope)< 0.1) cnt++;
    }
    // std::cout << cnt << std::endl;
    if (cnt > 4) 
        return 1;

    return 0;
}

void DetectLane::show(const cv::Point &carPos, const cv::Point *drivePoint) const
{
    if (birdview.empty())
    {
        return;
    }
    cv::Mat birdviewColor;
    cv::cvtColor(birdview, birdviewColor, cv::COLOR_GRAY2BGR);
    leftLane->show(birdviewColor, showDetectRegion);
    rightLane->show(birdviewColor, showDetectRegion);

    if (drivePoint)
    {
        cv::circle(birdviewColor, *drivePoint, 25, cv::Scalar{0, 255, 255}, -1);
    }

    cv::circle(birdviewColor, carPos, 25, cv::Scalar{0, 0, 255}, -1);

    // showImage("Lanes", birdviewColor);
    showImage(_lanePublisher, "bgr8", birdviewColor);
}

int DetectLane::whichLane(const cv::Mat &objectMask) const
{
    if (this->binary.empty())
    {
        return 0;
    }

    if (usebirdview)
    {
        cv::Mat M;
        cv::Mat birdviewObjectMask = birdviewTransformation(objectMask, birdwidth, birdheight, offsetLeft, offsetRight, skyline, M);

        cv::Mat black = cv::Mat::zeros(birdview.rows, birdview.cols, CV_8UC1);
        this->getLeftLane()->getMask(black);
        this->getRightLane()->getMask(black);

        // cv::imshow("ObjectMask", objectMask);

        black |= birdviewObjectMask;
        // cv::imshow("TwoLaneMask", black);

        // std::vector<cv::Point2f> points = {
        //     cv::Point2f{boundingBox.tl()},
        //     cv::Point2f{boundingBox.br().x, boundingBox.tl().y},
        //     cv::Point2f{boundingBox.br()},
        //     cv::Point2f{boundingBox.tl().x, boundingBox.br().y}
        // };
        // std::vector<cv::Point2f> birdviewBoundingBox;
        // cv::perspectiveTransform(points, birdviewBoundingBox, birdviewTransformMatrix);

        // cv::Mat colorBirdview;
        // cv::cvtColor(this->birdview, colorBirdview, cv::COLOR_GRAY2BGR);

        // for (int i = 0; i < 3; i++)
        //     cv::line(colorBirdview, birdviewBoundingBox[i], birdviewBoundingBox[i+1], cv::Scalar{255,255,0});
        // cv::line(colorBirdview, birdviewBoundingBox[0], birdviewBoundingBox[3], cv::Scalar{255,255,0});

        // cv::imshow("BoundingBoxBirdview", colorBirdview);

        return 0;
    }

    // I dont know
    return 0;
}

cv::Mat DetectLane::birdviewTransform(cv::Mat inputImage, cv::Mat &resultM) const
{
    cv::Mat birdviewResult = birdviewTransformation(inputImage, birdwidth, birdheight, skyline, offsetLeft, offsetRight, resultM);
    return birdviewResult;
}

Mat DetectLane::shadow(const Mat &src)
{
    Mat shadow, hsv;
    cvtColor(src, hsv, cv::COLOR_BGR2HSV);
    inRange(hsv, Scalar(minLaneInShadow[0], minLaneInShadow[1], minLaneInShadow[2]),
            Scalar(maxLaneInShadow[1], maxLaneInShadow[1], maxLaneInShadow[2]), shadow);
    return shadow;
}

Point DetectLane::Hough(const Mat &img, const Mat &src)
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

    // showImage("HoughLines", HoughTransform);
    showImage(_houghPublisher, "bgr8", HoughTransform);

    return Point(midX, midY);
}

void DetectLane::drawLine(float slope, float y_intercept, Mat &HoughTransform)
{
    float y0 = 240.0;
    float y1 = 1.0;
    float x0 = (y0 - y_intercept) / slope;
    float x1 = (y1 - y_intercept) / slope;
    line(HoughTransform, Point(x0, y0), Point(x1, y1), Scalar(0, 255, 0), 2, LINE_AA);
}

Mat DetectLane::ROI(const Mat &src)
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

Mat DetectLane::morphological(const Mat &img)
{
    Mat dst;

    dilate(img, dst, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

    return dst;
}

Mat DetectLane::preprocess(const Mat &src)
{
    Mat hsv, binary;

    cvtColor(src, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(minThreshold[0], minThreshold[1], minThreshold[2]),
            Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]), binary);

    return binary;
}

int DetectLane::getLaneWidth() const
{
    if (this->frameCount == 0)
    {
        return 0;
    }
    return this->sumLaneWidth / this->frameCount;
}

std::shared_ptr<LaneLine> DetectLane::getLeftLane() const
{
    return this->leftLane;
}

std::shared_ptr<LaneLine> DetectLane::getRightLane() const
{
    return this->rightLane;
}
