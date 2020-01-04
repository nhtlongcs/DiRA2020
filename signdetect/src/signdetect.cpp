#include "signdetect.h"
#include <dynamic_reconfigure/server.h>
#include "signdetect/signConfig.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <string>
#include <exception>
#include <ros/ros.h>
#include "utils.h"
using namespace cv;
using namespace std;

#define SIZE_X 64.0

constexpr const char* CONF_SIGN_WINDOW = "SignDetect";

static double matching(cv::Mat image, cv::Mat templ)
{
    Mat img_display;
    image.copyTo(img_display);

    int result_rows = image.rows - templ.rows + 1;
    int result_cols = image.cols - templ.cols + 1;
    cv::Mat result(result_rows, result_cols, CV_32FC1);

    int match_method = CV_TM_SQDIFF;
    matchTemplate(image, templ, result, match_method);
    //   normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

    /// Localizing the best match with minMaxLoc
    double minVal;
    double maxVal;
    Point minLoc;
    Point maxLoc;

    Point matchLoc;

    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
    return minVal;
}

DetectSign::DetectSign(const cv::Mat& leftTemplate, const cv::Mat& rightTemplate)
    : recentDetects{MAX_FRAME_COUNT, 0}
    , LEFT_TEMPLATE{leftTemplate}
    , RIGHT_TEMPLATE{rightTemplate}
    , _nh{"signdetect"}
    , _debugImage{_nh}
{
    _roiPublisher = _debugImage.advertise("/debug/sign/ROI", 1, false);
    _thresholdedPublisher = _debugImage.advertise("/debug/sign/blue", 1, false);
    _detectPublisher = _debugImage.advertise("/debug/sign/detect", 1, false);
    // cv::namedWindow(CONF_SIGN_WINDOW);
    // cv::createTrackbar("canny", CONF_SIGN_WINDOW, &canny, 255);
    // cv::createTrackbar("votes", CONF_SIGN_WINDOW, &votes, 255);

    // cv::createTrackbar("Strategy", CONF_SIGN_WINDOW, &classifyStrategy, 1);
    // cv::createTrackbar("DetectConfident", CONF_SIGN_WINDOW, &detectConfident, 100);
    // cv::createTrackbar("ClassifyConfident", CONF_SIGN_WINDOW, &classifyConfident, 100);
    // cv::createTrackbar("DiffToClassify", CONF_SIGN_WINDOW, &diffToClassify, 100);

    // cv::createTrackbar("MinBlue H", CONF_SIGN_WINDOW, &minBlue[0], 179);
    // cv::createTrackbar("MinBlue S", CONF_SIGN_WINDOW, &minBlue[1], 255);
    // cv::createTrackbar("MinBlue V", CONF_SIGN_WINDOW, &minBlue[2], 255);
    // cv::createTrackbar("MaxBlue H", CONF_SIGN_WINDOW, &maxBlue[0], 179);
    // cv::createTrackbar("MaxBlue S", CONF_SIGN_WINDOW, &maxBlue[1], 255);
    // cv::createTrackbar("MaxBlue V", CONF_SIGN_WINDOW, &maxBlue[2], 255);

    MAX_DIFF = matching(LEFT_TEMPLATE, cv::Scalar(255) - LEFT_TEMPLATE);
}

void DetectSign::configCallback(signdetect::signConfig &config, uint32_t level)
{
    // ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
    //         config.int_param, config.double_param, 
    //         config.str_param.c_str(), 
    //         config.bool_param?"True":"False", 
    //         config.size);

    classifyStrategy = config.classify_method;
    detectConfident = config.detect_confident;
    diffToClassify = config.diff_classify;
    classifyConfident = config.classify_confident;
    minBlue[0] = config.min_H;
    minBlue[1] = config.min_S;
    minBlue[2] = config.min_V;
    maxBlue[0] = config.max_H;
    maxBlue[1] = config.max_S;
    maxBlue[2] = config.max_V;
}


DetectSign::~DetectSign()
{
    cv::destroyAllWindows();
}

void DetectSign::updateRGB(const cv::Mat &rgb)
{
    this->rgb = rgb.clone();
}

void DetectSign::updateDepth(const cv::Mat &depth)
{
    this->depth = depth.clone();
}

int DetectSign::detectOneFrame()
{
    if (this->depth.empty())
    {
        return 0;
    }

    Mat blue;
    cvtColor(rgb, blue, cv::COLOR_BGR2HSV);
    inRange(blue, Scalar(minBlue[0], minBlue[1], minBlue[2]), Scalar(maxBlue[0], maxBlue[1], maxBlue[2]), blue);

    int turnFactor = 0;

    // new method
    
    Mat gray;
    gray = kmean(this->depth, 2);

    // cvtColor(gray, gray, CV_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0, 0);

    vector<Vec3f> circles;

    HoughCircles(gray, circles, CV_HOUGH_GRADIENT,
                 1,       // accumulator resolution (size of the image / 2)
                 300,     // minimum distance between two circles
                 canny,   // Canny high threshold
                 votes,   // minimum number of votes
                 0, 100); // min and max radius
    for (size_t i = 0; i < circles.size(); i++)
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle(this->depth, center, 3, Scalar(127), -1, 8, 0);
        // circle outline
        circle(this->depth, center, radius, Scalar(100), 3, 8, 0);
        cv::Rect roi(center.x - radius, center.y - radius, radius * 2, radius * 2);

        if ((roi & cv::Rect(0, 0, gray.cols, gray.rows)) == roi)
        {
            cv::Mat roiImg(blue, roi);
            float percent = cv::countNonZero(roiImg) * 100.0f / (roiImg.rows * roiImg.cols);

            // showImage("DetectSign", depth);
            // showImage("DetectSignROI", roiImg);
            showImage(_detectPublisher, "mono8", depth);

            cv::Mat colorBlue;
            cv::cvtColor(roiImg, colorBlue, cv::COLOR_GRAY2BGR);
            cv::Mat thresholdconcat;
            cv::hconcat(rgb(roi), colorBlue, thresholdconcat);
            showImage(_roiPublisher, "bgr8", thresholdconcat);
            ROS_INFO("percent = %.2f", percent);
            if (percent > detectConfident)
            {
                int sign = classify(rgb(roi));
                if (sign != 0)
                {
                    return sign;
                }
            }
        }
    }

    return 0;
}

int DetectSign::classify(const cv::Mat& colorROI) const
{
    switch (classifyStrategy)
    {
        case 0: return classifyCountBlue(colorROI);
        case 1: return classifyTemplateMatching(colorROI);
        default:
            return classifyCountBlue(colorROI);
    }
}

int DetectSign::classifyTemplateMatching(const cv::Mat& colorROI) const
{
    cv::Mat gray;
    cv::cvtColor(colorROI, gray, cv::COLOR_BGR2GRAY);

    // TODO: may be threshold?
    // ...

    cv::Mat resized;
    cv::resize(gray, resized, cv::Size(), SIZE_X / colorROI.cols, SIZE_X / colorROI.rows);
    cv::GaussianBlur(resized, resized, cv::Size(5, 5), 0, 0);

    double left_val = matching(resized, LEFT_TEMPLATE);
    double left_percent = (1.0 - left_val / MAX_DIFF) * 100;
    double right_val = matching(resized, RIGHT_TEMPLATE);
    double right_percent = (1.0 - right_val / MAX_DIFF) * 100;

    ROS_INFO("TM: left = %.2lf, right = %.2lf, classifyConf = %d, diff = %d", left_percent, right_percent, diffToClassify, diffToClassify);

    if (left_percent > classifyConfident || right_percent > classifyConfident)
    {
        int delta = (left_percent - right_percent);
        if (abs(delta) >= diffToClassify)
        {
            if (delta > 0) return -1;
            if (delta < 0) return 1;
        }
    }
    return 0;
}

int DetectSign::classifyCountBlue(const cv::Mat& colorROI) const
{
    cv::Mat blue;
    cv::cvtColor(colorROI, blue, cv::COLOR_BGR2HSV);
    cv::inRange(blue, Scalar(minBlue[0], minBlue[1], minBlue[2]), Scalar(maxBlue[0], maxBlue[1], maxBlue[2]), blue);

    int bottomLeftCount = cv::countNonZero(blue(cv::Rect{0, blue.rows/2, blue.cols/2, blue.rows/2}));
    int bottomRightCount = cv::countNonZero(blue(cv::Rect{blue.cols/2, blue.rows/2, blue.cols/2, blue.rows/2}));

    float left_percent = bottomLeftCount * 100.0f/ (blue.cols/2 * blue.rows/2);
    float right_percent = bottomRightCount * 100.0f / (blue.cols/2 * blue.rows/2);

    int delta = (left_percent - right_percent);
    ROS_INFO("CountBlue: left = %.2f, right = %.2f, diff = %d", left_percent, right_percent, diffToClassify);

    if (abs(delta) >= diffToClassify)
    {
        if (delta > 0) return -1;
        if (delta < 0) return 1;
    }
    return 0;
}

int DetectSign::detect()
{
    if (rgb.empty())
    {
        return 0;
    }

    int sign = detectOneFrame();

    recentDetects.pop_front();
    recentDetects.push_back(sign);

    int cntLeft = std::count(recentDetects.begin(), recentDetects.end(), -1);
    int cntRight = std::count(recentDetects.begin(), recentDetects.end(), 1);
    int cntStraight = std::count(recentDetects.begin(), recentDetects.end(), 0);

    int max = std::max(cntLeft, std::max(cntRight, cntStraight));
    if (max == cntLeft)
        return -1;
    else if (max == cntRight)
        return 1;
    else
        return 0;
}

// int DetectSign::detect() {
//     if (rgb.empty())
//     {
//         return 0;
//     }

//     Mat blue;
//     int turnFactor = 0;

//     cvtColor(rgb, blue, cv::COLOR_BGR2HSV);
//     inRange(blue, Scalar(minBlue[0], minBlue[1], minBlue[2]), Scalar(maxBlue[0], maxBlue[1], maxBlue[2]), blue);
//     imshow("blue", blue);

//     Mat canvas = Mat::zeros(rgb.size(), CV_8UC3);
//     vector<vector<Point>> contours;
//     findContours(blue, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

//     if (!contours.empty()) {

//         vector<vector<Point>> contoursPoly((int)contours.size());
//         vector<Rect> boundRect((int)contours.size());

//         for (int i = 0; i < (int)contours.size(); ++i) {
//             approxPolyDP(contours[i], contoursPoly[i], 3, true);
//             boundRect[i] = boundingRect(contoursPoly[i]);
//         }

//         int maxArea = 0, bestRect;
//         for (int i = 0; i < (int)boundRect.size(); ++i) {
//             int boundRect_W = abs(boundRect[i].tl().x - boundRect[i].br().x);
//             int boundRect_H = abs(boundRect[i].tl().y - boundRect[i].br().y);
//             if (boundRect_W * boundRect_H > maxArea) {
//                 maxArea = boundRect_W * boundRect_H;
//                 bestRect = i;
//             }
//         }

//         if (maxArea < 125) goto skip;

//         Point topLeft = boundRect[bestRect].tl();
//         Point bottomRight = boundRect[bestRect].br();
//         rectangle(canvas, topLeft, bottomRight, Scalar(0, 0, 255), 2, 8, 0);

//         int rectW = abs(topLeft.x - bottomRight.x) * 4;
//         int rectH = abs(topLeft.y - bottomRight.y) * 4;
//         Mat zoom(rectW, rectH, CV_8UC1);
//         Point2f boundingBox[4], zoomBox[4];

//         boundingBox[0] = topLeft;
//         boundingBox[1] = Point(bottomRight.x, topLeft.y);
//         boundingBox[2] = bottomRight;
//         boundingBox[3] = Point(topLeft.x, bottomRight.y);

//         zoomBox[0] = Point(0, 0);
//         zoomBox[1] = Point(rectW, 0);
//         zoomBox[2] = Point(rectW, rectH);
//         zoomBox[3] = Point(0, rectH);

//         Mat M = getPerspectiveTransform(boundingBox, zoomBox);
//         warpPerspective(blue, zoom, M, zoom.size(), INTER_LINEAR, BORDER_CONSTANT);

//         int cntLeftOnes = 0, cntRightOnes = 0;
//         for (int i = 0; i < rectW; ++i)
//             for (int j = 0; j < rectH; ++j)
//                 if ((int)zoom.at<uchar>(i, j) == 255) {
//                     if (i < rectW / 2) ++cntLeftOnes;
//                         else ++cntRightOnes;
//                 }

//         cntLeftOnes = cntLeftOnes * cntLeftOnes;
//         cntRightOnes = cntRightOnes * cntRightOnes;
//         turnFactor = cntLeftOnes > cntRightOnes ? 1 : -1;
//     }

//     skip:
//     addWeighted(rgb, 0.5, canvas, 1, 1, canvas);
//     imshow("SignsDetector", canvas);

//     recentDetects.pop_front();
//     recentDetects.push_back(turnFactor);

//     int cntLeft = std::count(recentDetects.begin(), recentDetects.end(), -1);
//     int cntRight = std::count(recentDetects.begin(), recentDetects.end(), 1);
//     int cntStraight = std::count(recentDetects.begin(), recentDetects.end(), 0);

//     int max = std::max(cntLeft, std::max(cntRight, cntStraight));
//     if (max == cntLeft) return -1;
//     else if (max == cntRight) return 1;
//     else return 0;
// }