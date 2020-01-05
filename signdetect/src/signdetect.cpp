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

constexpr const char *CONF_SIGN_WINDOW = "SignDetect";

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

DetectSign::DetectSign(const cv::Mat &leftTemplate, const cv::Mat &rightTemplate)
    : recentDetects{MAX_FRAME_COUNT, 0}, LEFT_TEMPLATE{leftTemplate}, RIGHT_TEMPLATE{rightTemplate}, _nh{"signdetect"}, _debugImage{_nh}
{
    _roiPublisher = _debugImage.advertise("/debug/sign/ROI", 1, false);
    _thresholdedPublisher = _debugImage.advertise("/debug/sign/blue", 1, false);
    _detectPublisher = _debugImage.advertise("/debug/sign/detect", 1, false);

    cv::namedWindow(CONF_SIGN_WINDOW);
    // cv::createTrackbar("canny", CONF_SIGN_WINDOW, &canny, 255);
    // cv::createTrackbar("votes", CONF_SIGN_WINDOW, &votes, 255);

    // cv::createTrackbar("Strategy", CONF_SIGN_WINDOW, &classifyStrategy, 1);
    // cv::createTrackbar("DetectConfident", CONF_SIGN_WINDOW, &detectConfident, 100);
    // cv::createTrackbar("ClassifyConfident", CONF_SIGN_WINDOW, &classifyConfident, 100);
    // cv::createTrackbar("DiffToClassify", CONF_SIGN_WINDOW, &diffToClassify, 100);

    cv::createTrackbar("MinBlue H", CONF_SIGN_WINDOW, &low_minBlue[0], 179);
    cv::createTrackbar("MinBlue S", CONF_SIGN_WINDOW, &low_minBlue[1], 255);
    cv::createTrackbar("MinBlue V", CONF_SIGN_WINDOW, &low_minBlue[2], 255);
    cv::createTrackbar("MaxBlue H", CONF_SIGN_WINDOW, &low_maxBlue[0], 179);
    cv::createTrackbar("MaxBlue S", CONF_SIGN_WINDOW, &low_maxBlue[1], 255);
    cv::createTrackbar("MaxBlue V", CONF_SIGN_WINDOW, &low_maxBlue[2], 255);


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
    votes = config.vote_kmean;
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

    Mat gray;
    inRange(depth, cv::Scalar{10}, cv::Scalar{180}, gray);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0, 0);
    cv::imshow("Gray", gray);

    Mat blue_low_pass;
    Mat hsv;
    cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);
    inRange(hsv, Scalar(low_minBlue[0], low_minBlue[1], low_minBlue[2]), Scalar(low_maxBlue[0], low_maxBlue[1], low_maxBlue[2]), blue_low_pass);
    cv::medianBlur(blue_low_pass, blue_low_pass, 5);

    cv::imshow("Blue Low Pass", blue_low_pass);

    gray &= blue_low_pass;
    cv::imshow("GrayAfter", gray);


    vector<Vec3f> circles;

    HoughCircles(gray, circles, CV_HOUGH_GRADIENT,
                 1,       // accumulator resolution (size of the image / 2)
                 300,     // minimum distance between two circles
                 canny,   // Canny high threshold
                 votes,   // minimum number of votes
                 0, 100); // min and max radius



    // debug
    ROS_INFO("Circles size = %ul", circles.size());
    for (size_t i = 0; i < circles.size(); i++)
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle(gray, center, 3, Scalar(127), -1, 8, 0);
        // // circle outline
        circle(gray, center, radius, Scalar(100), 3, 8, 0);
        cv::imshow("Draw Gray", gray);
    }

    // TODO: Debug
    // return 0;

    cv::Mat blue_high_pass;
    inRange(hsv, Scalar(minBlue[0], minBlue[1], minBlue[2]), Scalar(maxBlue[0], maxBlue[1], maxBlue[2]), blue_high_pass);
    const cv::Rect imageBound{0,0,gray.cols,gray.rows};
    for (size_t i = 0; i < circles.size(); i++)
    {
        const auto& circle = circles[i];
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]) + 10; // important

        cv::Rect roi(center.x - radius, center.y - radius, radius * 2, radius * 2);
        roi &= imageBound;

        cv::Mat roiImg(blue_high_pass, roi);

        int real_x1 = roiImg.cols - 1, real_y1 = roiImg.rows - 1, real_x2 = 0, real_y2 = 0;
        for (int row = 0; row < roiImg.rows; row++)
        {
            for (int col = 0; col < roiImg.cols; col++)
            {
                if (roiImg.at<uchar>(row, col))
                {
                    real_x1 = std::min(real_x1, col);
                    real_y1 = std::min(real_y1, row);
                    real_x2 = std::max(real_x2, col);
                    real_y2 = std::max(real_y2, row);
                }
            }
        }

        if (real_x1 > real_x2)
            std::swap(real_x1, real_x2);
        if (real_y1 > real_y2)
            std::swap(real_y1, real_y2);

        int w = abs(real_x2 - real_x1);
        int h = abs(real_y2 - real_y1);

        if (w < 10 || h < 10)
        {
            continue;
        }

        cv::Rect boundingBox{real_x1, real_y1, w, h};
        ROS_INFO_STREAM(boundingBox);
        roiImg = roiImg(boundingBox);
        cv::imshow("ROI Img before", roiImg);

        cv::morphologyEx(roiImg, roiImg, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, cv::Size{3,3}));
        cv::imshow("ROI Img after", roiImg);

        float percent = cv::countNonZero(roiImg) * 100.0f / (roiImg.rows * roiImg.cols);
        ROS_INFO("percent sign = %.2f", percent);
        if (percent < detectConfident)
        {
            continue;
        }

        cv::imshow("threshold + ROI", roiImg);
        cv::waitKey(1);
        // showImage(_detectPublisher, "mono8", depth);

            // cv::Point top, bot, left, right;
            // // find top
            // for (int row = 0; row < roiImg.rows; row++)
            // {
            //     int n = 0;
            //     for (int col = 0; col < roiImg.cols; col++)
            //     {
            //         if (roiImg.at<uchar>(row, col))
            //         {
            //             top.x += col;
            //             top.y += row;
            //             n++;
            //         }
            //     }
            //     if (n > 0)
            //     {
            //         top.x /= n * 1.0f;
            //         top.y /= n * 1.0f;
            //         break;
            //     }
            // }

            // ROS_INFO_STREAM("top ok: " << top);

            // // find bot
            // for (int row = roiImg.rows - 1; row >= 0; row--)
            // {
            //     int n = 0;
            //     for (int col = 0; col < roiImg.cols; col++)
            //     {
            //         if (roiImg.at<uchar>(row, col))
            //         {
            //             bot.x += col;
            //             bot.y += row;
            //             n++;
            //         }
            //     }
            //     if (n > 0)
            //     {
            //         bot.x /= n * 1.0f;
            //         bot.y /= n * 1.0f;
            //         break;
            //     }
            // }

            // ROS_INFO_STREAM("bot ok: " << bot);
            // // find left
            // for (int col = 0; col < roiImg.cols; col++)
            // {
            //     int n = 0;
            //     for (int row = 0; row < roiImg.rows; row++)
            //     {
            //         if (roiImg.at<uchar>(row, col))
            //         {
            //             left.x += col;
            //             left.y += row;
            //             n++;
            //         }
            //     }
            //     if (n > 0)
            //     {
            //         left.x /= n * 1.0f;
            //         left.y /= n * 1.0f;
            //         break;
            //     }
            // }
            // ROS_INFO_STREAM("left ok: " << left);
            // // find right
            // for (int col = roiImg.cols - 1; col >= 0; col--)
            // {
            //     int n = 0;
            //     for (int row = 0; row < roiImg.rows; row++)
            //     {
            //         if (roiImg.at<uchar>(row, col))
            //         {
            //             right.x += col;
            //             right.y += row;
            //             n++;
            //         }
            //     }
            //     if (n > 0)
            //     {
            //         right.x /= n * 1.0f;
            //         right.y /= n * 1.0f;
            //         break;
            //     }
            // }
            // ROS_INFO_STREAM("right ok: " << right);
            // // std::swap(top, bot);
            // // std::swap(left, right);
            {
                cv::Mat resized;
                cv::resize(roiImg, resized, cv::Size{64, 64});
                cv::imshow("ROI resized", resized);
            }

            // cv::Mat warppedROI(roiImg.rows, roiImg.cols, roiImg.type());
            // {
            //     // int W = roiImg.cols;
            //     // int H = roiImg.rows;

            //     cv::Point2f inQuad[4] = {
            //         top,
            //         right,
            //         bot,
            //         left};

            //     cv::Point2f outQuad[4] = {
            //         cv::Point{roiImg.cols / 2, 0},
            //         cv::Point{roiImg.cols - 1, roiImg.rows / 2},
            //         cv::Point{roiImg.cols / 2, roiImg.rows - 1},
            //         cv::Point{0, roiImg.rows / 2}};

            //     // cv::Point2f inQuad[4] = {
            //     //     cv::Point(0, 0),
            //     //     cv::Point(W - 1, 0),
            //     //     cv::Point(0, H - 1),
            //     //     cv::Point(W - 1, H - 1)};

            //     // cv::Point2f outQuad[4] = {
            //     //     cv::Point(0, 0),
            //     //     cv::Point(SIZE_X - 1, 0),
            //     //     cv::Point(0, SIZE_X - 1),
            //     //     cv::Point(SIZE_X - 1, SIZE_X - 1)};

            //     cv::Mat M = getPerspectiveTransform(inQuad, outQuad);
            //     warpPerspective(roiImg, warppedROI, M, warppedROI.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
            //     // ROS_INFO_STREAM("warp in: " << inQuad[0] << inQuad[1] << inQuad[2] << inQuad[3]);
            //     // ROS_INFO_STREAM("warp out: " << outQuad[0] << outQuad[1] << outQuad[2] << outQuad[3]);
            // }
            // cv::imshow("ROI img warpped", warppedROI);

            int sign = classify(rgb(roi)(boundingBox));
            if (sign != 0)
            {
                return sign;
            }

            // fit bounding box
            // std::vector<std::vector<cv::Point>> contours;
            // std::vector<cv::Vec4i> hierarchy;
            // cv::findContours(roiImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            // ROS_INFO("contours size = %ull", contours.size());
            // for (size_t i = 0; i < contours.size(); i++)
            // {
            //     ROS_INFO("cv::contourArea(contours[%d]) = %.2lf", i, cv::contourArea(contours[i]));
            //     if (cv::contourArea(contours[i]) < 50)
            //     {
            //         continue;
            //     }

            //     cv::Rect &&boundingBox = cv::boundingRect(contours[i]); // fitter to the sign
            //     cv::Rect &&newROI = cv::Rect{0, 0, roiImg.cols, roiImg.rows} & boundingBox;
            //     roiImg = roiImg(newROI);
            //     cv::imshow("ROI Img", roiImg);
            //     // cv::Mat colorBlue;
            //     // cv::cvtColor(roiImg, colorBlue, cv::COLOR_GRAY2BGR);
            //     // cv::Mat thresholdconcat;
            //     // cv::hconcat((rgb(roi))(boundingBox), colorBlue, thresholdconcat);
            //     // showImage(_roiPublisher, "bgr8", thresholdconcat);

            //     int sign = classify(rgb(roi)(newROI));
            //     if (sign != 0)
            //     {
            //         return sign;
            //     }

            //     // showImage("DetectSign", depth);
            //     // showImage("DetectSignROI", roiImg);
            // }
    }

    return 0;
}

int DetectSign::classify(const cv::Mat &colorROI) const
{
    switch (classifyStrategy)
    {
    case 0:
        return classifyCountBlue(colorROI);
    case 1:
        return classifyTemplateMatching(colorROI);
    default:
        return classifyCountBlue(colorROI);
    }
}

int DetectSign::classifyTemplateMatching(const cv::Mat &colorROI) const
{
    cv::imshow("ClassifyInput", colorROI);
    cv::Mat gray;
    cv::cvtColor(colorROI, gray, cv::COLOR_BGR2GRAY);

    // TODO: may be threshold?
    // ...

    cv::Mat resized;
    cv::resize(gray, resized, cv::Size(), SIZE_X / colorROI.cols, SIZE_X / colorROI.rows);
    cv::imshow("ClassifyInputResized", resized);
    cv::GaussianBlur(resized, resized, cv::Size(5, 5), 0, 0);
    cv::imshow("ClassifyInputResizedBlurred", resized);

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
            if (delta > 0)
                return -1;
            if (delta < 0)
                return 1;
        }
    }
    return 0;
}

int DetectSign::classifyCountBlue(const cv::Mat &colorROI) const
{
    cv::Mat blue;
    cv::cvtColor(colorROI, blue, cv::COLOR_BGR2HSV);
    cv::inRange(blue, Scalar(minBlue[0], minBlue[1], minBlue[2]), Scalar(maxBlue[0], maxBlue[1], maxBlue[2]), blue);

    int bottomLeftCount = cv::countNonZero(blue(cv::Rect{0, blue.rows / 2, blue.cols / 2, blue.rows / 2}));
    int bottomRightCount = cv::countNonZero(blue(cv::Rect{blue.cols / 2, blue.rows / 2, blue.cols / 2, blue.rows / 2}));

    float left_percent = bottomLeftCount * 100.0f / (blue.cols / 2 * blue.rows / 2);
    float right_percent = bottomRightCount * 100.0f / (blue.cols / 2 * blue.rows / 2);

    int delta = (left_percent - right_percent);
    ROS_INFO("CountBlue: left = %.2f, right = %.2f, diff = %d", left_percent, right_percent, diffToClassify);

    if (abs(delta) >= diffToClassify)
    {
        if (delta > 0)
            return -1;
        if (delta < 0)
            return 1;
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
    // return sign;
    recentDetects.push_back(sign);  
    if (recentDetects.size() > MAX_FRAME_COUNT)
    {
        recentDetects.pop_front();
    }

    int cntLeft = std::count(recentDetects.begin(), recentDetects.end(), -1);
    int cntRight = std::count(recentDetects.begin(), recentDetects.end(), 1);
    int cntStraight = std::count(recentDetects.begin(), recentDetects.end(), 0);

    int max = std::max(cntLeft, std::max(cntRight, cntStraight));
    ///  WTF ??? 
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