#include "detectsign.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <string>
#include <exception>
#include "utils.h"
using namespace cv;
using namespace std;


#define SIZE_X 64.0

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
{
    cv::createTrackbar("canny", "Threshold", &canny, 255);
    cv::createTrackbar("votes", "Threshold", &votes, 255);
    

    cv::createTrackbar("Sign percent", "Threshold", &maxPercent, 100);

    cv::createTrackbar("MinBlue H", "Threshold", &minBlue[0], 179);
    cv::createTrackbar("MinBlue S", "Threshold", &minBlue[1], 255);
    cv::createTrackbar("MinBlue V", "Threshold", &minBlue[2], 255);
    cv::createTrackbar("MaxBlue H", "Threshold", &maxBlue[0], 179);
    cv::createTrackbar("MaxBlue S", "Threshold", &maxBlue[1], 255);
    cv::createTrackbar("MaxBlue V", "Threshold", &maxBlue[2], 255);
    
    MAX_DIFF = matching(LEFT_TEMPLATE, cv::Scalar::all(255) - LEFT_TEMPLATE);
}

void DetectSign::updateRGB(const cv::Mat& rgb)
{
    this->rgb = rgb.clone();
}

void DetectSign::updateDepth(const cv::Mat& depth)
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
    int turnFactor = 0;

    // new method 

    cvtColor(rgb, blue, cv::COLOR_BGR2HSV);

    inRange(blue, Scalar(minBlue[0], minBlue[1], minBlue[2]), Scalar(maxBlue[0], maxBlue[1], maxBlue[2]), blue);


    Mat gray;
    gray = kmean(this->depth,2);

    cvtColor(gray, gray, CV_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0, 0);

    vector<Vec3f> circles;

    HoughCircles(gray, circles, CV_HOUGH_GRADIENT,
          1,   // accumulator resolution (size of the image / 2)
          300,  // minimum distance between two circles
          canny, // Canny high threshold
          votes, // minimum number of votes
          0, 100); // min and max radius
    for (size_t i = 0; i < circles.size(); i++)
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle(this->depth, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        // circle outline
        circle(this->depth, center, radius, Scalar(0, 0, 255), 3, 8, 0);
        cv::Rect roi(center.x-radius, center.y-radius, radius*2, radius*2 );

        if ((roi & cv::Rect(0, 0, gray.cols, gray.rows)) == roi )
        {
            cv::Mat roiImg( blue, roi);
            float percent = cv::countNonZero(roiImg) * 100.0f/ (roiImg.rows * roiImg.cols);
            imshow("blue", roiImg);
            imshow("detected circles", this->depth);    
            if (percent > maxPercent) 
            {
                std::cout << "SIGN DETECTED\n";
                return 1;
            }
        }
    }
    
    // new method

    return 0;

    // vector<vector<Point>> contours;
    // findContours(blue, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
    // drawContours(blue,contours,-1, (255,0,0), 2);
    // if (!contours.empty()) {
        
    //     vector<vector<Point>> contoursPoly((int)contours.size());
    //     vector<Rect> boundRect((int)contours.size());

    //     for (int i = 0; i < (int)contours.size(); ++i) {
    //         approxPolyDP(contours[i], contoursPoly[i], 3, true);
    //         boundRect[i] = boundingRect(contoursPoly[i]);    
    //     }

    //     float maxPercent = 0;
    //     int i_max = -1, max_type = 0;

    //     for (int i = 0; i < (int)boundRect.size(); ++i) {
    //         if (boundRect[i].area() < 20*20) {
    //             continue;
    //         }

    //         cv::Mat mask = Mat::zeros(blue.size(), CV_8UC1);
    //         drawContours(mask, contours, i, Scalar(255), CV_FILLED);
    //         cv::Mat crop = cv::Mat::zeros(blue.size(), CV_8UC1);
    //         blue.copyTo(crop, mask);

    //         cv::Mat resized;
    //         cv::resize(crop(boundRect[i]), resized, cv::Size(), SIZE_X / boundRect[i].width, SIZE_X / boundRect[i].height);
    //         cv::GaussianBlur(resized, resized, cv::Size(5, 5), 0, 0);

    //         {
    //             double val = matching(resized, LEFT_TEMPLATE);
    //             double percent = 1.0 - val / MAX_DIFF;
    //             if (percent > maxPercent)
    //             {
    //                 i_max = i;
    //                 maxPercent = percent;
    //                 max_type = -1;
    //             }
    //         }

    //         {
    //             double val = matching(resized, RIGHT_TEMPLATE);
    //             double percent = 1.0 - val / MAX_DIFF;
    //             if (percent > maxPercent)
    //             {
    //                 i_max = i;
    //                 maxPercent = percent;
    //                 max_type = 1;
    //             }
    //         }
    //     }

    //     if (maxPercent >= 0.70)
    //     {
    //         turnFactor = max_type;
    //     }
    // }

    // return turnFactor;
}

int DetectSign::detect() {
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
    if (max == cntLeft) return -1;
    else if (max == cntRight) return 1;
    else return 0;
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