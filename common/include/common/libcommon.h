#ifndef UTILS_H
#define UTILS_H

#include <memory>
#include <array>
#include <vector>
#include <opencv2/opencv.hpp>
#include <image_transport/publisher.h>

typedef std::array<double, 3> LineParams;

// x = ay^2 + by + c
std::shared_ptr<LineParams> calcLineParams(const std::vector<cv::Point> &listPoint);

int getXByY(const LineParams &params, double y);

cv::Mat kmean(cv::Mat image, size_t kCluster);

cv::Mat birdviewTransformation(const cv::Mat &src, int birdwidth, int birdheight, int offsetLeft, int offsetRight, int skyline, cv::Mat &returnM);

void showImage(const image_transport::Publisher &publisher, const std::string &encode, const cv::Mat &image);
void showImage(const std::string &winname, const cv::Mat &image);

#endif