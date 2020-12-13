#ifndef UTILS_H
#define UTILS_H

#include <memory>
#include <array>
#include <vector>
#include <opencv2/opencv.hpp>
#include "common/lineparams.h"

enum Direct {
    LEFT = -1,
    NONE = 0,
    RIGHT = 1,
};

// x = ay^2 + by + c
LineParams calcLineParams(const std::vector<cv::Point> &listPoint);

int getXByY(const LineParams &params, double y);

cv::Mat kmean(cv::Mat image, size_t kCluster);

cv::Mat birdviewTransformation(const cv::Mat &src, int birdwidth, int birdheight, int offsetLeft, int offsetRight, int skyline, cv::Mat &returnM);

float errorAngle(const cv::Point &src, const cv::Point &dst);

#endif