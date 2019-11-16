#ifndef UTILS_H
#define UTILS_H

#include <memory>
#include <array>
#include <vector>
#include <opencv2/opencv.hpp>

typedef std::array<float, 3> LineParams;

std::shared_ptr<LineParams> calcLineParams(const std::vector<cv::Point>& listPoint);

float getXByY(const LineParams& params, float y);
float getYByX(const LineParams& params, float y);


#endif