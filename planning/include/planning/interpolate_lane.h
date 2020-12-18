#pragma once
#include <array>
#include "common/libcommon.h"
#include <opencv2/core.hpp>

std::vector<cv::Point> getPointsFromParams(const std::array<float, 3>& lineParams, int imageHeight)
{
    std::vector<cv::Point> points;
    for (int y = 0; y < imageHeight; y++)
    {
        cv::Point point{getXByY(lineParams, y), y};
        points.push_back(point);
    }
    return points;
}

std::vector<cv::Point> calcGradient(const std::vector<cv::Point> &points) const
{
    std::vector<cv::Point> gradients;
    for (int i = 0; i < points.size() - 1; i++)
    {
        gradients.push_back(points[i + 1] - points[i]);
    }
    return gradients;
}

std::array<float, 3> moveByGradient(const std::array<float, 3>& lineParams, int imageHeight, int distance, int direct)
{
    std::vector<cv::Point> points = getPointsFromParams(lineParams, imageHeight);
    std::vector<cv::Point> gradients = calcGradient(points);
    for (int i = 0; i < points.size() - 1; i++)
    {
        cv::Point perdepencular;
        if (direct == Direct::LEFT)
        {
            perdepencular = cv::Point{gradients[i].y, -gradients[i].x};
        } else if (direct == Direct::RIGHT)
        {
            perdepencular = cv::Point{-gradients[i].y, gradients[i].x};
        } else
        {
            continue;
        }
        perdepencular /= sqrt(perdepencular.x * perdepencular.x + perdepencular.y * perdepencular.y);
        points[i] += (perdepencular * distance);

    }

    std::array<float, 3> newParams = calcLineParams(points);
    return newParams;
}
