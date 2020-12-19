#pragma once
#include <array>
#include "common/libcommon.h"
#include <ros/ros.h>
#include <opencv2/core.hpp>

void visualizeLine(cv::Mat image, const std::array<float,3>& lineParams, cv::Scalar color)
{
    for (int y = 0; y < 240; y++)
    {
        cv::Point p{getXByY(lineParams, y), y};
        cv::circle(image, p, 3, color);
    }
}


std::vector<cv::Point> getPointsFromParams(const std::array<float, 3>& lineParams, int imageHeight)
{
    std::vector<cv::Point> points;
    for (int y = 0; y < imageHeight; y+=2)
    {
        cv::Point point{getXByY(lineParams, y), y};
        points.push_back(point);
    }
    return points;
}

std::vector<cv::Point> calcGradient(const std::vector<cv::Point> &points)
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
    // cv::Mat debug = cv::Mat::zeros(480, 640, CV_8UC3);
    std::vector<cv::Point> points = getPointsFromParams(lineParams, imageHeight);
    std::vector<cv::Point> points2;
    std::vector<cv::Point> gradients = calcGradient(points);
    for (int i = 0; i < points.size() - 1; i+=2)
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
        cv::Point p = points[i] + (perdepencular * distance);
        if (p.y <= 240 && p.y >= 0)
        {
            points2.push_back(p);
        }
    }
    points2.push_back(cv::Point{points.back().x + distance, points.back().y});

    ROS_INFO_STREAM(points.size() << ' ' << points2.size());

    // for(int i = 0; i < (int)points2.size(); ++i)
    // {
    //     cv::circle(debug, points2[i], 3, cv::Scalar(0,255,0));
    // }

    // return calcLineParams(points2).getParams();

    auto params = calcLineParams(points2).getParams();
    // visualizeLine(debug, params, cv::Scalar{255, 0, 0});
    // cv::imshow("Debug", debug);
    // cv::waitKey(1);

    return params;
}
