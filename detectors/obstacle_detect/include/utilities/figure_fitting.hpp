/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#pragma once

#include "utilities/point.hpp"
#include "utilities/point_set.hpp"
#include "utilities/segment.hpp"
#include "utilities/circle.hpp"

#include <opencv2/core.hpp>

#include <cassert>

namespace obstacle_detect {

/*
 * Returns a total best fit approximation of
 * segment based on given point set. The equation
 * used for fitting is given by
 *    Ax + By = -C
 * and the A, B, C parameters are normalized.
 */
Segment fitSegment(PointSet const& point_set) {
  int N = point_set.nbPoints;
  assert(N >= 2);

  cv::Mat input = cv::Mat(N, 2, CV_64F);
  cv::Mat output = cv::Mat::ones(N, 1, CV_64F);

  PointIterator pt = point_set.begin;
  for (int i = 0; i < N; ++i, ++pt) {
    input.at<double>(i, 0) = pt->x;
    input.at<double>(i, 1) = pt->y;
  }
  cv::Mat pinv(2, N, CV_64F);
  cv::invert(input, pinv, cv::DECOMP_SVD);
  cv::Mat params = pinv * output;

  double A = params.at<double>(0), B = params.at<double>(1), C = -1;

  // Find end points
  Point p1 = *point_set.begin;
  Point p2 = *point_set.end;

  Segment segment(p1, p2);
  segment.point_sets.push_back(point_set);

  double D = (A * A + B * B);

  // Project end points on the line
  if (D > 0) {
    Point projected_p1{(B * B * p1.x - A * B * p1.y - A * C) / D,
                       (-A * B * p1.x + A * A * p1.y - B * C) / D};
    Point projected_p2{(B * B * p2.x - A * B * p2.y - A * C) / D,
                       (-A * B * p2.x + A * A * p2.y - B * C) / D};

    segment.first_point = projected_p1;
    segment.last_point = projected_p2;
  }

  return segment;
}

Segment fitSegment(std::vector<PointSet> const& point_sets) {
  int N = 0;
  for (PointSet ps : point_sets) {
    N += ps.nbPoints;
  }
  assert(N >= 2);

  cv::Mat input = cv::Mat(N, 2, CV_64F);
  cv::Mat output = cv::Mat::ones(N, 1, CV_64F);

  int n = 0;
  for (PointSet ps : point_sets) {
    PointIterator point = ps.begin;
    for (int i = 0; i < ps.nbPoints; ++i) {
      input.at<double>(i + n, 0) = point->x;
      input.at<double>(i + n, 1) = point->y;
      std::advance(point, 1);
    }

    n += ps.nbPoints;
  }

  cv::Mat pinv(2, N, CV_64F);
  cv::invert(input, pinv, cv::DECOMP_SVD);
  cv::Mat params = pinv * output;
  double A = params.at<double>(0), B = params.at<double>(1), C = -1;

  // Find end points
  Point p1 = *point_sets.front().begin;
  Point p2 = *point_sets.back().end;

  Segment segment(p1, p2);
  segment.point_sets = point_sets;

  double D = (A * A + B * B);

  // Project end points on the line
  if (D > 0.0) {
    Point projected_p1, projected_p2;

    projected_p1.x = (B * B * p1.x - A * B * p1.y - A * C) / D;
    projected_p1.y = (-A * B * p1.x + A * A * p1.y - B * C) / D;

    projected_p2.x = (B * B * p2.x - A * B * p2.y - A * C) / D;
    projected_p2.y = (-A * B * p2.x + A * A * p2.y - B * C) / D;

    segment.first_point = projected_p1;
    segment.last_point = projected_p2;
  }

  return segment;
}

/*
 * Returns a total best fit approximation of
 * a circle based on given point set. The equation
 * used for fitting is given by
 *   a1 * x + a2 * y + a3 = -(x^2 + y^2)
 * where parameters a1, a2, a3 are obtained from
 * circle equation
 *   (x-x0)^2 + (y-y0)^2 = r^2.
 */
Circle fitCircle(std::list<Point> const& point_set) {
  int N = point_set.size();
  assert(N >= 3);

  cv::Mat input = cv::Mat(N, 3, CV_64F);
  cv::Mat output = cv::Mat(N, 1, CV_64F);

  int i = 0;
  for (Point const& point : point_set) {
    input.at<double>(i, 0) = point.x;
    input.at<double>(i, 1) = point.y;
    input.at<double>(i, 2) = 1.0;
    output.at<double>(i) = -(pow(point.x, 2) + pow(point.y, 2));
    ++i;
  }
  cv::Mat pinv(3, N, CV_64F);
  cv::invert(input, pinv, cv::DECOMP_SVD);
  cv::Mat params = pinv * output;

  Point center(-params.at<double>(0) / 2.0, -params.at<double>(1) / 2.0);
  double radius = sqrt((params.at<double>(0) * params.at<double>(0) +
                        params.at<double>(1) * params.at<double>(1)) /
                           4.0 -
                       params.at<double>(2));

  return Circle(center, radius);
}
}  // namespace obstacle_detect
