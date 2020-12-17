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
#pragma once

#include "utilities/point.hpp"
#include "utilities/segment.hpp"
#include "utilities/circle.hpp"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include "obstacle_detect/obstacleDetectConfig.h"

#include <mutex>

namespace obstacle_detect {

class ObstacleDetect {
 public:
  ObstacleDetect(ros::NodeHandle& nh);

 private:
  ros::Subscriber scanSub_;
  ros::Publisher obstaclePub_;
  std::string frameId_;
  ros::Time stamp_;

  std::list<Point> inPts_;
  std::list<Segment> segments_;
  std::list<Circle> circles_;

  void lidarCallback(sensor_msgs::LaserScan::ConstPtr const& scanMsg);

  void publishObstacles();

  void preprocessInput();
  void doSegmentations(PointSet const& pointSet);
  void mergeSegments();
  bool compareSegments(Segment const& s1, Segment const& s2,
                       Segment& mergedSegment);
  bool checkSegmentsProximity(Segment const& s1, Segment const& s2);
  bool checkSegmentsCollinearity(Segment const& segment, Segment const& s1,
                                 Segment const& s2);

  void detectCircles();
  void mergeCircles();
  bool compareCircles(Circle const& c1, Circle const& c2,
                      Circle& merged_circle);

  bool debug_;

  using cfg_t = obstacle_detect::obstacleDetectConfig;
  std::unique_ptr<dynamic_reconfigure::Server<cfg_t>> drServerPtr_;
  void drCallback(cfg_t& cfg, uint32_t level);
  cfg_t cfg_;

  int y_max, x_min, x_max;
};
}  // namespace obstacle_detect
