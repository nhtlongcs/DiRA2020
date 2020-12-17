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

#include "obstacle_detect/obstacle_detect.hpp"
#include "utilities/figure_fitting.hpp"
#include "cds_msgs/obstacles.h"
#include <std_msgs/Bool.h>

namespace obstacle_detect {

ObstacleDetect::ObstacleDetect(ros::NodeHandle& nh) {
  ros::param::param<bool>("~debug", debug_, false);

  ros::param::get("~min_range", cfg_.min_range);
  ros::param::get("~max_range", cfg_.max_range);

  ros::param::get("~use_split_and_merge", cfg_.use_split_and_merge);
  ros::param::get("~detect_only_visible_circle",
                  cfg_.detect_only_visible_circle);
  ros::param::get("~publish_seg_of_circle", cfg_.publish_seg_of_circle);

  ros::param::get("~min_group_points", cfg_.min_group_points);

  ros::param::get("~p_max_group_distance_", cfg_.max_group_distance);
  ros::param::get("~p_distance_proportion_", cfg_.distance_proportion);
  ros::param::get("~max_split_distance", cfg_.max_split_distance);
  ros::param::get("~max_merge_separation", cfg_.max_merge_separation);
  ros::param::get("~max_merge_spread", cfg_.max_merge_spread);
  ros::param::get("~max_circle_radius", cfg_.max_circle_radius);

  ros::param::get("~frame_id", frameId_);

  ros::param::get("~y_max", y_max);
  ros::param::get("~x_min", x_min);
  ros::param::get("~x_max", x_max);

  std::string topic;
  ros::param::param<std::string>("~laser_topic", topic, "/scan");
  scanSub_ = nh.subscribe(topic, 5, &ObstacleDetect::lidarCallback, this);
  ros::param::param<std::string>("~object_topic", topic, "/object");
  obstaclePub_ = nh.advertise<std_msgs::Bool>(topic, 5);

  if (debug_) {
    drServerPtr_ = std::make_unique<dynamic_reconfigure::Server<cfg_t>>();
    drServerPtr_->setCallback([&](auto&& cfg, auto&& level) {
      drCallback(std::forward<decltype(cfg)>(cfg),
                 std::forward<decltype(level)>(level));
    });
  }
}

void ObstacleDetect::drCallback(cfg_t& cfg, uint32_t level) {
  cfg_ = cfg;
  ROS_INFO_STREAM(cfg_.max_circle_radius);
}

void ObstacleDetect::lidarCallback(
    sensor_msgs::LaserScan::ConstPtr const& scanMsg) {
  frameId_ = scanMsg->header.frame_id;
  stamp_ = scanMsg->header.stamp;
  double phi = scanMsg->angle_min;
  for (auto const& r : scanMsg->ranges) {
    if (cfg_.min_range <= r && r <= cfg_.max_range)
      inPts_.emplace_back(r * cos(phi), r * sin(phi));
    phi += scanMsg->angle_increment;
  }

  segments_.clear();
  circles_.clear();

  preprocessInput();
  mergeSegments();

  detectCircles();
  mergeCircles();

  publishObstacles();
  inPts_.clear();
}

void ObstacleDetect::preprocessInput() {
  static double sin_dp = sin(2.0 * cfg_.distance_proportion);
  PointSet pt_set{inPts_.begin(), inPts_.begin(), 1, true};

  for (PointIterator point = inPts_.begin(); point != inPts_.end(); ++point) {
    double range = point->length();
    double distance = (*point - *pt_set.end).length();

    if (distance < cfg_.max_group_distance + range * cfg_.distance_proportion) {
      pt_set.end = point;
      ++pt_set.nbPoints;
    } else {
      double prevRange = pt_set.end->length();
      // Heron's equation
      double p = (range + prevRange + distance) / 2;
      double S = sqrt(p * (p - range) * (p - prevRange) * (p - distance));
      // Sine of angle between beams
      double sin_d = 2 * S / (range * prevRange);

      // TODO: This condition can be fulfilled if the point are on the opposite
      // sides of the scanner (angle = 180 deg). Needs another check.
      if (std::abs(sin_d) < sin_dp && range < prevRange) {
        pt_set.is_visible = false;
      }
      doSegmentations(pt_set);

      // Begin new point set
      pt_set.begin = point;
      pt_set.end = point;
      pt_set.nbPoints = 1;
      pt_set.is_visible = (std::abs(sin_d) > sin_dp || range < prevRange);
    }
  }
  doSegmentations(pt_set);  // Check the last point set too!
}

void ObstacleDetect::doSegmentations(PointSet const& pt_set) {
  if (pt_set.nbPoints < cfg_.min_group_points) {
    return;
  }

  Segment segment(*pt_set.begin,
                  *pt_set.end);  // Use Iterative End Point Fit

  if (cfg_.use_split_and_merge) {
    segment = fitSegment(pt_set);
  }

  PointIterator set_divider;
  double max_distance = 0, distance = 0;
  int split_idx = 0;  // Natural index of splitting point (counting from 1)
  int point_idx = 0;  // Natural index of current point in the set

  // Seek the point of division
  for (PointIterator point = pt_set.begin; point != pt_set.end; ++point) {
    ++point_idx;

    if ((distance = segment.distanceTo(*point)) >= max_distance) {
      double r = point->length();

      if (cfg_.max_group_distance + r * cfg_.distance_proportion < distance) {
        max_distance = distance;
        set_divider = point;
        split_idx = point_idx;
      }
    }
  }
  // Split the set only if the sub-groups are not 'small'
  if (max_distance > 0 && cfg_.min_group_points < split_idx &&
      split_idx < pt_set.nbPoints - cfg_.min_group_points) {
    set_divider = inPts_.insert(
        set_divider, *set_divider);  // Clone the dividing point for each group

    PointSet subset1{pt_set.begin, set_divider, split_idx, pt_set.is_visible};
    PointSet subset2{++set_divider, pt_set.end, pt_set.nbPoints - split_idx,
                     pt_set.is_visible};
    doSegmentations(subset1);
    doSegmentations(subset2);
  } else {  // Add the segment
    if (!cfg_.use_split_and_merge) {
      segment = fitSegment(pt_set);
    }
    segments_.push_back(segment);
  }
}

void ObstacleDetect::mergeSegments() {
  for (auto i = segments_.begin(); i != segments_.end(); ++i) {
    for (auto j = i; j != segments_.end(); ++j) {
      Segment mergedSegment;

      if (compareSegments(*i, *j, mergedSegment)) {
        auto tmpItr = segments_.insert(i, mergedSegment);
        segments_.erase(i);
        segments_.erase(j);
        i = --tmpItr;  // Check the new segment against others
        break;
      }
    }
  }
}

bool ObstacleDetect::compareSegments(Segment const& s1, Segment const& s2,
                                     Segment& merged_segment) {
  if (&s1 == &s2) {
    return false;
  }

  // Segments must be provided counter-clockwise
  if (s1.first_point.cross(s2.first_point) < 0.0)
    return compareSegments(s2, s1, merged_segment);

  if (checkSegmentsProximity(s1, s2)) {
    std::vector<PointSet> point_sets;
    point_sets.insert(point_sets.end(), s1.point_sets.begin(),
                      s1.point_sets.end());
    point_sets.insert(point_sets.end(), s2.point_sets.begin(),
                      s2.point_sets.end());

    Segment segment = fitSegment(point_sets);
    if (checkSegmentsCollinearity(segment, s1, s2)) {
      merged_segment = segment;
      return true;
    }
  }
  return false;
}

bool ObstacleDetect::checkSegmentsProximity(Segment const& s1,
                                            Segment const& s2) {
  auto& maxMergeSeperation = cfg_.max_merge_separation;
  return s1.trueDistanceTo(s2.first_point) < maxMergeSeperation ||
         s1.trueDistanceTo(s2.last_point) < maxMergeSeperation ||
         s2.trueDistanceTo(s1.first_point) < maxMergeSeperation ||
         s2.trueDistanceTo(s1.last_point) < maxMergeSeperation;
}

bool ObstacleDetect::checkSegmentsCollinearity(Segment const& segment,
                                               Segment const& s1,
                                               Segment const& s2) {
  auto& maxMergeSpread = cfg_.max_merge_spread;
  return segment.distanceTo(s1.first_point) < maxMergeSpread &&
         segment.distanceTo(s1.last_point) < maxMergeSpread &&
         segment.distanceTo(s2.first_point) < maxMergeSpread &&
         segment.distanceTo(s2.last_point) < maxMergeSpread;
}

void ObstacleDetect::detectCircles() {
  for (auto segment = segments_.begin(); segment != segments_.end();
       ++segment) {
    if (cfg_.detect_only_visible_circle) {
      for (auto const& ps : segment->point_sets) {
        if (!ps.is_visible) {
          continue;
        }
      }
    }

    Circle circle(*segment);
    if (circle.radius < cfg_.max_circle_radius) {
      circles_.push_back(circle);
      if (!cfg_.publish_seg_of_circle) {
        segment = segments_.erase(segment);
        --segment;
      }
    }
  }
}

void ObstacleDetect::mergeCircles() {
  for (auto i = circles_.begin(); i != circles_.end(); ++i) {
    for (auto j = i; j != circles_.end(); ++j) {
      Circle mergedCircle;

      if (compareCircles(*i, *j, mergedCircle)) {
        auto temp_itr = circles_.insert(i, mergedCircle);
        circles_.erase(i);
        circles_.erase(j);
        i = --temp_itr;
        break;
      }
    }
  }
}

bool ObstacleDetect::compareCircles(Circle const& c1, Circle const& c2,
                                    Circle& mergedCircle) {
  if (&c1 == &c2) return false;

  // If circle c1 is fully inside c2 - merge and leave as c2
  if (c2.radius - c1.radius >= (c2.center - c1.center).length()) {
    mergedCircle = c2;
    return true;
  }
  // If circle c2 is fully inside c1 - merge and leave as c1
  if (c1.radius - c2.radius >= (c2.center - c1.center).length()) {
    mergedCircle = c1;
    return true;
  }
  // If circles intersect and are 'small' - merge
  if (c1.radius + c2.radius >= (c2.center - c1.center).length()) {
    Point center = c1.center + (c2.center - c1.center) * c1.radius /
                                   (c1.radius + c2.radius);
    double radius = (c1.center - center).length() + c1.radius;

    Circle circle(center, radius);
    circle.radius += std::max(c1.radius, c2.radius);

    if (circle.radius < cfg_.max_circle_radius) {
      circle.point_sets.insert(circle.point_sets.end(), c1.point_sets.begin(),
                               c1.point_sets.end());
      circle.point_sets.insert(circle.point_sets.end(), c2.point_sets.begin(),
                               c2.point_sets.end());
      mergedCircle = circle;
      return true;
    }
  }
  return false;
}

void ObstacleDetect::publishObstacles() {
  // cds_msgs::obstaclesPtr obstaclesMsg(new cds_msgs::obstacles);
  std_msgs::Bool obstaclesMsg;

  // for (Segment const& s : segments_) {
  //   cds_msgs::segment_obstacle segment;
  //   segment.first_point.x = s.first_point.x;
  //   segment.first_point.y = s.first_point.y;
  //   segment.last_point.x = s.last_point.x;
  //   segment.last_point.y = s.last_point.y;
  //   obstaclesMsg->segments.emplace_back(segment);
  // }

  bool isObstacleDetect = false;
  for (Circle const& c : circles_) {
    cds_msgs::circle_obstacle circle;
    circle.center.x = c.center.x;
    circle.center.y = c.center.y;
    circle.radius = c.radius;

    if (circle.center.x > x_min && circle.center.x < x_max && circle.center.y >= 0 && circle.center.y < y_max)
    {
      isObstacleDetect = true;
    }
    // obstaclesMsg->circles.emplace_back(circle);
  }
  obstaclesMsg.data = isObstacleDetect;
  obstaclePub_.publish(obstaclesMsg);
}

}  // namespace obstacle_detect