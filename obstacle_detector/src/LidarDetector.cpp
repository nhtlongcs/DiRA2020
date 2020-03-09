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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "obstacle_detector/LidarDetector.hpp"
#include "utilities/figure_fitting.h"
#include "utilities/math_utilities.h"

LidarDetector::LidarDetector(ros::NodeHandle const& _nh,
                             ros::NodeHandle const& _pnh)
    : nh(_nh),
      pnh(_pnh),
      drServerCB(boost::bind(&LidarDetector::ReconfigureCB, this, _1, _2)),
      tfListener(tfBuffer, nh) {
  scanSub = nh.subscribe("/scan", 10, &LidarDetector::LidarCB, this);
  obstaclesPub = nh.advertise<cds_msgs::Obstacles>("raw_obstacles", 10);
  drServer.setCallback(drServerCB);
}

void LidarDetector::ReconfigureCB(
    obstacle_detector::LidarDetectorConfig& config, uint32_t level) {
  auto PrintChange = [](auto& val, auto const& newVal,
                        std::string const& nameVal) {
    if (val != newVal) {
      auto temp = val;
      val = newVal;
      ROS_INFO_STREAM("\033[92m" << nameVal << ": " << temp << " -> "
                                 << newVal);
    }
  };
  PrintChange(minGroupPoints, config.minGroupPoints_, "minGroupPoints");
  PrintChange(minRange, config.minRange_, "minRange");
  PrintChange(maxRange, config.maxRange_, "maxRange");
  PrintChange(maxGroupDistance, config.maxGroupDistance_, "maxGroupDistance");
  PrintChange(distanceProportion, config.distanceProportion_,
              "distanceProportion");
  PrintChange(maxSplitDistance, config.maxSplitDistance_, "maxSplitDistance");
  PrintChange(maxMergeSeparation, config.maxMergeSeparation_,
              "maxMergeSeparation");
  PrintChange(maxMergeSpread, config.maxMergeSpread_, "maxMergeSpread");
  PrintChange(maxCircleRadius, config.maxCircleRadius_, "maxCircleRadius");
  PrintChange(radiusEnlargement, config.radiusEnlargement_,
              "radiusEnlargement");
}

void LidarDetector::LidarCB(sensor_msgs::LaserScan::ConstPtr const& scanMsg) {
  baseFrameID = scanMsg->header.frame_id;
  stamp = scanMsg->header.stamp;
  double phi = scanMsg->angle_min;
  for (float const& r : scanMsg->ranges) {
    if (minRange <= r && r <= maxRange)
      pointArr.push_back(Point::fromPoolarCoords(r, phi));
    phi += scanMsg->angle_increment;
  }
  ProcessPoints();
}

void LidarDetector::ProcessPoints() {
  segArr.clear();
  circleArr.clear();

  GroupPoints();  // Grouping points simultaneously detects segments
  MergeSegments();

  DetectCircles();
  MergeCircles();

  PublishObstacles();
  pointArr.clear();
}

void LidarDetector::GroupPoints() {
  static double sin_dp = sin(2.0 * distanceProportion);

  PointSet pointSet;
  pointSet.begin = pointArr.begin();
  pointSet.end = pointArr.begin();
  pointSet.num_points = 1;
  pointSet.is_visible = true;

  for (PointIterator point = pointArr.begin()++; point != pointArr.end();
       ++point) {
    double range = (*point).length(),
           distance = (*point - *pointSet.end).length();

    if (distance < maxGroupDistance + range * distanceProportion) {
      pointSet.end = point;
      ++pointSet.num_points;
    } else {
      double prevRange = (*pointSet.end).length();

      // Heron's equation
      double p = (range + prevRange + distance) / 2.0,
             S = sqrt(p * (p - range) * (p - prevRange) * (p - distance));
      // Sine of angle between beams
      double sin_d = 2.0 * S / (range * prevRange);

      // TODO: This condition can be fulfilled if the point are on the opposite
      // sides of the scanner (angle = 180 deg). Needs another check.
      if (std::abs(sin_d) < sin_dp && range < prevRange)
        pointSet.is_visible = false;

      DetectSegments(pointSet);

      // Begin new point set
      pointSet.begin = point;
      pointSet.end = point;
      pointSet.num_points = 1;
      pointSet.is_visible = (std::abs(sin_d) > sin_dp || range < prevRange);
    }
  }
  DetectSegments(pointSet);  // Check the last point set too!
}

void LidarDetector::DetectSegments(PointSet const& point_set) {
  if (point_set.num_points < minGroupPoints) return;

  Segment segment(*point_set.begin,
                  *point_set.end);  // Use Iterative End Point Fit

  if (splitAndMerge) segment = fitSegment(point_set);

  PointIterator set_divider;
  double max_distance = 0.0, distance = 0.0;

  int split_index = 0;  // Natural index of splitting point (counting from 1)
  int point_index = 0;  // Natural index of current point in the set

  // Seek the point of division
  for (PointIterator point = point_set.begin; point != point_set.end; ++point) {
    ++point_index;

    if ((distance = segment.distanceTo(*point)) >= max_distance) {
      double r = (*point).length();

      if (distance > maxSplitDistance + r * distanceProportion) {
        max_distance = distance;
        set_divider = point;
        split_index = point_index;
      }
    }
  }
  // Split the set only if the sub-groups are not 'small'
  if (max_distance > 0.0 && split_index > minGroupPoints &&
      split_index < point_set.num_points - minGroupPoints) {
    set_divider = pointArr.insert(
        set_divider, *set_divider);  // Clone the dividing point for each group

    PointSet subset1, subset2;
    subset1.begin = point_set.begin;
    subset1.end = set_divider;
    subset1.num_points = split_index;
    subset1.is_visible = point_set.is_visible;

    subset2.begin = ++set_divider;
    subset2.end = point_set.end;
    subset2.num_points = point_set.num_points - split_index;
    subset2.is_visible = point_set.is_visible;

    DetectSegments(subset1);
    DetectSegments(subset2);
  } else {  // Add the segment
    if (!splitAndMerge) {
      segment = fitSegment(point_set);
    }
    segArr.push_back(segment);
  }
}

void LidarDetector::MergeSegments() {
  for (auto i = segArr.begin(); i != segArr.end(); ++i) {
    for (auto j = i; j != segArr.end(); ++j) {
      Segment mergedSegment;

      if (CompareSegments(*i, *j, mergedSegment)) {
        auto temp_itr = segArr.insert(i, mergedSegment);
        segArr.erase(i);
        segArr.erase(j);
        i = --temp_itr;  // Check the new segment against others
        break;
      }
    }
  }
}

bool LidarDetector::CompareSegments(Segment const& s1, Segment const& s2,
                                    Segment& merged_segment) {
  if (&s1 == &s2) return false;

  // Segments must be provided counter-clockwise
  if (s1.first_point.cross(s2.first_point) < 0.0)
    return CompareSegments(s2, s1, merged_segment);

  if (CheckSegmentsProximity(s1, s2)) {
    std::vector<PointSet> point_sets;
    point_sets.insert(point_sets.end(), s1.point_sets.begin(),
                      s1.point_sets.end());
    point_sets.insert(point_sets.end(), s2.point_sets.begin(),
                      s2.point_sets.end());

    Segment segment = fitSegment(point_sets);
    if (CheckSegmentsCollinearity(segment, s1, s2)) {
      merged_segment = segment;
      return true;
    }
  }
  return false;
}

bool LidarDetector::CheckSegmentsProximity(Segment const& s1,
                                           Segment const& s2) {
  return (s1.trueDistanceTo(s2.first_point) < maxMergeSeparation ||
          s1.trueDistanceTo(s2.last_point) < maxMergeSeparation ||
          s2.trueDistanceTo(s1.first_point) < maxMergeSeparation ||
          s2.trueDistanceTo(s1.last_point) < maxMergeSeparation);
}

bool LidarDetector::CheckSegmentsCollinearity(Segment const& segment,
                                              Segment const& s1,
                                              Segment const& s2) {
  return (segment.distanceTo(s1.first_point) < maxMergeSpread &&
          segment.distanceTo(s1.last_point) < maxMergeSpread &&
          segment.distanceTo(s2.first_point) < maxMergeSpread &&
          segment.distanceTo(s2.last_point) < maxMergeSpread);
}

void LidarDetector::DetectCircles() {
  for (auto segment = segArr.begin(); segment != segArr.end(); ++segment) {
    if (circlesFromVisibles) {
      bool segment_is_visible = true;
      for (const PointSet& ps : segment->point_sets) {
        if (!ps.is_visible) {
          segment_is_visible = false;
          break;
        }
      }
      if (!segment_is_visible) continue;
    }

    Circle circle(*segment);
    circle.radius += radiusEnlargement;

    if (circle.radius < maxCircleRadius) {
      circleArr.push_back(circle);

      if (discardConvertedSegments) {
        segment = segArr.erase(segment);
        --segment;
      }
    }
  }
}

void LidarDetector::MergeCircles() {
  for (auto i = circleArr.begin(); i != circleArr.end(); ++i) {
    for (auto j = i; j != circleArr.end(); ++j) {
      Circle merged_circle;

      if (CompareCircles(*i, *j, merged_circle)) {
        auto temp_itr = circleArr.insert(i, merged_circle);
        circleArr.erase(i);
        circleArr.erase(j);
        i = --temp_itr;
        break;
      }
    }
  }
}

bool LidarDetector::CompareCircles(Circle const& c1, Circle const& c2,
                                   Circle& merged_circle) {
  if (&c1 == &c2) return false;

  // If circle c1 is fully inside c2 - merge and leave as c2
  if (c2.radius - c1.radius >= (c2.center - c1.center).length()) {
    merged_circle = c2;
    return true;
  }
  // If circle c2 is fully inside c1 - merge and leave as c1
  if (c1.radius - c2.radius >= (c2.center - c1.center).length()) {
    merged_circle = c1;
    return true;
  }
  // If circles intersect and are 'small' - merge
  if (c1.radius + c2.radius >= (c2.center - c1.center).length()) {
    Point center = c1.center + (c2.center - c1.center) * c1.radius /
                                   (c1.radius + c2.radius);
    double radius = (c1.center - center).length() + c1.radius;

    Circle circle(center, radius);
    circle.radius += std::max(c1.radius, c2.radius);

    if (circle.radius < maxCircleRadius) {
      circle.point_sets.insert(circle.point_sets.end(), c1.point_sets.begin(),
                               c1.point_sets.end());
      circle.point_sets.insert(circle.point_sets.end(), c2.point_sets.begin(),
                               c2.point_sets.end());
      merged_circle = circle;
      return true;
    }
  }
  return false;
}

void LidarDetector::PublishObstacles() {
  cds_msgs::ObstaclesPtr obstacles_msg(new cds_msgs::Obstacles);
  obstacles_msg->header.stamp = stamp;
  if (transformCoordinates) {
    geometry_msgs::TransformStamped tfGeom;
    try {
      tfGeom = tfBuffer.lookupTransform(outFrameID, baseFrameID, stamp,
                                        ros::Duration(0.1));
    } catch (tf2::TransformException& ex) {
      ROS_WARN_STREAM("Could NOT transform: " << ex.what());
      return;
    }
    tf2::Stamped<tf2::Transform> transform;
    tf2::convert(tfGeom, transform);
    for (Segment& s : segArr) {
      s.first_point = transformPoint(s.first_point, transform);
      s.last_point = transformPoint(s.last_point, transform);
    }
    for (Circle& c : circleArr) c.center = transformPoint(c.center, transform);
    obstacles_msg->header.frame_id = outFrameID;
  } else
    obstacles_msg->header.frame_id = baseFrameID;

  for (Segment const& s : segArr) {
    cds_msgs::SegmentObstacle segment;
    segment.first_point.x = s.first_point.x;
    segment.first_point.y = s.first_point.y;
    segment.last_point.x = s.last_point.x;
    segment.last_point.y = s.last_point.y;
    obstacles_msg->segments.push_back(segment);
  }
  for (Circle const& c : circleArr) {
    cds_msgs::CircleObstacle circle;
    circle.center.x = c.center.x;
    circle.center.y = c.center.y;
    circle.velocity.x = 0.0;
    circle.velocity.y = 0.0;
    circle.radius = c.radius;
    circle.true_radius = c.radius - radiusEnlargement;
    obstacles_msg->circles.push_back(circle);
  }
  obstaclesPub.publish(obstacles_msg);
}
