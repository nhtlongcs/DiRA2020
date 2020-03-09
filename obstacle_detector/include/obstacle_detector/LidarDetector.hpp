#pragma once
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>

#include "cds_msgs/Obstacles.h"
#include "obstacle_detector/LidarDetectorConfig.h"
#include "utilities/circle.h"

class LidarDetector {
 public:
  LidarDetector(ros::NodeHandle const& nh, ros::NodeHandle const& pnh);
  ~LidarDetector() = default;

 private:
  ros::NodeHandle nh, pnh;
  ros::Subscriber scanSub;
  ros::Publisher obstaclesPub;
  std::string baseFrameID, outFrameID;
  ros::Time stamp;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  std::list<Point> pointArr;
  std::list<Segment> segArr;
  std::list<Circle> circleArr;

  void LidarCB(sensor_msgs::LaserScan::ConstPtr const& scanMsg);
  void PublishObstacles();

  void ProcessPoints();
  void GroupPoints();
  void DetectSegments(PointSet const& pointSet);
  void MergeSegments();
  bool CompareSegments(Segment const& s1, Segment const& s2,
                       Segment& mergedSegment);
  bool CheckSegmentsProximity(Segment const& s1, Segment const& s2);
  bool CheckSegmentsCollinearity(Segment const& segment, Segment const& s1,
                                 Segment const& s2);

  void DetectCircles();
  void MergeCircles();
  bool CompareCircles(Circle const& c1, Circle const& c2,
                      Circle& merged_circle);

  // Parameters
  bool splitAndMerge = true;
  bool circlesFromVisibles = true;
  bool discardConvertedSegments = true;
  bool transformCoordinates = true;
  int minGroupPoints = 5;
  double minRange = .15;
  double maxRange = 1.75;
  double maxGroupDistance = .1;
  double distanceProportion = .00628;
  double maxSplitDistance = .2;
  double maxMergeSeparation = .2;
  double maxMergeSpread = .2;
  double maxCircleRadius = .6;
  double radiusEnlargement = .3;

  dynamic_reconfigure::Server<obstacle_detector::LidarDetectorConfig> drServer;
  dynamic_reconfigure::Server<
      obstacle_detector::LidarDetectorConfig>::CallbackType drServerCB;
  void ReconfigureCB(obstacle_detector::LidarDetectorConfig& config,
                     uint32_t level);
};