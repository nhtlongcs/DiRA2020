#include "obstacle_detector/PclDetector.hpp"

#include <pcl/common/centroid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/filters/extract_indices.h>
#include <pcl_ros/filters/filter.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/segmentation/extract_clusters.h>

#include "cds_msgs/PclObstacles.h"

PclDetector::PclDetector(ros::NodeHandle const &_nh,
                         ros::NodeHandle const &_pnh)
    : nh(_nh),
      pnh(_pnh),
      cloud(new PointCloud<PointXYZ>),
      drServerCB(boost::bind(&PclDetector::ReconfigureCB, this, _1, _2)) {
  pclPub = nh.advertise<cds_msgs::PclObstacles>("pclCentroid", 1);
  pclSub =
      nh.subscribe("/camera/depth/points", 1, &PclDetector::PclCallback, this);
  drServer.setCallback(drServerCB);
}

void PclDetector::ReconfigureCB(obstacle_detector::PclDetectorConfig &config,
                                uint32_t level) {
  auto PrintChange = [](auto &val, auto const &newVal,
                        std::string const &nameVal) {
    if (val != newVal) {
      auto temp = val;
      val = newVal;
      ROS_INFO_STREAM("\033[92m" << nameVal << ": " << temp << " -> "
                                 << newVal);
    }
  };
  if (isShow != config.isShow_) {
    isShow = config.isShow_;
    if (isShow) {
      visualPub = nh.advertise<sensor_msgs::PointCloud2>("pclObjVisual", 1);
    } else {
      visualPub.shutdown();
    }
  }
  PrintChange(scanRange, config.scanRange_, "scanRange");
  PrintChange(sacThreshold, config.sacThreshold_, "sacThreshold");
  PrintChange(minObjHeight, config.minObjHeight_, "minObjHeight");
  PrintChange(tolerance, config.tolerance_, "tolerance");
}

void PclDetector::PclCallback(sensor_msgs::PointCloud2ConstPtr const &msg) {
  static int ib = 0;
  frameID = msg->header.frame_id;
  stamp = msg->header.stamp;
  fromROSMsg(*msg, *cloud);
  PreProcess();
  RemovePlane();
  ExtractObjs();
  if (isShow) {
    Visualize();
  }
  segArr.clear();
  ROS_INFO_STREAM("Calback: " << ++ib);
}

void PclDetector::PreProcess() {
  std::vector<int> mapping;
  removeNaNFromPointCloud(*cloud, *cloud, mapping);

  // Downsample
  VoxelGrid<PointXYZ> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.01f, 0.01f, 0.01f);
  vg.filter(*cloud);

  // Limit range
  PassThrough<PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, scanRange);
  // TOCHOSE pass.setFilterLimitsNegative (true);
  pass.filter(*cloud);
}

void PclDetector::RemovePlane() {
  boost::shared_ptr<std::vector<int> > inliers(new std::vector<int>);
  SampleConsensusModelPerpendicularPlane<PointXYZ>::Ptr model(
      new SampleConsensusModelPerpendicularPlane<PointXYZ>(cloud));

  model->setAxis(Eigen::Vector3f(0, 1, 0));
  model->setEpsAngle(M_PI / 12);
  RandomSampleConsensus<PointXYZ> ransac(model, sacThreshold);
  ransac.computeModel();
  ransac.getInliers(*inliers);

  // Eigen::VectorXf coeff, coeff_refined;
  // ransac.getModelCoefficients(coeff);
  // model->optimizeModelCoefficients(*inliers, coeff, coeff_refined);
  // model->selectWithinDistance(coeff_refined, minObjHeight, *inliers);

  ExtractIndices<PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setNegative(true);
  extract.setIndices(inliers);
  extract.filter(*cloud);
}

void PclDetector::ExtractObjs() {
  std::vector<PointIndices> clusterIndices;

  search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
  tree->setInputCloud(cloud);
  EuclideanClusterExtraction<PointXYZ> ec;
  ec.setClusterTolerance(tolerance);  // 2cm
  ec.setMinClusterSize(400);
  ec.setMaxClusterSize(20000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusterIndices);

  cds_msgs::PclObstaclesPtr pclObstacleMsgs(new cds_msgs::PclObstacles);
  pclObstacleMsgs->header.stamp = stamp;
  pclObstacleMsgs->header.frame_id = frameID;

  ExtractIndices<PointXYZ> extract;
  extract.setInputCloud(cloud);
  for (auto const &cluster : clusterIndices) {
    extract.setIndices(boost::make_shared<const PointIndices>(cluster));
    PointCloud<PointXYZ>::Ptr temp(new PointCloud<PointXYZ>);
    extract.filter(*temp);
    segArr.push_back(temp);
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*temp, centroid);
    geometry_msgs::Point p;
    p.x = centroid[0];
    p.y = centroid[1];
    p.z = centroid[2];
    pclObstacleMsgs->centroids.push_back(p);
  }
  pclPub.publish(pclObstacleMsgs);
}

void PclDetector::Visualize() {
  static int ia = 0;
  PointCloud<PointXYZRGB>::Ptr out(new PointCloud<PointXYZRGB>),
      tempOut(new PointCloud<PointXYZRGB>);
  out->header = cloud->header;
  uint8_t color[5][3] = {
      {255, 255, 0}, {255, 0, 0}, {255, 0, 255}, {0, 255, 0}, {0, 0, 255}};
  int i = 0;
  for (auto const &segment : segArr) {
    copyPointCloud(*segment, *tempOut);
    for (auto &&point : tempOut->points) {
      point.r = color[i][0];
      point.g = color[i][1];
      point.b = color[i][2];
    }
    ++i;
    if (i > 4) i = 0;
    *out += *tempOut;
  }
  visualPub.publish(*out);
  ROS_INFO_STREAM("Visual: " << ++ia);
}
