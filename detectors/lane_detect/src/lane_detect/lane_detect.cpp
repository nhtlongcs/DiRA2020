#include <common/libcommon.h>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <geometry_msgs/Point32.h>

#include "cds_msgs/lane.h"
#include "lane_detect/lane_detect.h"
#include "lane_detect/laneline.h"
#include "lane_detect/LaneConfig.h"

using namespace cv;
using namespace std;

constexpr const char *CONF_BIRDVIEW_WINDOW = "Birdview";
constexpr const char *LANE_WINDOW = "LaneDetect";

LaneDetect::LaneDetect(bool isDebug)
    : _nh{"lane_detect"},
      _image_transport{_nh},
      _configServer{_nh},
      isDebug{isDebug},
      left{debugImage},
      right{debugImage} {
  _isTurnableSrv =
      _nh.advertiseService("IsTurnable", &LaneDetect::isTurnable, this);

  std::string lane_seg_topic, road_seg_topic, depth_topic, lane_output_topic,
      transport_hint_str;
  ROS_ASSERT(ros::param::get("/lane_segmentation_topic", lane_seg_topic));
  ROS_ASSERT(ros::param::get("/road_segmentation_topic", road_seg_topic));
  ROS_ASSERT(ros::param::get("/lane_detect_topic", lane_output_topic));
  ROS_ASSERT(ros::param::get("/depth_topic", depth_topic));
  ROS_ASSERT(ros::param::get("/transport_hint", transport_hint_str));

  _lanePub = _nh.advertise<cds_msgs::lane>(lane_output_topic, 1);
  image_transport::TransportHints transport_hint{transport_hint_str};
  _laneSegSub = _image_transport.subscribe(lane_seg_topic, 1,
                                           &LaneDetect::updateLaneSegCallback,
                                           this, transport_hint);
  _roadSegSub = _image_transport.subscribe(road_seg_topic, 1,
                                           &LaneDetect::updateRoadSegCallback,
                                           this, transport_hint);
  _depthImageSub = _image_transport.subscribe(
      depth_topic, 1, &LaneDetect::updateDepthCallback, this, transport_hint);
  _configServer.setCallback(std::bind(&LaneDetect::configCallback, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));

  std::string resetSrvTopic, recoverSrvTopic;
  ROS_ASSERT(ros::param::get("/reset_lane_srv", resetSrvTopic));
  ROS_ASSERT(ros::param::get("/recover_lane_srv", recoverSrvTopic));

  _resetLaneSrv =
      _nh.advertiseService(resetSrvTopic, &LaneDetect::resetLaneSrv, this);
  _recoverLaneSrv =
      _nh.advertiseService(recoverSrvTopic, &LaneDetect::recoverLaneSrv, this);
}

LaneDetect::~LaneDetect() { cv::destroyAllWindows(); }

void LaneDetect::configCallback(lane_detect::LaneConfig &config,
                                uint32_t level) {
  usebirdview = config.use_birdview;
  showDetectRegion = config.show_detect_region;
  dropTop = config.drop_top;
  initLaneWidth = config.init_lane_width;
  birdwidth = config.birdwidth;
  birdheight = config.birdheight;
  skyline = config.skyline;
  offsetLeft = config.offset_birdview_left;
  offsetRight = config.offset_birdview_right;
  left.setFindBeginPointRegion(config.offset_left, config.left_width);
  right.setFindBeginPointRegion(config.offset_right, config.right_width);
}

bool LaneDetect::isNeedRedetect(const LaneLine &left,
                                const LaneLine &right) const {
  if (left.isFound() && right.isFound()) {
    std::vector<int> diff;
    const auto &leftParams = left.getLineParams();
    const auto &rightParams = right.getLineParams();
    for (size_t y = 0; y < birdview.rows; y++) {
      int xleft = getXByY(leftParams, y * 1.0, nullptr, nullptr);
      int xright = getXByY(rightParams, y * 1.0, nullptr, nullptr);
      diff.push_back(abs(xleft - xright));
    }
    return std::any_of(diff.begin(), diff.end(), [this](const int &amount) {
      return amount < this->initLaneWidth;
    });
  }
  return false;
}

bool LaneDetect::resetLaneSrv(cds_msgs::ResetLaneRequest &req,
                              cds_msgs::ResetLaneResponse &res) {
  if (req.lane == req.LEFT) {
    this->left.reset();
    res.left_params.clear();
  } else if (req.lane == req.RIGHT) {
    this->right.reset();
    res.right_params.clear();
  } else {
    this->left.reset();
    this->right.reset();
    res.left_params.clear();
    res.right_params.clear();
  }
  return true;
}

bool LaneDetect::recoverLaneSrv(cds_msgs::RecoverLaneRequest &req,
                                cds_msgs::RecoverLaneResponse &res) {
  ROS_DEBUG("Receive Recover Lane Request");
  if (req.lane == req.LEFT &&
      this->left.recoverFrom(right, this->initLaneWidth)) {
    res.params.insert(res.params.begin(), left.getLineParams().begin(),
                      left.getLineParams().end());
    ROS_DEBUG("Recover Left lane OK");
    return true;
  } else if (req.lane == req.RIGHT &&
             this->right.recoverFrom(left, this->initLaneWidth)) {
    res.params.insert(res.params.begin(), right.getLineParams().begin(),
                      right.getLineParams().end());
    auto params = right.getLineParams();
    return true;
  }
  ROS_DEBUG("Recover FAIL");
  return false;
}

void LaneDetect::detect() {
  this->birdview =
      birdviewTransformation(this->binary, birdwidth, birdheight, skyline,
                             offsetLeft, offsetRight, birdviewTransformMatrix);

  birdview(cv::Rect(0, 0, birdview.cols, dropTop)) = cv::Scalar{0};

  if (isDebug) {
    cv::cvtColor(this->birdview, debugImage, cv::COLOR_GRAY2BGR);
  }

  left.update(birdview);
  right.update(birdview);

  if (roadSeg.empty()) {
    return;
  }

  if (right.isFound() && !isCorrect(&right, roadSeg, RIGHT)) {
    ROS_DEBUG("RightLane is not correct");
    right.reset();
  }

  if (left.isFound() && !isCorrect(&left, roadSeg, LEFT)) {
    ROS_DEBUG("LeftLane is not correct");
    left.reset();
  }

  if (isNeedRedetect(left, right)) {
    ROS_DEBUG("Need Redetect due to closed lanes");
    if (left.getConfScore() < right.getConfScore()) {
      ROS_DEBUG("Reset Left");
      left.reset();
      // left.update(birdview);
      if (left.recoverFrom(right, initLaneWidth)) {
        countRedetectLane = 0;
        // std::cout << "Recovered left from right!" <<  std::endl;
      } else {
        countRedetectLane++;
        // std::cout << "Cannot recover left from right!" <<  std::endl;
      }
    } else if (right.getConfScore() < left.getConfScore()) {
      ROS_DEBUG("Reset Right");
      right.reset();
      // right.update(birdview);
      if (right.recoverFrom(left, initLaneWidth)) {
        // std::cout << "Recovered right from left" << std::endl;
        countRedetectLane = 0;
      } else {
        countRedetectLane++;
        // std::cout << "Cannot recover right from left" << std::endl;
      }
    }
    if (isNeedRedetect(left, right)) redetect();

    if (isNeedRedetect(left, right)) {
      if (left.getConfScore() < right.getConfScore())
        left.reset();
      else
        right.reset();
    }
  }
  // else if (left.isFound()){
  //     right.recoverFrom(left,initLaneWidth*2);
  // }
  // else if (right.isFound()){
  //     left.recoverFrom(right,initLaneWidth*2);
  // }

  // if (left.isFound() && right.isFound())
  // {

  // } else if (left.isFound())
  // {

  // }

  if (isDebug) {
    // if (left.isFound()) {
    //   left.showLinePoints(debugImage);
    // }
    // if (right.isFound()) {
    //   right.showLinePoints(debugImage);
    // }
    // cv::imshow(LANE_WINDOW, debugImage);
    // -cv::waitKey(1);
  }
}

bool LaneDetect::isCorrect(LaneLine *lane, const cv::Mat &roadSeg,
                           int direct) const {
  cv::Mat M;
  cv::Mat roadBirdview = birdviewTransform(roadSeg, M);
  cv::Mat drawBirdview;
  cv::cvtColor(roadBirdview, drawBirdview, cv::COLOR_GRAY2BGR);
  // cv::imshow("Road Birdview", roadBirdview);
  // cv::waitKey(1);

  auto laneParams = lane->getLineParams();
  // ROS_INFO("1");
  int sumLeftArea = 0, sumRightArea = 0;
  int height = 10;
  cv::Rect imageRect{0, 0, roadBirdview.cols - 1, roadBirdview.rows - 1};
  for (int y = 0; y < roadBirdview.rows; y += height) {
    // ROS_INFO("2");
    int x = std::max(0, getXByY(laneParams, y * 1.0));
    cv::Rect roiLeft{0, y, x, height};  // left side
    roiLeft &= imageRect;
    // ROS_INFO("3");
    // ROS_DEBUG_STREAM("roiLeft " << roiLeft);
    // cv::rectangle(drawBirdview, roiLeft, cv::Scalar{0, 255, 0}, 3);
    if (roiLeft.height * roiLeft.width != 0) {
      sumLeftArea += cv::countNonZero(roadBirdview(roiLeft));
    }
  }
  // ROS_INFO("4");
  sumRightArea = cv::countNonZero(roadBirdview) - sumLeftArea;
  // ROS_DEBUG_STREAM("sumArea " << sumLeftArea << ' ' << sumRightArea);

  // ROS_INFO("5");
  // cv::imshow("Road Birdview", drawBirdview);
  // cv::waitKey(1);
  // ROS_INFO("6");
  return ((direct == LEFT) && (sumRightArea > sumLeftArea)) ||
         ((direct == RIGHT) && (sumRightArea < sumLeftArea));
}

void LaneDetect::redetect() {
  this->birdview =
      birdviewTransformation(this->binary, birdwidth, birdheight, skyline,
                             offsetLeft, offsetRight, birdviewTransformMatrix);
  birdview(cv::Rect(0, 0, birdview.cols, dropTop)) = cv::Scalar{0};

  cv::Mat newLaneDebug;
  cv::cvtColor(this->birdview, newLaneDebug, cv::COLOR_GRAY2BGR);

  LeftLane newLeft{newLaneDebug};
  RightLane newRight{newLaneDebug};

  newLeft.update(birdview);
  newRight.update(birdview);

  if (newLeft.isFound() && newRight.isFound()) {
    if (isNeedRedetect(newLeft, newRight)) {
      // std::cout << "Plan 2 still need redetect!!!" << std::endl;
      if (left.getConfScore() < right.getConfScore()) {
        // std::cout << "Reset LEFT" << std::endl;
        left.reset();
      } else if (right.getConfScore() < left.getConfScore()) {
        // std::cout << "Reset RIGHT" << std::endl;
        right.reset();
      }
    } else {
      // std::cout << "Use new lanes" << std::endl;
      left.setLineParams(newLeft.getLineParams());
      right.setLineParams(newRight.getLineParams());
    }
  } else if (newLeft.isFound()) {
    left.setLineParams(newLeft.getLineParams());
    right.reset();
  } else if (newRight.isFound()) {
    left.reset();
    right.setLineParams(newRight.getLineParams());
  } else {
    left.reset();
    right.reset();
    // std::cout << "BOTH LANE NOT FOUND!" << std::endl;
  }

  if (right.isFound() && !isCorrect(&right, roadSeg, RIGHT)) {
    ROS_DEBUG("RightLane is not correct");
    right.reset();
  }

  if (left.isFound() && !isCorrect(&left, roadSeg, LEFT)) {
    ROS_DEBUG("LeftLane is not correct");
    left.reset();
  }
}

cv::Mat LaneDetect::extractFeatureY(cv::Mat img) const {
  cv::Mat binaryTemp = img.clone();

  cv::Mat skel(binaryTemp.size(), CV_8UC1, cv::Scalar(0));
  {
    cv::Mat temp;
    cv::Mat eroded;

    cv::Mat element =
        cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    bool done;
    do {
      cv::erode(binaryTemp, eroded, element);
      cv::dilate(eroded, temp, element);  // temp = open(binaryTemp)
      cv::subtract(binaryTemp, temp, temp);
      cv::bitwise_or(skel, temp, skel);
      eroded.copyTo(binaryTemp);

      done = (cv::countNonZero(binaryTemp) == 0);
    } while (!done);
  }
  return skel;
}
bool LaneDetect::isTurnable(cds_msgs::IsTurnableRequest &req,
                            cds_msgs::IsTurnableResponse &res) {
  if (this->binary.empty()) {
    return false;
  }

  ROS_DEBUG("Service IsTurnable is called");

  // cv::imshow("binary", binary);

  cv::Mat nearRegionMask;
  cv::inRange(depth, cv::Scalar{lowThreshold * 1.0},
              cv::Scalar{highThreshold * 1.0}, nearRegionMask);
  // cv::imshow("nearRegionMask", nearRegionMask);

  cv::Mat binaryNearRegion = binary & nearRegionMask;
  // cv::imshow("binaryNearRegion", binaryNearRegion);

  // cv::Mat sobely;
  // cv::Mat skel = extractFeatureY(roadSegmentation);
  // cv::imshow("Sobely", skel);

  // cv::Sobel(skel, sobely, CV_8U, 0, 1);
  // cv::imshow("Sobely", binary);
  int cnt = 0;
  std::vector<cv::Vec4f> lines;
  cv::HoughLinesP(binaryNearRegion, lines, 1, CV_PI / 180, votes, minLinlength,
                  maxLineGap);
  // cv::HoughLinesP(skel, lines, 1, CV_PI / 180, votes, minLinlength,
  // maxLineGap);

  float Slope_average = 0;
  for (const cv::Vec4f &line : lines) {
    float slope = 0;
    if (abs(line[0] - line[2]) < 0.01f) {
      // vertical line, tan = inf
      slope = 0;
      continue;
    } else {
      slope = (line[1] - line[3]) / (line[0] - line[2]);
    }

    // std::cout << slope << std::endl;
    if (fabs(slope) < 0.1) cnt++;
  }
  // std::cout << cnt << std::endl;
  if (cnt > 4) {
    res.ok = true;
  } else {
    res.ok = false;
  }
  return true;
}

cv::Mat LaneDetect::birdviewTransform(cv::Mat inputImage,
                                      cv::Mat &resultM) const {
  cv::Mat birdviewResult =
      birdviewTransformation(inputImage, birdwidth, birdheight, skyline,
                             offsetLeft, offsetRight, resultM);
  return birdviewResult;
}

Mat LaneDetect::shadow(const Mat &src) {
  Mat shadow, hsv;
  cvtColor(src, hsv, cv::COLOR_BGR2HSV);
  inRange(hsv,
          Scalar(minLaneInShadow[0], minLaneInShadow[1], minLaneInShadow[2]),
          Scalar(maxLaneInShadow[1], maxLaneInShadow[1], maxLaneInShadow[2]),
          shadow);
  return shadow;
}

Point LaneDetect::Hough(const Mat &img, const Mat &src) {
  vector<Vec4f> lines;
  Mat HoughTransform = Mat::zeros(src.size(), CV_8UC3);
  float cntL = 0.0, cntR = 0.0;
  float leftSlope_average = 0.0;
  float rightSlope_average = 0.0;
  float leftYintercept_average = 0.0;
  float rightYintercept_average = 0.0;

  HoughLinesP(img, lines, 1, CV_PI / 180, votes, minLinlength, maxLineGap);

  for (int i = 0; i < (int)lines.size(); ++i) {
    Vec4f t = lines[i];
    float slope = (t[1] - t[3]) / (t[0] - t[2]);
    float Yintercept = t[1] - slope * t[0];

    if (slope < 0.0) {
      leftSlope_average += slope, leftYintercept_average += Yintercept,
          cntL += 1.0;
    } else {
      rightSlope_average += slope, rightYintercept_average += Yintercept,
          cntR += 1.0;
    }
  }

  if (cntL > 0) {
    leftSlope_average /= cntL;
    leftYintercept_average /= cntL;
    drawLine(leftSlope_average, leftYintercept_average, HoughTransform);
  }

  if (cntR > 0) {
    rightSlope_average /= cntR;
    rightYintercept_average /= cntR;
    drawLine(rightSlope_average, rightYintercept_average, HoughTransform);
  }

  float midY = offsetY;
  float midX = offsetX;

  if (fabs(rightSlope_average) < 0.05) cntR = 0;
  if (fabs(leftSlope_average) < 0.05) cntL = 0;

  if (cntL == 0.0 && cntR > 0) {
    midX = (midY - rightYintercept_average) / rightSlope_average;
  } else if (cntL > 0 && cntR == 0.0) {
    midX = (midY - leftYintercept_average) / leftSlope_average;
  } else if (cntL > 0 && cntR > 0) {
    midX = (((midY - leftYintercept_average) / leftSlope_average) +
            ((midY - rightYintercept_average) / rightSlope_average)) /
           2.0;
  }

  addWeighted(src, 0.5, HoughTransform, 1, 1, HoughTransform);
  circle(HoughTransform, Point(midX, midY), 3, Scalar(0, 0, 255), -1);
  circle(HoughTransform, Point(offsetX, offsetY), 3, Scalar(0, 255, 0), -1);

  return Point(midX, midY);
}

void LaneDetect::drawLine(float slope, float y_intercept, Mat &HoughTransform) {
  float y0 = 240.0;
  float y1 = 1.0;
  float x0 = (y0 - y_intercept) / slope;
  float x1 = (y1 - y_intercept) / slope;
  line(HoughTransform, Point(x0, y0), Point(x1, y1), Scalar(0, 255, 0), 2,
       LINE_AA);
}

Mat LaneDetect::ROI(const Mat &src) {
  // Crop region of interest (ROI)
  int W = src.size().width;
  int H = src.size().height;
  Mat RegionOfInterest;
  Mat mask = Mat::zeros(src.size(), CV_8UC1);

  vector<Point> vertices;
  vertices.push_back(Point(0, skyline));
  vertices.push_back(Point(W, skyline));
  vertices.push_back(Point(W, H));
  vertices.push_back(Point(0, H));

  fillConvexPoly(mask, vertices, Scalar(255, 255, 255), 8, 0);
  bitwise_and(src, mask, RegionOfInterest);

  // imshow("RegionOfInterest", RegionOfInterest);
  return RegionOfInterest;
}

Mat LaneDetect::morphological(const Mat &img) {
  Mat dst;

  dilate(img, dst, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  erode(dst, dst, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

  return dst;
}

Mat LaneDetect::preprocess(const Mat &src) {
  Mat hsv, binary;

  cvtColor(src, hsv, COLOR_BGR2HSV);
  inRange(hsv, Scalar(minThreshold[0], minThreshold[1], minThreshold[2]),
          Scalar(maxThreshold[0], maxThreshold[1], maxThreshold[2]), binary);

  return binary;
}

void LaneDetect::updateLaneSegCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    if (!cv_ptr->image.empty()) {
      this->binary = cv_ptr->image.clone();
      detect();
      publishMessage();
    }
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }
}

void LaneDetect::updateRoadSegCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    if (!cv_ptr->image.empty()) {
      this->roadSeg = cv_ptr->image.clone();
    }
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }
}

void LaneDetect::updateDepthCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding.c_str());
    if (!cv_ptr->image.empty()) {
      cv_ptr->image.convertTo(this->depth, CV_8UC1, 1.0f / 256);
      // this->depth = cv_ptr->image.clone();
    }
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }
}

void LaneDetect::publishMessage() const {
  cds_msgs::lane lane_msg;
  if (left.isFound()) {
    const auto &params = left.getLineParams();
    lane_msg.left_params.insert(lane_msg.left_params.begin(), params.begin(),
                                params.end());
  }
  if (right.isFound()) {
    const auto &params = right.getLineParams();
    lane_msg.right_params.insert(lane_msg.right_params.begin(), params.begin(),
                                 params.end());
  }
  _lanePub.publish(lane_msg);
}
