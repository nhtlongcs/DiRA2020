#include <algorithm>
#include "lane_detect/laneline.h"

static bool findPointInWindow(const cv::Mat &roi, cv::Point &outPoint,
                              int &outVal) {
  int maxCount = 0;
  cv::Point maxPoint;

  const int BIN_WIDTH = roi.cols / N_BINS;
  cv::Mat binImage;
  for (int bin = 0; bin < N_BINS; bin++) {
    cv::Rect binROI{bin * BIN_WIDTH, 0, BIN_WIDTH, roi.rows};
    binImage = roi(binROI);
    int nonZero = countNonZero(binImage);

    if (nonZero > maxCount) {
      maxCount = nonZero;
      maxPoint =
          cv::Point2i{int((bin + 0.5) * BIN_WIDTH), int(binImage.rows / 2)};
    }
  }

  if (maxCount == 0) {
    return false;
  }

  outPoint = maxPoint;
  outVal = maxCount;
  return true;
}

static cv::Rect getROITracking(const cv::Point &centerPoint) {
  return cv::Rect{centerPoint.x - W_TRACKING / 2,
                  centerPoint.y - H_TRACKING / 2, W_TRACKING, H_TRACKING};
}

LaneLine::LaneLine(cv::Mat &debugImage)
    : lineParams{nullptr}, debugImage{debugImage} {}

void LaneLine::update(const cv::Mat &lineImage) {
  this->lineImage = lineImage.clone();
  lineImageRect = cv::Rect{0, 0, lineImage.cols, lineImage.rows};

  if (isFound()) {
    // std::cout << getName() << "TRACK!" << std::endl;
    track();
  } else {
    // std::cout << getName() << "DETECT!" << std::endl;
    reset();
    detect();
  }
}

void LaneLine::reset() {
  confident_score = 0;
  lineParams = nullptr;
}

void LaneLine::track() {
  // TODO: fix me
  confident_score = 0;
  std::vector<cv::Point> trackingPoints;
  for (int y = lineImage.rows - H_TRACKING / 2 - 1; y > H_TRACKING / 2 + 1;
       y -= 5) {
    int x = getXByY(*lineParams, y * 1.0);
    cv::Point center{x, y};
    int nonZeroValue = 0;

    if (updateNewCenter(lineImage, center, &nonZeroValue)) {
      trackingPoints.push_back(center);
      confident_score += 1;
    }
    //  else if (nonZeroValue < 10)
    // {
    //     // Lost tracking
    //     break;
    // }
  }
  // std::cout << getName() << "Confident Score: " << confident_score <<
  // std::endl; std::cout.flush();

  if (trackingPoints.size() >= minPointTrack) {
    lineParams = std::make_shared<LineParams>(calcLineParams(trackingPoints));
  } else {
    reset();
    // detect();
  }
}

void LaneLine::detect() {
  cv::Point beginPoint;
  if (findBeginPoint(beginPoint)) {
    // cv::circle(debugImage, beginPoint, 30, getLaneColor(), -1);
    // cv::imshow("Debug", debugImage);
    // cv::waitKey(0);
    std::vector<cv::Point> points;
    std::vector<cv::Point> &&pointsUp = findPoints(beginPoint, 1);
    std::vector<cv::Point> &&pointsDown = findPoints(beginPoint, -1);
    std::move(pointsUp.begin(), pointsUp.end(), std::back_inserter(points));
    std::move(pointsDown.begin(), pointsDown.end(), std::back_inserter(points));

    if (points.size() > minPointDetect) {
      lineParams = std::make_shared<LineParams>(calcLineParams(points));
    }
  }
}

LineParams LaneLine::getLineParams() const { return *(this->lineParams); }

cv::Rect LaneLine::getDetectBeginPointRegion() const {
  return cv::Rect{0, 0, lineImage.cols, lineImage.rows};
}

std::vector<cv::Point> LaneLine::getPointsFromParams(
    const std::shared_ptr<LineParams> params) const {
  std::vector<cv::Point> points;
  if (params) {
    for (int y = 0; y < lineImage.rows; y++) {
      cv::Point point{getXByY(*lineParams, y), y};
      if (!isOutOfImage(point)) {
        points.push_back(point);
      }
    }
  }
  return points;
}

bool LaneLine::isFound() const { return lineParams != nullptr; }

std::vector<cv::Point> LaneLine::calcGradient(
    const std::vector<cv::Point> &points) const {
  std::vector<cv::Point> gradients;
  for (int i = 0; i < points.size() - 1; i++) {
    gradients.push_back(points[i + 1] - points[i]);
  }
  return gradients;
}

bool LaneLine::isOutOfImage(const cv::Point &point) const {
  return point.x < 0 || point.x >= lineImage.cols || point.y < 0 ||
         point.y >= lineImage.rows;
}

void LaneLine::swap(std::shared_ptr<LaneLine> other) {
  std::swap(other->lineParams, this->lineParams);
}

void LaneLine::setFindBeginPointRegion(int offset, int width) {
  this->offsetX = offset;
  this->width = width;
}

bool LaneLine::findBeginPoint(cv::Point &returnPoint) const {
  cv::Rect &&regionRect = getDetectBeginPointRegion();
  // std::cout << "RightRegionRect: " << regionRect << ' ';
  // std::cout.flush();

  const cv::Mat region = lineImage(regionRect);

  for (int center_y = region.rows - 1 - H_TRACKING / 2;
       center_y > H_TRACKING / 2 + 1; center_y -= H_TRACKING) {
    int maxNonZero = 0;
    cv::Point maxPoint;

    for (int center_x = W_TRACKING / 2 + 1;
         center_x < region.cols - W_TRACKING / 2 - 1; center_x += W_TRACKING) {
      cv::Point center{center_x, center_y};
      //         std::cout << "CenterBefore: " << center << ' ';
      // std::cout.flush();

      int nonZeroValue = 0;
      updateNewCenter(region, center, &nonZeroValue);

      //         std::cout << "CenterAfter: " << center << ' ';
      // std::cout.flush();

      // ROS_INFO("nonZeroValue: %d", nonZeroValue);
      // std::cout.flush();
      if (nonZeroValue > maxNonZero) {
        maxNonZero = nonZeroValue;
        maxPoint = center;
        // std::cout << "Max: " << center << ' ';
      }
    }

    if (maxNonZero > maxNonZeroThreshold) {
      returnPoint = regionRect.tl() + maxPoint;
      //         std::cout << "Return: " << returnPoint << std::endl;
      // std::cout.flush();

      return true;
    }
  }

  return false;
}

std::vector<cv::Point> LaneLine::findPoints(const cv::Point &beginPoint,
                                            int direct) const {
  std::vector<cv::Point> result;
  cv::Point nextPoint{beginPoint.x, beginPoint.y + direct * H_TRACKING / 2};

  while (true) {
    if (!updateNewCenter(lineImage, nextPoint, nullptr)) {
      break;
    }
    result.push_back(nextPoint);
    nextPoint.y += direct * H_TRACKING / 3;
  }
  return result;
}

bool LaneLine::updateNewCenter(const cv::Mat &region, cv::Point &center,
                               int *const nonZeroValue) const {
  cv::Rect &&roiTracking = getROITracking(center);
  // std::cout << "ROI: " << roiTracking << " LineImageRect " << lineImageRect
  // << ' ' << (roiTracking & lineImageRect) << std::endl;

  if ((roiTracking & lineImageRect) != roiTracking) {
    // std::cout << "OUTTTT";
    return false;
  }
  // cv::Mat d;
  // cv::cvtColor(region, d, cv::COLOR_GRAY2BGR);
  // cv::rectangle(d, roiTracking, getLaneColor(), 2);
  // cv::imshow(getName() + "Tracking", d);
  // cv::waitKey(1);

  // cv::Mat d;
  // cv::cvtColor(region, d, cv::COLOR_GRAY2BGR);
  // cv::rectangle(d, roiTracking, getLaneColor(), 2);
  // cv::imshow("DebugRight", d);
  // cv::waitKey(2);

  const cv::Mat trackingImage = region(roiTracking);

  cv::Point maxPointResult;
  int value = 0;
  bool found = findPointInWindow(trackingImage, maxPointResult, value);
  if (found) {
    center = roiTracking.tl() + maxPointResult;
    if (nonZeroValue) {
      *nonZeroValue = value;
    }
  }
  return found;
}

std::vector<cv::Point> LaneLine::moveByGradient(int distance) const {
  std::vector<cv::Point> points = getPointsFromParams(lineParams);
  std::vector<cv::Point> gradients = calcGradient(points);
  for (int i = 0; i < points.size() - 1; i++) {
    points[i] += calcPerpendicular(gradients[i]) * distance;
  }
  return points;
}

bool LaneLine::recoverFrom(const LaneLine &lane, int laneWidth) {
  if (!lane.isFound() || laneWidth <= 0) {
    return false;
  }

  std::vector<cv::Point> &&movedPoints = lane.moveByGradient(laneWidth);
  auto newParams = calcLineParams(movedPoints);
  lineParams = std::make_shared<LineParams>(newParams);
  track();
  return isFound();
}

void LaneLine::setLineParams(const LineParams &other) {
  if (!this->lineParams) {
    this->lineParams = std::make_shared<LineParams>(other);
  }
}

//////////////////////////////////////////

LeftLane::LeftLane(cv::Mat &debugImage) : LaneLine{debugImage} {}

cv::Rect LeftLane::getDetectBeginPointRegion() const {
  // int real_width = std::min(width, lineImage.cols - offsetX - 1);
  // return cv::Rect{offsetX, 0, real_width, lineImage.rows};
  return cv::Rect{0, 0, lineImage.cols / 2, lineImage.rows};
}

cv::Scalar LeftLane::getLaneColor() const {
  return cv::Scalar{0, 255, 0};  // Green
}

void LeftLane::showLinePoints(cv::Mat &drawImage) const {
  for (int y = 0; y < drawImage.rows; y += 20) {
    const cv::Point point{getXByY(*lineParams, y * 1.0), y};
    circle(drawImage, point, 5, getLaneColor(), -1, 1, 0);
  }
}

cv::Point LeftLane::calcPerpendicular(const cv::Point &point) const {
  return cv::Point{point.y, -point.x} /
         sqrt(point.y * point.y + point.x * point.x);
}

void LeftLane::visualizeTrackingPoint(cv::Mat &image,
                                      const std::vector<cv::Point> &points) {
  // for (const auto& point : points)
  // {
  //     cv::circle(image, point, 3, getLaneColor(), -1);
  // }
  // cv::imshow("Left Tracking", image);
  // cv::waitKey(1);
}

// bool LeftLane::findBeginPoint(cv::Point &returnPoint) const
// {
//     cv::Rect &&regionRect = getDetectBeginPointRegion();
//     // std::cout << "RegionRect: " << regionRect << ' ';
//     // std::cout.flush();

//     const cv::Mat region = lineImage(regionRect);

//     for (int center_y = region.rows - 1 - H_TRACKING / 2; center_y >
//     H_TRACKING / 2; center_y -= H_TRACKING)
//     {
//         int maxNonZero = 0;
//         cv::Point maxPoint;

//         for (int center_x = W_TRACKING / 2 + 1; center_x < region.cols -
//         W_TRACKING / 2 - 1; center_x += W_TRACKING)
//         {
//             cv::Point center{center_x, center_y};
//             int nonZeroValue = 0;
//             updateNewCenter(region, center, &nonZeroValue);
//             // std::cout << "Center: " << center << ' ';
//             // std::cout.flush();

//             // cv::imshow("LeftRegion", region);
//             // cv::waitKey(0);
//             // ROS_INFO("nonZeroValue: %d", nonZeroValue);
//             if (nonZeroValue > maxNonZero)
//             {
//                 maxNonZero = nonZeroValue;
//                 maxPoint = center;
//             }
//         }

//         if (maxNonZero > maxNonZeroThreshold)
//         {
//             returnPoint = regionRect.tl() + maxPoint;
//             // std::cout << "Return: " << returnPoint << std::endl;
//             // std::cout.flush();

//             return true;
//         }
//     }

//     return false;
// }

// bool LeftLane::updateNewCenter(const cv::Mat &region, cv::Point &center, int
// *const nonZeroValue) const
// {
//     const cv::Rect roiTracking = getROITracking(center);

//     if ((roiTracking & lineImageRect) != roiTracking)
//     {
//         return false;
//     }

//     // cv::Mat d;
//     // cv::cvtColor(region, d, cv::COLOR_GRAY2BGR);
//     // cv::rectangle(d, roiTracking, getLaneColor(), 2);
//     // cv::imshow("LeftUpdate", d);
//     // cv::waitKey(1);

//     const cv::Mat trackingImage = region(roiTracking);

//     cv::Point maxPointResult;
//     int value = 0;
//     bool found = findPointHasBiggestValueInBin(trackingImage, maxPointResult,
//     value); if (found)
//     {
//         center = roiTracking.tl() + maxPointResult;
//         if (nonZeroValue)
//         {
//             *nonZeroValue = value;
//         }
//     }
//     return found;
// }

//////////////////////////////////////////

RightLane::RightLane(cv::Mat &debugImage) : LaneLine{debugImage} {}

cv::Rect RightLane::getDetectBeginPointRegion() const {
  // int x = std::max(0, lineImage.cols - offsetX - width - 1);
  // int real_width = std::min(width, lineImage.cols - x);
  // return cv::Rect{x, 0, real_width, lineImage.rows};
  // return cv::Rect{lineImage.cols * 1 / 3, 0, lineImage.cols * 2 / 3,
  // lineImage.rows};
  return cv::Rect{lineImage.cols / 2, 0, lineImage.cols / 2, lineImage.rows};
}

cv::Scalar RightLane::getLaneColor() const { return cv::Scalar{255, 0, 0}; }

void RightLane::showLinePoints(cv::Mat &drawImage) const {
  for (int y = 7; y < drawImage.rows; y += 20) {
    const cv::Point point{getXByY(*lineParams, y * 1.0), y};
    circle(drawImage, point, 5, getLaneColor(), -1, 1, 0);
  }
}

cv::Point RightLane::calcPerpendicular(const cv::Point &point) const {
  return {-point.y, point.x};
}

void RightLane::visualizeTrackingPoint(cv::Mat &image,
                                       const std::vector<cv::Point> &points) {
  for (const auto &point : points) {
    cv::circle(image, point, 3, getLaneColor(), -1);
  }
  // cv::imshow("Right Tracking", image);
  // cv::waitKey(1);
}
