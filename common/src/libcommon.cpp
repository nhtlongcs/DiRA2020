#include "common/libcommon.h"
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <array>
#include <cv_bridge/cv_bridge.h>
using namespace Eigen;

/////////////////////////////////////////////////////////////////////////////////////////////
// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++)
  {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++)
  {
    for (int i = 0; i < order; i++)
    {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++)
  {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

std::shared_ptr<LineParams> calcLineParams(const std::vector<cv::Point> &listPoint)
{
  Eigen::VectorXd xvals{listPoint.size()}, yvals{listPoint.size()};

  for (size_t i = 0; i < listPoint.size(); i++)
  {
    xvals[i] = listPoint[i].y;
    yvals[i] = listPoint[i].x;
  }

  auto coeff = polyfit(xvals, yvals, 2);
  auto result = std::make_shared<LineParams>();
  (*result)[0] = coeff[2];
  (*result)[1] = coeff[1];
  (*result)[2] = coeff[0];

  return result;
}

int getXByY(const LineParams &params, double y)
{
  return static_cast<int>(params[0] * y * y + params[1] * y + params[2]);
}

cv::Mat kmean(cv::Mat image, size_t clusterCount)
{
  cv::Mat floatImage;
  image.convertTo(floatImage, CV_32F);

  cv::Mat samples = floatImage.reshape(1, floatImage.total());
  clusterCount = std::min(samples.total(), clusterCount);

  cv::Mat labels;
  constexpr const int attempts = 5;
  cv::Mat centers;
  kmeans(samples, clusterCount, labels, cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10000, 0.0001), attempts, cv::KMEANS_PP_CENTERS, centers);

  std::array<int, 2> originalSize = {floatImage.rows, floatImage.cols};
  labels = labels.reshape(1, originalSize.size(), originalSize.data());

  cv::Mat new_image(image.size(), CV_8UC1);
  for (int row = 0; row < image.rows; row++)
  {
    for (int col = 0; col < image.cols; col++)
    {
      int cluster_idx = labels.at<int>(row, col);
      new_image.at<uchar>(row, col) = static_cast<int>(centers.at<float>(cluster_idx, 0));
    }
  }

  return new_image;
}

cv::Mat birdviewTransformation(const cv::Mat &src, int birdwidth, int birdheight, int skyline, int offsetLeft, int offsetRight, cv::Mat &returnM)
{
  int W = src.cols;
  int H = src.rows;

  cv::Point2f inQuad[4] = {
      cv::Point(0, skyline),
      cv::Point(W - 1, skyline),
      cv::Point(0, H - 1),
      cv::Point(W - 1, H - 1)};

  cv::Point2f outQuad[4] = {
      cv::Point(0, skyline),
      cv::Point(W - 1, skyline),
      cv::Point(offsetLeft, H - 1),
      cv::Point(W - offsetRight, H - 1)};

  returnM = getPerspectiveTransform(inQuad, outQuad);
  cv::Mat resultBirdview(birdheight, birdwidth, src.type());
  warpPerspective(src, resultBirdview, returnM, resultBirdview.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);

  return resultBirdview;
}

float errorAngle(const cv::Point &src, const cv::Point &dst)
{
  float X = dst.x - src.x;
  float Y = dst.y;
  float angle = atan2(Y, X) * 180.0 / CV_PI - 90.0;
  return angle;
}