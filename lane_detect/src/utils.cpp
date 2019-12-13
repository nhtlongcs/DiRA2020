#include "utils.h"
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace Eigen;

/////////////////////////////////////////////////////////////////////////////////////////////
// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,int order) 
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

std::shared_ptr<LineParams> calcLineParams(const std::vector<cv::Point>& listPoint)
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

int getXByY(const LineParams& params, double y)
{
    return static_cast<int>(params[0]*y*y + params[1]*y + params[2]);
}


cv::Mat kmean(cv::Mat image, int clusterCount) {
    cv::Mat samples(image.rows * image.cols, 3, CV_32F);
    for( int y = 0; y < image.rows; y++ )
        for( int x = 0; x < image.cols; x++ )
        for( int z = 0; z < 3; z++)
            samples.at<float>(y + x*image.rows, z) = image.at<cv::Vec3b>(y,x)[z];

    cv::Mat labels;
    int attempts = 5;
    cv::Mat centers;
    kmeans(samples, clusterCount, labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.0001), attempts, cv::KMEANS_PP_CENTERS, centers );


    cv::Mat new_image( image.size(), image.type() );
    for( int y = 0; y < image.rows; y++ )
        for( int x = 0; x < image.cols; x++ )
        { 
        int cluster_idx = labels.at<int>(y + x*image.rows,0);
        new_image.at<cv::Vec3b>(y,x)[0] = centers.at<float>(cluster_idx, 0);
        new_image.at<cv::Vec3b>(y,x)[1] = centers.at<float>(cluster_idx, 1);
        new_image.at<cv::Vec3b>(y,x)[2] = centers.at<float>(cluster_idx, 2);
        }
    return new_image;
}


cv::Mat birdviewTransformation(const cv::Mat& src) {
    const int offsetX = 160;
    const int offsetY = 180;
    const int birdwidth = 300;
    const int birdheight = 330;
    const int skyline = 100;

    int W = src.size().width;
    int H = src.size().height;

    cv::Point2f srcVertices[4];
    srcVertices[0] = cv::Point(0, skyline);
    srcVertices[1] = cv::Point(W, skyline);
    srcVertices[2] = cv::Point(0, H);
    srcVertices[3] = cv::Point(W, H);
 
    cv::Point2f dstVertices[4];
    dstVertices[0] = cv::Point(0, 0);
    dstVertices[1] = cv::Point(birdwidth, skyline);
    dstVertices[2] = cv::Point(skyline, birdheight);
    dstVertices[3] = cv::Point(birdwidth - skyline, birdheight);

    cv::Mat M = getPerspectiveTransform(srcVertices, dstVertices);
    cv::Mat resultBirdview(birdwidth, birdheight, CV_8UC3);
    warpPerspective(src, resultBirdview, M, resultBirdview.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    return resultBirdview;
}