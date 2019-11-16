#include "utils.h"
#include <eigen3/Eigen/Dense>
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

