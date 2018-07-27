#ifndef HOMOGRAPHY_COST_H_
#define HOMOGRAPHY_COST_H_

#include "ceres/ceres.h"
#include "opencv2/opencv.hpp"
#include "vector"


class OnlineCalibrateCost {
public:
  bool operator()(const double* online_param, double* residuals) const;
  static ceres::CostFunction* Create(double x1,
                                     double y1,
                                     double x2,
                                     double y2, int pos, 
                                     std::vector<std::vector<double>> rs, std::vector<std::vector<double>> ts,
                                     float viewrange, float pix_dis);

  ~OnlineCalibrateCost(){};

private:
  OnlineCalibrateCost() {};
  double observed_x1_;
  double observed_y1_;

  double observed_x2_;
  double observed_y2_;

  int pos_;

  std::vector<std::vector<double>> vec_r_, vec_t_;
};


#endif
