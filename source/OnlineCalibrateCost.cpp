#include "OnlineCalibrateCost.h"
#include "eigen3/Eigen/Dense"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

using namespace ceres;
using namespace std;
using namespace cv;

ceres::CostFunction* OnlineCalibrateCost::Create(double x1, double y1, double x2, double y2, int pos,
                                                std::vector<std::vector<double>> rs, std::vector<std::vector<double>> ts,
                                                float viewrange, float pix_dis) {   
  std::unique_ptr<OnlineCalibrateCost> cost_functor(
      new OnlineCalibrateCost);
  int pixes = round(viewrange/pix_dis);
  int half_mid = pixes/2;
  cost_functor->observed_x1_ = 1.0*(x1-half_mid)/pixes*viewrange;
  cost_functor->observed_y1_ = 1.0*(y1-half_mid)/pixes*viewrange;
  cost_functor->observed_x2_ = 1.0*(x2-half_mid)/pixes*viewrange;
  cost_functor->observed_y2_ = 1.0*(y2-half_mid)/pixes*viewrange;
  cost_functor->pos_ = pos;
  cost_functor->vec_r_ = rs;
  cost_functor->vec_t_ = ts;

  return new ceres::NumericDiffCostFunction<OnlineCalibrateCost,
                                            ceres::CENTRAL, 2, 3>(
      cost_functor.release());//, TAKE_OWNERSHIP, 3);
}

bool OnlineCalibrateCost::operator() (const double* param, double* residuals) const {
    
    const int prev_pos = (pos_+3)%4;
    vector<double> pr = vec_r_[prev_pos];
    vector<double> pt = vec_t_[prev_pos];
    vector<double> cr = vec_r_[pos_];
    vector<double> ct = vec_t_[pos_];

    double a00 = observed_x1_*pr[6]-pr[0];
    double a01 = observed_x1_*pr[7]-pr[1];
    double a02 = observed_x1_*pr[8]-pr[2];
    double b0 = pt[0] - observed_x1_*pt[2];

    double a10 = observed_y1_*pr[6]-pr[3];
    double a11 = observed_y1_*pr[7]-pr[4];
    double a12 = observed_y1_*pr[8]-pr[5];
    double b1 = pt[1] - observed_y1_*pt[2];

    double a20 = observed_x2_*cr[6]-cr[0];
    double a21 = observed_x2_*cr[7]-cr[1];
    double a22 = observed_x2_*cr[8]-cr[3];
    double b2 = ct[0] - observed_x2_*ct[2];

    double a30 = observed_y2_*cr[6]-cr[3];
    double a31 = observed_y2_*cr[7]-cr[4];
    double a32 = observed_y2_*cr[8]-cr[5];
    double b3 = ct[1] - observed_y2_*ct[2];

    Eigen::Matrix<double, 4, 3> coeff;
    coeff << a00, a01, a02,
             a10, a11, a12,
             a20, a21, a22,
             a30, a31, a32;
    Eigen::Vector4d B;
    B<<b0, b1, b2, b3;

    Eigen::Vector3d P = coeff.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

    double degrees[] = {param[0], param[1], 0.0};
    double arrayR[9];

    EulerAnglesToRotationMatrix(degrees, 3, arrayR);

    Eigen::Matrix3d R1, R2, R;
    R1<<pr[0], pr[1], pr[2],
        pr[3], pr[4], pr[5],
        pr[6], pr[7], pr[8];
    
    R2<<cr[0], cr[1], cr[2],
        cr[3], cr[4], cr[5],
        cr[6], cr[7], cr[8];

    R<<arrayR[0], arrayR[1], arrayR[2],
       arrayR[3], arrayR[4], arrayR[5],
       arrayR[6], arrayR[7], arrayR[8];

    Eigen::Vector3d T1, T2, T;
    T1<<pt[0], pt[1], pt[2];
    T2<<ct[0], ct[1], ct[2];
    T<<0.0, 0.0, param[2];
    
    Eigen::Vector3d P1 = R1*R*P+R1*T+T1;
    Eigen::Vector3d P2 = R2*R*P+R2*T+T2;

    residuals[0] = P1[0]/P1[2]-P2[0]/P2[2];
    residuals[1] = P1[1]/P1[2]-P2[1]/P2[2];

  return true;
}