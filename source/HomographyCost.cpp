#include "HomographyCost.h"
#include "eigen3/Eigen/Dense"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

using namespace ceres;
using namespace std;
using namespace cv;

//namespace {
//template<typename T>
//T r(T k2, T k3, T k4, T k5, T theta)
//{
//    // k1 = 1
//    return theta +
//           k2 * theta * theta * theta +
//           k3 * theta * theta * theta * theta * theta +
//           k4 * theta * theta * theta * theta * theta * theta * theta +
//           k5 * theta * theta * theta * theta * theta * theta * theta * theta * theta;
//}  
//}
//
//ceres::CostFunction* HomographyCost::Create(double x,
//                                     double y, double px, double py) {   
//
//  std::unique_ptr<HomographyCost> cost_functor(
//      new HomographyCost);
//  cost_functor->observed_x = x;
//  cost_functor->observed_y = y;
//  cost_functor->point_x = px;
//  cost_functor->point_y = py;
//
//  return new ceres::NumericDiffCostFunction<HomographyCost,
//                                            ceres::CENTRAL, 4, 8>(
//      cost_functor.release());//, TAKE_OWNERSHIP, 3);
//}
//
//bool HomographyCost::operator() (const double* H, double* residuals) const {
//  //projection error
//  #if 1
//  double point[2] = {point_x, point_y};
//  double scale = H[6]*point[0] + H[7]*point[1] + 1.0;
//  if(scale == 0.0)
//  {
//    double inf = 10000000.0;
//    residuals[0] = inf;
//    residuals[1] = inf;
//  }else
//  {
//    double xx = H[0]*point[0] + H[1]*point[1] + H[2];
//    double yy = H[3]*point[0] + H[4]*point[1] + H[5];
//    xx = xx / scale;
//    yy = yy / scale;
//    residuals[0] = xx - observed_x;
//    residuals[1] = yy - observed_y;
//  }
//  #endif
//
//  //back-projection error
//  #if 1
//  cv::Mat_<double> h_inv(3, 3);
//  h_inv(0, 0) = H[0]; h_inv(0, 1) = H[1]; h_inv(0, 2) = H[2];
//  h_inv(1, 0) = H[3]; h_inv(1, 1) = H[4]; h_inv(1, 2) = H[5];
//  h_inv(2, 0) = H[6]; h_inv(2, 1) = H[7]; h_inv(2, 2) = 1.0;
//  h_inv = h_inv.inv();
//  double point2[2] = {observed_x, observed_y};
//  double scale2 = h_inv(2, 0)*point2[0] + h_inv(2, 1)*point2[1] + h_inv(2, 2);
//  if(scale2 == 0.0)
//  {
//    double inf = 10000000.0;
//    residuals[2] = inf;
//    residuals[3] = inf;
//  }else
//  {
//    double xx = h_inv(0, 0)*point2[0] + h_inv(0, 1)*point2[1] + h_inv(0, 2);
//    double yy = h_inv(1, 0)*point2[0] + h_inv(1, 1)*point2[1] + h_inv(1, 2);
//    xx = xx / scale2;
//    yy = yy / scale2;
//    residuals[2] = xx - point_x;
//    residuals[3] = yy - point_y;
//  }  
//  #endif
//
//  return true;
//}
//

ceres::CostFunction* GlobalCost::Create(
  double x, double y, double px, double py, int pos, bool flag) {   

  std::unique_ptr<GlobalCost> cost_functor(
      new GlobalCost);
  cost_functor->observed_x = x;
  cost_functor->observed_y = y;
  cost_functor->point_x = px;
  cost_functor->point_y = py; 
  cost_functor->pos = pos;
  cost_functor->flag = flag;

  return new ceres::NumericDiffCostFunction<GlobalCost,
                                            ceres::CENTRAL, 4, 32>(
      cost_functor.release());//, TAKE_OWNERSHIP, 3);
}

ceres::CostFunction* GlobalCost::Create(
  double x, double y, double x2, double y2, double px, double py, int pos, bool flag) {   

  std::unique_ptr<GlobalCost> cost_functor(
      new GlobalCost);
  cost_functor->observed_x = x;
  cost_functor->observed_y = y;
  cost_functor->observed_x2_ = x2;
  cost_functor->observed_y2_ = y2;  
  cost_functor->point_x = px;
  cost_functor->point_y = py; 
  cost_functor->pos = pos;
  cost_functor->flag = flag;

  return new ceres::NumericDiffCostFunction<GlobalCost,
                                            ceres::CENTRAL, 4, 32>(
      cost_functor.release());//, TAKE_OWNERSHIP, 3);
}


bool GlobalCost::operator() (const double* H, double* residuals) const {
  //projection error
  int start_index = pos*8;
  double beta = 0.01;
  #if 1
  double point[2] = {point_x, point_y};
  double scale = H[start_index+6]*point[0] + H[start_index+7]*point[1] + 1.0;
  if(scale == 0.0)
  {
    double inf = 10000000.0;
    residuals[0] = inf;
    residuals[1] = inf;
  }else
  {
    double xx = H[start_index+0]*point[0] + H[start_index+1]*point[1] + H[start_index+2];
    double yy = H[start_index+3]*point[0] + H[start_index+4]*point[1] + H[start_index+5];
    xx = xx / scale;
    yy = yy / scale;
    residuals[0] = xx - observed_x;
    residuals[1] = yy - observed_y;
    //residuals[2] = 0.0;
    //residuals[3] = 0.0;

    #if 1
    double nxx, nyy;
    if(flag)
    {
      int n_start_index = (start_index + 8)%32;
      scale = H[n_start_index+6]*point[0] + H[n_start_index+7]*point[1] + 1.0;
      nxx = H[n_start_index+0]*point[0] + H[n_start_index+1]*point[1] + H[n_start_index+2];
      nyy = H[n_start_index+3]*point[0] + H[n_start_index+4]*point[1] + H[n_start_index+5];
      nxx = nxx / scale;
      nyy = nyy / scale;
      residuals[0] = 0.0;//beta*(xx - nxx);
      residuals[1] = 0.0;//beta*(yy - nyy);    
    } 
    #endif   
  }
  #endif

  //back-projection error
  #if 1
  cv::Mat_<double> h_inv(3, 3);
  h_inv(0, 0) = H[start_index+0]; h_inv(0, 1) = H[start_index+1]; h_inv(0, 2) = H[start_index+2];
  h_inv(1, 0) = H[start_index+3]; h_inv(1, 1) = H[start_index+4]; h_inv(1, 2) = H[start_index+5];
  h_inv(2, 0) = H[start_index+6]; h_inv(2, 1) = H[start_index+7]; h_inv(2, 2) = 1.0;
  h_inv = h_inv.inv();
  double point2[2] = {observed_x, observed_y};
  double scale2 = h_inv(2, 0)*point2[0] + h_inv(2, 1)*point2[1] + h_inv(2, 2);
  if(scale2 == 0.0)
  {
    double inf = 10000000.0;
    residuals[2] = inf;
    residuals[3] = inf;
  }else
  {
    double xx = h_inv(0, 0)*point2[0] + h_inv(0, 1)*point2[1] + h_inv(0, 2);
    double yy = h_inv(1, 0)*point2[0] + h_inv(1, 1)*point2[1] + h_inv(1, 2);
    xx = xx / scale2;
    yy = yy / scale2;
    residuals[2] = xx - point_x;
    residuals[3] = yy - point_y;

    #if 1
    double nxx, nyy;
    point2[0] = observed_x2_; point2[1] = observed_y2_;
    if(flag)
    {
      int n_start_index = (start_index + 8)%32;
      for(int i = 0; i < 8; ++i)
      {
        int ww = i%3, hh = i/3;
        h_inv(hh, ww) = H[n_start_index+i];
      }
      h_inv(2, 2) = 1.0;
      h_inv = h_inv.inv();   
      scale2 = h_inv(2, 0)*point2[0] + h_inv(2, 1)*point2[1] + h_inv(2, 2);

      nxx = h_inv(0, 0)*point2[0] + h_inv(0, 1)*point2[1] + h_inv(0, 2);
      nyy = h_inv(1, 0)*point2[0] + h_inv(1, 1)*point2[1] + h_inv(1, 2);
      nxx = nxx / scale2;
      nyy = nyy / scale2;

      //std::cout<<"diff: "<<xx<<", "<<nxx<<", y: "<<yy<<", "<<nyy<<std::endl;
      residuals[2] = beta*(xx - nxx);
      residuals[3] = beta*(yy - nyy);    
    } 
    #endif       
  }  
  #endif

  //static int count=0;
  //cout<<"for "<<count++<<" error  "<<residuals[0]<<", "<<residuals[1]<<", "<<residuals[2]<<", "<<residuals[3]<<endl; 
  return true;

}

//
//ceres::CostFunction* CameraCost::Create(
//                cv::Point3f sp, 
//                cv::Point2f ip, int pos) {   
//
//  std::unique_ptr<CameraCost> cost_functor(
//      new CameraCost);
//  cost_functor->space_point = sp;
//  cost_functor->image_point = ip;
//  cost_functor->pos = pos;
//  
//  return new ceres::NumericDiffCostFunction<CameraCost,
//                                            ceres::CENTRAL, 2, 6, 24>(
//      cost_functor.release());//, TAKE_OWNERSHIP, 3);
//}
//
//bool CameraCost::operator() (const double* K, const double* H, double* residuals) const {
//  int start_index = pos*6;
//  double camera[3] = {H[start_index], H[start_index + 1], H[start_index + 2]};
//  double translation[3] = {H[start_index + 3], H[start_index + 4], H[start_index + 5]};
//  double p[3];
//  double temp[3] = {space_point.x, space_point.y, space_point.z};
//
//  ceres::AngleAxisRotatePoint(camera, temp, p);  
//  p[0] += translation[0]; p[1] += translation[1]; p[2] += translation[2];
//  double dpx, dpy, r2;
//  const double r = std::sqrt(p[0] * p[0] + p[1] * p[1]);
//  const double theta = std::atan2(r, p[2]);
//  const double theta2 = theta * theta;
//  if (theta2 > double(std::numeric_limits<double>::epsilon())) {
//      dpx = theta * p[0] / r;
//      dpy = theta * p[1] / r;
//  }
//  else {
//      dpx = p[0] / p[2];
//      dpy = p[1] / p[2];
//  }
//  r2 = theta2;
//  // Apply radial distortion
//  double radial_distortion_1 = K[4];
//  double radial_distortion_2 = K[5];
//  const double distortion =
//      1.0 + r2 * (radial_distortion_1 + radial_distortion_2 * r2);
//  dpx *= distortion;
//  dpy *= distortion;
//
//  double focal_length_x = K[0];
//  double focal_length_y = K[1];
//  double u0 = K[2];
//  double v0 = K[3];
//  // Map the distorted ray to the image plane and return the depth.
//  double res_px = focal_length_x * dpx + u0;
//  double res_py = focal_length_y * dpy + v0;  
//
//  residuals[0] = res_px - image_point.x;
//  residuals[1] = res_py - image_point.y;
//
//  return true;
//}
//
//
//
//ceres::CostFunction* ExtrinsicCost::Create(
//                std::vector<cv::Mat> k_matrix,
//                cv::Point3f sp, 
//                cv::Point2f ip, int pos) {   
//
//  std::unique_ptr<ExtrinsicCost> cost_functor(
//      new ExtrinsicCost);
//  cost_functor->space_point = sp;
//  cost_functor->image_point = ip;
//  cost_functor->pos = pos;
//  cost_functor->k = k_matrix;
//
//  return new ceres::NumericDiffCostFunction<ExtrinsicCost,
//                                            ceres::CENTRAL, 2, 24>(
//      cost_functor.release());//, TAKE_OWNERSHIP, 3);
//}
//
//
//bool ExtrinsicCost::operator() (const double* H, double* residuals) const {
//  int start_index = pos*6;
//  double camera[3] = {H[start_index], H[start_index + 1], H[start_index + 2]};
//  double translation[3] = {H[start_index + 3], H[start_index + 4], H[start_index + 5]};
//  double p[3] = {0.0, 0.0, 0.0};
//  double temp[3] = {space_point.x, space_point.y, space_point.z};
//
//  #if 1
//  std::vector<cv::Point3f> object_points(1);
//  object_points[0] = space_point;
//  std::vector<cv::Point2f> p_image_points(1);
//  cv::Mat rvec = (cv::Mat_<double>(1, 3)<<camera[0], camera[1], camera[2]);
//  cv::Mat tvec = (cv::Mat_<double>(1, 3)<<translation[0], translation[1], translation[2]);
//  cv::Mat kk = (cv::Mat_<double>(3, 3)<<k[pos].at<double>(0, 0), 0.0, k[pos].at<double>(0, 2),
//                                        0.0, k[pos].at<double>(0, 1), k[pos].at<double>(0, 3),
//                                        0.0, 0.0, 1.0);
//  cv::Mat dist_coeffs = (cv::Mat_<double>(1, 4)<<k[pos].at<double>(0, 4), k[pos].at<double>(0, 5),
//        k[pos].at<double>(0, 6), k[pos].at<double>(0, 7));
//  
//  cv::fisheye::projectPoints(object_points, p_image_points, rvec, 
//            tvec, kk, dist_coeffs);
//
//  residuals[0] = p_image_points[0].x - image_point.x;
//  residuals[1] = p_image_points[0].y - image_point.y;
//  return true;
//  
//  #endif
//
//  #if 0
//  cv::Mat R0;
//  cv::Mat rvec = (cv::Mat_<double>(1, 3)<<camera[0], camera[1], camera[2]);
//  cv::Rodrigues(rvec, R0);
//
//  Eigen::MatrixXd R(3,3);
//  R << R0.at<double>(0,0), R0.at<double>(0,1), R0.at<double>(0,2),
//        R0.at<double>(1,0), R0.at<double>(1,1), R0.at<double>(1,2),
//        R0.at<double>(2,0), R0.at<double>(2,1), R0.at<double>(2,2);
//  Eigen::Vector3d t;
//  t << translation[0], translation[1], translation[2];
//  Eigen::Vector3d P;
//  P << temp[0], temp[1], temp[2];
//  P = R * P + t;
//  Eigen::Vector2d res;
//  double theta = ::acos(P(2) / P.norm());
//  double phi = ::atan2(P(1), P(0));
//  double radial_distortion_1 = k[pos].at<double>(0, 4);
//  double radial_distortion_2 = k[pos].at<double>(0, 5);
//  double radial_distortion_3 = k[pos].at<double>(0, 6);
//  double radial_distortion_4 = k[pos].at<double>(0, 7);
//  Eigen::Vector2d p_u = r(radial_distortion_1, radial_distortion_2, radial_distortion_3, radial_distortion_4, 
//      theta) * Eigen::Vector2d(::cos(phi), ::sin(phi));
//
//  double focal_length_x = k[pos].at<double>(0, 0);
//  double focal_length_y = k[pos].at<double>(0, 1);
//  double u0 = k[pos].at<double>(0, 2);
//  double v0 = k[pos].at<double>(0, 3);
//
//  // Apply generalised projection matrix
//  res << focal_length_x * p_u(0) + u0, focal_length_y * p_u(1) + v0;
//  residuals[0] = res(0) - image_point.x;
//  residuals[1] = res(1) - image_point.y;
//  return true;
//  #endif
//
//  #if 0
//  ceres::AngleAxisRotatePoint(camera, temp, p);  
//  p[0] += translation[0]; p[1] += translation[1]; p[2] += translation[2];
//  double dpx, dpy, r2;
//  const double r = std::sqrt(p[0] * p[0] + p[1] * p[1]);
//  const double theta = std::atan2(r, p[2]);
//  const double theta2 = theta * theta;
//  if (theta2 > double(std::numeric_limits<double>::epsilon())) {
//      dpx = theta * p[0] / r;
//      dpy = theta * p[1] / r;
//  }
//  else {
//      dpx = p[0] / p[2];
//      dpy = p[1] / p[2];
//  }
//  r2 = theta2;
//  // Apply radial distortion
//  double radial_distortion_1 = k[pos].at<double>(0, 4);
//  double radial_distortion_2 = k[pos].at<double>(0, 5);
//  double radial_distortion_3 = k[pos].at<double>(0, 6);
//  double radial_distortion_4 = k[pos].at<double>(0, 7);
//
//  const double distortion =
//      1.0 + r2 * (radial_distortion_1 + radial_distortion_2 * r2
//                 +radial_distortion_3*r2*r2 + radial_distortion_4*r2*r2*r2);
//  dpx *= distortion;
//  dpy *= distortion;
//
//  double focal_length_x = k[pos].at<double>(0, 0);
//  double focal_length_y = k[pos].at<double>(0, 1);
//  double u0 = k[pos].at<double>(0, 2);
//  double v0 = k[pos].at<double>(0, 3);
//  // Map the distorted ray to the image plane and return the depth.
//  double res_px = focal_length_x * dpx + u0;
//  double res_py = focal_length_y * dpy + v0;  
//
//  residuals[0] = res_px - image_point.x;
//  residuals[1] = res_py - image_point.y;
//
//  return true;
//  #endif
//}
