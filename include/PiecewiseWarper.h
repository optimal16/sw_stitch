#ifndef PIECEWISE_WAPER_H
#define PIECEWISE_WAPER_H

#include <opencv2/core.hpp>
#include <IOConfig.h>
#include <vector>

void BuildFishEyeToSphereMap(FishEyeCam& cam, cv::Mat& xmap, cv::Mat& ymap, float scale=1000.0f);

void BuildSphereToFishEyeMap(FishEyeCam& cam, cv::Mat& xmap, cv::Mat& ymap, float scale=1000.0f);

void BuildPiecewiseMap(FishEyeCam& cam, cv::Mat& xmap, cv::Mat& ymap, float focal_x, float focal_y, float x_view_degree, float y_view_degree, float rotx, float roty, float rotz);

void BuildPiecewiseMap(FishEyeCam& cam, cv::Mat& xmap, cv::Mat& ymap, float focal_x, float focal_y, float x_view_degree, float y_view_degree, cv::Mat& R);

void BuildPiecewise5QuadrantMap(FishEyeCam& cam, cv::Mat& xmap, cv::Mat& ymap, float focal, float x_view_degree, float y_view_degree, float rotx, float roty, float rotz);

#endif
