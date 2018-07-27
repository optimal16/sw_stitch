#ifndef COMPOSITION_H
#define COMPOSITION_H

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "IOConfig.h"

//#define PIX_WORLD_DIS 0.02
//#define BOUNDING_PIX 18
//#define INNER_PADDING 10
//#define IMAGE_WIDTH 1280
//#define IMAGE_HEIGHT 720


void GenerateHWarpMapAndMask(cv::Mat& map, cv::Mat& mask, FishEyeCam& cam, cv::Mat& H, cv::Size ouput_size);

void GenerateHWarpMapAndMask(cv::Mat& map, cv::Mat& mask, FishEyeCam& cam, cv::Mat& r, cv::Mat& t, float viewrange, float pix_dis);

void CalcInnerRect(std::vector<cv::Mat>& vec_masks, cv::Rect& inner_rect);

cv::Size GenSeparateMask(std::vector<cv::Mat>& vec_masks, std::vector<cv::Point>& vec_st_points, cv::Size output_size, cv::Rect inner_rect, int extend_off=4, int cut_off=18, int lean_pix=0);

void SeparateMap(std::vector<cv::Mat>& vec_sep_maps, std::vector<cv::Mat>& vec_maps, std::vector<cv::Mat>& vec_masks, std::vector<cv::Point>& vec_st_points);

void RoundMap(std::vector<cv::Mat>& vec_round_maps, std::vector<cv::Mat>& vec_maps);

void RemapWidthIndexMap(cv::Mat& dst, std::vector<cv::Mat>& inputs, std::vector<cv::Mat>& vec_maps, std::vector<cv::Mat>& vec_masks, std::vector<cv::Point>& vec_st_points, cv::Size output_size);

#endif
