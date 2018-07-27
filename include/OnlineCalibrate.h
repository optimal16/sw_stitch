#ifndef ONLINE_CALIBRATE_H
#define ONLINE_CALIBRATE_H

#include "IOConfig.h"
#include <opencv2/core/core.hpp>
#include <vector>
#include <string>

struct OverlapPoints
{
    cv::Point2f p1;
    cv::Point2f p2;
    int pos;
};

class OnlineCalibrate
{
public:
    OnlineCalibrate(StitchParam& param, std::vector<FishEyeCam>& vec_cam);

    // 根据重合区的点refine地平面
    void RefinePlane(double& roll, double& pitch, double& z, std::vector<OverlapPoints>& points);

    void ExtractOverlapPoints(std::vector<OverlapPoints>& points, std::vector<cv::Mat>& vec_src, StitchParam& param);

    int ReadOverlapPoints(std::string filename, std::vector<OverlapPoints>& points, int pos);

    void AddStPoints(std::vector<OverlapPoints>& points, cv::Point st);
    void AddStPoints(std::vector<OverlapPoints>& points, int pos);

    ~OnlineCalibrate();
private:
    
    std::vector<std::vector<double>> vec_r_;
    std::vector<std::vector<double>> vec_t_;

    float viewrange_;
    float pix_dis_;

    std::vector<cv::Mat> vec_map_, vec_left_map_, vec_right_map_;
    std::vector<cv::Mat> vec_mask_;
    std::vector<FishEyeCam> vec_cam_;
    std::vector<cv::Rect> vec_rect_;
};

#endif
