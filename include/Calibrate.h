#ifndef CALIBRATE_H
#define CALIBRATE_H

#include "FindPoints.h"
#include "WorldPlane.h"
#include "IOConfig.h"

class Calibrate
{

private:
    WorldPlane wplane_;
    std::vector<std::vector<ImagePoints>> vec_image_points_;
    std::vector<FishEyeCam> fisheye_cams_;
    StitchParam stitch_param_;
    CalibrateParam calib_param_;
    std::string config_file_;
    cv::Size image_size_, output_size_;
    std::vector<cv::Mat> images_;
    bool debug_;

public:

    Calibrate(std::string config_file);
    ~Calibrate();

    void Init();
    void Run();
};


#endif
