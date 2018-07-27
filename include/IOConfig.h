#ifndef IO_CONFIG_H
#define IO_CONFIG_H

#include<string>
#include<opencv2/core.hpp>
#include<vector>

typedef struct OmniCamera
{
    cv::Mat K_, D_, xi_;
    cv::Size size_;

    void Print();

}OmniCam;

int ReadOmniCamera(std::string filename, OmniCam& cam);

/*--------------------------------------------------------------------------*/


typedef struct FishEyeCamera
{
    cv::Mat K_, D_;
    cv::Size size_;

    void Print();
}FishEyeCam;

int ReadFishEyeCamera(std::string filename, FishEyeCam& cam);
int ReadFishEyeCamera(std::string filename, std::vector<FishEyeCam>& cams);


/*----------------------------------------------------------------------------*/

typedef struct CalibrateParam
{
    std::string intrinsic_file_;
    std::string image_points_dir_;
    float left_right_to_front_distance_, car_x_, car_y_, viewrange_, pix_dis_;
    std::string stitch_file_;

    static void TestOut(std::string filename="calib.yml");
}CalibParam;

int ReadCalibParam(std::string filename, CalibrateParam& param);
int WriteCalibParam(std::string filename, CalibrateParam& param);

/*----------------------------------------------------------------------------*/

struct StitchParam
{
    StitchParam();
    void Set(CalibrateParam& cparam);
    float car_x_, car_y_, left_right_to_front_distance_;
    float viewrange_, pix_dis_;
    int lean_pix_;
    int extend_off_, cut_off_;
    cv::Rect inner_rect_;
    std::vector<cv::Mat> Hs_, Rs_, Ts_;
    void Print();
};

int ReadStitchParam(std::string filename, StitchParam& param);
int WriteStitchParam(std::string filename, StitchParam& param);


/*----------------------------------------------------------------------------*/



#endif

