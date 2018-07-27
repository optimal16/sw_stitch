#include "IOConfig.h"
#include <vector>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
    string intrinsic_yml = "../config/intrinsic.yml";
    string extrinsic_yml = "../config/extrinsic.yml";

    vector<FishEyeCam> cams;
    ReadFishEyeCamera(intrinsic_yml, cams);
    for(auto c:cams)
        c.Print();

    StitchParam param;
    ReadStitchParam(extrinsic_yml, param);
    param.Print();


    //CalibrateParam cp;
    //cp.intrinsic_file = "../data/test/intrinsic.yml";
    //cp.image_points_dir = "../data/test/";
    //cp.left_right_to_front_distance = 1.488f;
    //cp.car_y = 5.386f;
    //cp.viewrange = 12.0f;
    //cp.pix_dis = 0.02;
    //cp.extrinsic_file = "extrinsic.yml";
    //WriteCalibrateParam("calib.yml", cp);
   
    CalibrateParam::TestOut("calib_seed_0.yml");
    return 0;
}
