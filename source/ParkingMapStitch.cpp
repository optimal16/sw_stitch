#include "ParkingMapStitch.h"
#include "IOConfig.h"
#include "Composition.h"
#include <iostream>

using namespace std;
using namespace cv;

struct StitchMap
{
    vector<Mat> vec_maps_;
    vector<Mat> vec_round_maps_;
    vector<Mat> vec_masks_;
    vector<Point> vec_st_points_;
    Size output_size_;
    Mat dst_;
    StitchMap(){
        vec_round_maps_.resize(4);
        vec_maps_.resize(4);
        vec_st_points_.resize(4);
        vec_masks_.resize(4);
    }
};

static StitchMap g_stitch;

int StitchInit(std::string config_dir)
{
    // read intrinsic and extrinsic yaml param
    string intrinsic_yml = config_dir + "intrinsic.yml";
    string stitch_yml = config_dir + "stitch.yml";
    vector<FishEyeCam> cams;
    if(ReadFishEyeCamera(intrinsic_yml, cams)!=0)
        return -1;

    for(auto c:cams)
        c.Print();

    StitchParam sparam;
    if(ReadStitchParam(stitch_yml, sparam)!=0)
        return -2;
    sparam.Print();

    int pixes = round(sparam.viewrange_/sparam.pix_dis_);
    Size output_size = Size(pixes, pixes);
//    cout<<"init size "<<output_size<<endl;
    
    // init map and separate map
    vector<Mat> vec_init_maps(4), vec_init_masks(4);
    for(int i=0; i<4; i++)
    {
        GenerateHWarpMapAndMask(vec_init_maps[i], vec_init_masks[i], cams[i], sparam.Hs_[i], output_size);
        //imwrite("mask_" + to_string(i) + ".jpg", vec_init_masks[i]);
    }

    Rect inner_rect;
    CalcInnerRect(vec_init_masks, inner_rect);
    if(sparam.inner_rect_!=inner_rect && sparam.inner_rect_.width*sparam.inner_rect_.height!=0)
    {
        cout<<"use custom inner rect "<<sparam.inner_rect_<<endl;
        inner_rect = sparam.inner_rect_;
    }


    g_stitch.output_size_ = GenSeparateMask(g_stitch.vec_masks_, g_stitch.vec_st_points_, output_size, inner_rect, sparam.extend_off_, sparam.cut_off_, sparam.lean_pix_);

    cout<<"final output size "<<g_stitch.output_size_<<endl;
    for(int i=0; i<4; i++)
    {
        //imwrite("separate_mask_" + to_string(i) + ".jpg", g_stitch.vec_masks_[i]);
    }
    
    SeparateMap(g_stitch.vec_maps_, vec_init_maps, g_stitch.vec_masks_, g_stitch.vec_st_points_);

    // round map and malloc dst
    RoundMap(g_stitch.vec_round_maps_, g_stitch.vec_maps_);
    g_stitch.dst_ = Mat::zeros(g_stitch.output_size_, CV_8UC3);

    cout<<"stitch init success!"<<endl;
    return 0;
}

cv::Mat StitchProcess(std::vector<cv::Mat>& inputs)
{
    RemapWidthIndexMap(g_stitch.dst_, inputs, g_stitch.vec_round_maps_, g_stitch.vec_masks_, g_stitch.vec_st_points_, g_stitch.output_size_);
    return g_stitch.dst_;
}


int StitchUinit()
{
    for(int i=0; i<4; i++)
    {
        g_stitch.vec_maps_[i].release();
        g_stitch.vec_round_maps_[i].release();
        g_stitch.vec_masks_[i].release();
    }
    return 0;
}

