#include "IOConfig.h"
#include <iostream>

using namespace std;
using namespace cv;

int ReadOmniCamera(std::string filename, OmniCam& cam)
{   
    cv::FileStorage ifile(filename, cv::FileStorage::READ);
    if(!ifile.isOpened())
    {
        cout<<"read file "<<filename<<" failed!"<<endl;
        return -1;
    }
    ifile["image_width"]>>cam.size_.width;
    ifile["image_height"]>>cam.size_.height;

    double xi, k1, k2, p1, p2, gamma1, gamma2, u0, v0;
    cv::FileNode mirror = ifile["mirror_parameters"];
    mirror["xi"]>>xi;
    cv::FileNode distort = ifile["distortion_parameters"];
    distort["k1"]>>k1;
    distort["k2"]>>k2;
    distort["p1"]>>p1;
    distort["p2"]>>p2;

    cv::FileNode project = ifile["projection_parameters"];
    project["gamma1"]>>gamma1;
    project["gamma2"]>>gamma2;
    project["u0"]>>u0;
    project["v0"]>>v0;

    cam.K_ = (Mat_<double>(3,3)<<gamma1, 0.0, u0, 0.0, gamma2, v0, 0.0, 0.0, 1.0);
    cam.D_ = (Mat_<double>(4,1)<<k1, k2, p1, p2);
    cam.xi_ = (Mat_<double>(1,1)<<xi);

    ifile.release();
    cout<<"read file "<<filename<<" success!"<<endl;
    return 0;
}


void OmniCam::Print()
{
    cout<<"============================OmniCam print============================"<<endl;
    cout<<"image size: "<<size_<<endl;
    cout<<"K: "<<K_<<endl;
    cout<<"D: "<<D_<<endl;
    cout<<"xi: "<<xi_<<endl;
}


void FishEyeCam::Print()
{
    cout<<"============================FishEyeCam print============================"<<endl;
    cout<<"image size: "<<size_<<endl;
    cout<<"K: "<<K_<<endl;
    cout<<"D: "<<D_<<endl;
}

void ReadProjectNode(FileNode& project, Mat& K, Mat& D)
{

    double k2, k3, k4, k5, mu, mv, u0, v0;
    project["k2"]>>k2;
    project["k3"]>>k3;
    project["k4"]>>k4;
    project["k5"]>>k5;

    project["mu"]>>mu;
    project["mv"]>>mv;
    project["u0"]>>u0;
    project["v0"]>>v0;

    K = (Mat_<double>(3,3)<<mu, 0.0, u0, 0.0, mv, v0, 0.0, 0.0, 1.0);
    D = (Mat_<double>(4,1)<<k2, k3, k4, k5);
}


int ReadFishEyeCamera(std::string filename, FishEyeCam& cam)
{
    cv::FileStorage ifile(filename, cv::FileStorage::READ);
    if(!ifile.isOpened())
    {
        cout<<"read file "<<filename<<" failed!"<<endl;
        return -1;
    }
    ifile["image_width"]>>cam.size_.width;
    ifile["image_height"]>>cam.size_.height;

    cv::FileNode project = ifile["projection_parameters"];
    ReadProjectNode(project, cam.K_, cam.D_);
    cout<<"read file "<<filename<<" success!"<<endl;
    return 0;
}


int ReadFishEyeCamera(std::string filename, vector<FishEyeCam>& cams)
{
    cv::FileStorage ifile(filename, cv::FileStorage::READ);
    if(!ifile.isOpened())
    {
        cout<<"read file "<<filename<<" failed!"<<endl;
        return -1;
    }


    cams.resize(4);
    Size size;
    ifile["image_width"]>>size.width;
    ifile["image_height"]>>size.height;

    string project_names[4] = {"projection_parameters_front", "projection_parameters_right","projection_parameters_back", "projection_parameters_left"};
    for(int i=0; i<4; i++)
    {
        cams[i].size_ = size;
        cv::FileNode project = ifile[project_names[i]];
        ReadProjectNode(project, cams[i].K_, cams[i].D_);
    }

    ifile.release();
    cout<<"read file "<<filename<<" success!"<<endl;
    return 0;
}

int ReadCalibParam(std::string filename, CalibrateParam& param)
{
    cv::FileStorage ifile(filename, cv::FileStorage::READ);
    if(!ifile.isOpened())
    {
        cout<<"read file "<<filename<<" failed!"<<endl;
        return -1;
    }
    ifile["intrinsic_file"]>>param.intrinsic_file_;
    ifile["image_points_dir"]>>param.image_points_dir_;
    ifile["stitch_file"]>>param.stitch_file_;
    ifile["left_right_to_front_distance"]>>param.left_right_to_front_distance_;
    ifile["car_y"]>>param.car_y_;
    ifile["car_x"]>>param.car_x_;
    ifile["viewrange"]>>param.viewrange_;
    ifile["pix_dis"]>>param.pix_dis_;

    cout<<"read file "<<filename<<" success!"<<endl;
    ifile.release();
    return 0;
}

int WriteCalibParam(std::string filename, CalibrateParam& param)
{
    cv::FileStorage ofile(filename, cv::FileStorage::WRITE);
    if(!ofile.isOpened())
    {
        cout<<"read file "<<filename<<" failed!"<<endl;
        return -1;
    }
    ofile<<"intrinsic_file"<<param.intrinsic_file_;
    ofile<<"image_points_dir"<<param.image_points_dir_;
    ofile<<"stitch_file"<<param.stitch_file_;
    ofile<<"left_right_to_front_distance"<<param.left_right_to_front_distance_;
    ofile<<"car_y"<<param.car_y_;
    ofile<<"car_x"<<param.car_x_;
    ofile<<"viewrange"<<param.viewrange_;
    ofile<<"pix_dis"<<param.pix_dis_;

    cout<<"write file "<<filename<<" success!"<<endl;
    ofile.release();
    return 0;
}

void CalibParam::TestOut(string filename)
{
    CalibParam cp;
    cp.intrinsic_file_ = "../data/test/intrinsic.yml";
    cp.image_points_dir_ = "../data/test/";
    cp.left_right_to_front_distance_ = 1.488f;
    cp.car_y_ = 5.386f;
    cp.car_x_ = 4.0f;
    cp.viewrange_ = 12.0f;
    cp.pix_dis_ = 0.02;
    cp.stitch_file_ = "stitch.yml";
    WriteCalibParam(filename, cp);
}

StitchParam::StitchParam()
{
    lean_pix_ = 0;
    pix_dis_ = 0.02f;
    cut_off_ = 18;
    extend_off_ = 4;
    inner_rect_ = Rect(0, 0, 0, 0);
    Hs_.resize(4);
    Rs_.resize(4);
    Ts_.resize(4);
}

void StitchParam::Set(CalibrateParam& cparam)
{
    left_right_to_front_distance_ = cparam.left_right_to_front_distance_;
    car_y_ = cparam.car_y_;
    car_x_ = cparam.car_x_;
    viewrange_ = cparam.viewrange_;
    pix_dis_ = cparam.pix_dis_;
}


void StitchParam::Print()
{
    cout<<"\n\n================================================================="<<endl;
    cout<<"car_y "<<car_y_<<endl;
    cout<<"car_x "<<car_x_<<endl;
    cout<<"left_right_to_front_distance "<<left_right_to_front_distance_<<endl;
    cout<<"viewrange "<<viewrange_<<endl;
    cout<<"pix_dis "<<pix_dis_<<endl;
    cout<<"lean_pix "<<lean_pix_<<endl;
    cout<<"extend_off "<<extend_off_<<endl;
    cout<<"cut_off "<<cut_off_<<endl;
    cout<<"inner_rect"<<inner_rect_<<endl;

    int num = Hs_.size();
    for(int i=0; i<num; i++)
    {
        cout<<"-----------------------------------------------------------------"<<endl;
        cout<<"for index "<<i<<endl; 
        cout<<"H "<<Hs_[i]<<endl;
        cout<<"R "<<Rs_[i]<<endl;
        cout<<"T "<<Ts_[i]<<endl;
    }
    cout<<endl;
}

int ReadStitchParam(std::string filename, StitchParam& param)
{
    cv::FileStorage fExtrinsic(filename, cv::FileStorage::READ);
    if(!fExtrinsic.isOpened()) {
        cout<< "failed to read stitch file: " <<filename<<endl;
        return -1;
    }

    fExtrinsic["car_y"]>>param.car_y_;
    fExtrinsic["car_x"]>>param.car_x_;
    fExtrinsic["left_right_to_front_distance"]>>param.left_right_to_front_distance_;
    fExtrinsic["viewrange"]>>param.viewrange_;
    fExtrinsic["pix_dis"]>>param.pix_dis_;
    fExtrinsic["lean_pix"]>>param.lean_pix_;
    fExtrinsic["extend_off"]>>param.extend_off_;
    fExtrinsic["cut_off"]>>param.cut_off_;
    fExtrinsic["inner_rect"]>>param.inner_rect_;

    param.Hs_.resize(4);
    param.Rs_.resize(4);
    param.Ts_.resize(4);
    for(int i = 0; i < 4; ++i)
    {
        fExtrinsic["homography_"+std::to_string(i)] >> param.Hs_[i];
        fExtrinsic["rvec_" + std::to_string(i)] >> param.Rs_[i];
        fExtrinsic["tvec_" + std::to_string(i)] >> param.Ts_[i];
    }
    fExtrinsic.release();

    cout<<"successfully read stitch file! "<<filename<<endl;
    return 0;
}

int WriteStitchParam(std::string filename, StitchParam& param)
{
    cv::FileStorage ofile(filename, cv::FileStorage::WRITE);
    if(!ofile.isOpened())
    {
        cout<<"read file "<<filename<<" failed!"<<endl;
        return -1;
    }
    ofile<<"car_y"<<param.car_y_;
    ofile<<"car_x"<<param.car_x_;
    ofile<<"left_right_to_front_distance"<<param.left_right_to_front_distance_;
    ofile<<"viewrange"<<param.viewrange_;
    ofile<<"pix_dis"<<param.pix_dis_;
    ofile<<"lean_pix"<<param.lean_pix_;
    ofile<<"extend_off"<<param.extend_off_;
    ofile<<"cut_off"<<param.cut_off_;
    ofile<<"inner_rect"<<param.inner_rect_;

    for(int i=0; i<4; i++)
    {
        ofile<<"homography_"+to_string(i) << param.Hs_[i];
        ofile<<"rvec_"+to_string(i) << param.Rs_[i];
        ofile<<"tvec_"+to_string(i) << param.Ts_[i];
    }

    cout<<"write file "<<filename<<" success!"<<endl;
    ofile.release();
    return 0;
}
