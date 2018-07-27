#include "PiecewiseWarper.h"
#include "IOConfig.h"
#include <opencv2/ccalib/omnidir.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <gflags/gflags.h>
#include "filedir.h"
#include <unistd.h>

using namespace std;
using namespace cv;

DEFINE_string(input, "../data/front0.jpg", "input image filename");
DEFINE_string(config, "../config/intrinsic.yml", "intrinsic yml");
DEFINE_int32(model, 0, "for model 0: kannala-brandt; for model 1: mei");
DEFINE_string(output, "", "output dir");

Mat cvOmniWarper(Mat& src, OmniCam& cam)
{
    Mat newK =cam.K_.clone();
    newK.at<double>(0, 0) = newK.at<double>(0, 0)/6;
    newK.at<double>(1, 1) = newK.at<double>(1, 1)/6;
    newK.at<double>(0, 2) = src.cols/2;
    newK.at<double>(1, 2) = src.rows/2;

    Mat dst;
    omnidir::undistortImage(src, dst, cam.K_, cam.D_, cam.xi_, cv::omnidir::RECTIFY_PERSPECTIVE, newK, src.size());
    return dst;
}

Mat FishEyeSphereTest(Mat& src, FishEyeCam& cam)
{
    Mat xmap, ymap;
    BuildFishEyeToSphereMap(cam, xmap, ymap);
    Mat dst;
    remap(src, dst, xmap, ymap, CV_INTER_LINEAR);
    return dst;
}


Mat cvPlaneWaper(Mat& src, FishEyeCam& cam)
{
//    Mat newK, dst;
//    float fov_scale=2.0f;
//    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cam.K, cam.D, src.size(), cv::Matx33d::eye(), newK, 1, src.size()*2, fov_scale);
    Mat newK =cam.K_.clone();
    newK.at<double>(0, 0) = newK.at<double>(0, 0)/2;
    newK.at<double>(1, 1) = newK.at<double>(1, 1)/2;
    newK.at<double>(0, 2) = src.cols;
    newK.at<double>(1, 2) = src.rows;

    Mat dst;
    cv::fisheye::undistortImage(src, dst, cam.K_, cam.D_, newK, src.size()*2);
    return dst;
}


int main(int argc, char** argv)
{

    google::ParseCommandLineFlags(&argc,&argv, true);
    
    int model = FLAGS_model;
    string input = FLAGS_input;
    string config = FLAGS_config;
    string output = FLAGS_output;

    string tag="_mei.jpg";
    if(model==0)
        tag = "kannala.jpg";

    if(output.empty())
    {
        output = input.substr(input.size()-4)+tag;
    }

    Mat src = imread(input);
    Mat dst;
    if(model==0)
    {
        FishEyeCam cam;
        ReadFishEyeCamera(config, cam);
        cam.Print();
        dst = cvPlaneWaper(src, cam);
    }else
    {
        OmniCam cam;
        ReadOmniCamera(config, cam);
        cam.Print();
        dst = cvOmniWarper(src, cam);
    }

    imwrite(output, dst);

    return 0;

}
