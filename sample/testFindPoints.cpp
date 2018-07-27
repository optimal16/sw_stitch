#include "IOConfig.h"
#include <vector>
#include <iostream>
#include "Calibrate.h"
#include "WorldPlane.h"
#include "FindPoints.h"
#include "filedir.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <gflags/gflags.h>

using namespace std;
using namespace cv;

DEFINE_string(calib_file, "calib.yml", "input calib yml");
DEFINE_int32(camera_id, -1, "-1 for all camera, 0:front, 1:right, 2:back, 3:left");
DEFINE_int32(debug, 1, "1 for debug");

int main(int argc, char** argv)
{
    google::ParseCommandLineFlags(&argc,&argv, true);

    if(FLAGS_camera_id<-1 || FLAGS_camera_id>3)
    {
        cout<<"input camera id error! "<<FLAGS_camera_id<<endl;
        return -1;
    }

    CalibrateParam calib_config;
    if(ReadCalibParam(FLAGS_calib_file, calib_config)!=0)
    {
        cout<<"load calib config file failed!"<<endl;
        return -1;
    }

    vector<FishEyeCam> cams;
    if(ReadFishEyeCamera(calib_config.intrinsic_file_, cams)!=0)
    {
        cout<<"load intrinsic file failed!"<<endl;
    }

    string filetags[] ={"front", "right", "back", "left"};
    vector<Mat> vec_images;
    for(int i=0; i<4; i++)
    {
        string filename = calib_config.image_points_dir_ + filetags[i] + ".jpg";
        Mat src = imread(filename);
        if(src.empty())
        {
            cout<<"load image failed! "<<filename<<endl;
            return -1;
        }
        vec_images.push_back(src);
    }
    
    vector<int> find_index={0, 1, 2, 3};
    if(FLAGS_camera_id!=-1)
        find_index = {FLAGS_camera_id};

    string debug_dir = calib_config.image_points_dir_ + "findpoints/";
    if(FLAGS_debug)
        MkDirs(debug_dir);

    int find_num = find_index.size();
    for(int i=0; i<find_num; i++)
    {
        int index = find_index[i];
        Mat gray;
        cvtColor(vec_images[index], gray, CV_BGR2GRAY);
        CFindPoints cfp(gray, cams[index], CameraPos(index), calib_config.left_right_to_front_distance_, calib_config.car_y_, calib_config.car_x_);
        vector<Point2f> points;
        if(cfp.FindPoints(points)!=0)
            return -1;
        string filename = calib_config.image_points_dir_+filetags[index]+".txt";
        CFindPoints::WriteImagePoints(filename, points);
        cout<<"output points file: "<<filename<<endl;
        if(FLAGS_debug)
        {
            Mat plot = CFindPoints::PlotCorners(vec_images[index], points);
            imwrite(debug_dir + filetags[i] +"_points.jpg", plot);
        }
    }

    return 0;
}
