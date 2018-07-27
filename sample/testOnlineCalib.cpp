#include "IOConfig.h"
#include <vector>
#include <iostream>
#include "Composition.h"
#include "Calibrate.h"
#include "WorldPlane.h"
#include "FindPoints.h"
#include "ParkingMapStitch.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <gflags/gflags.h>
#include "OnlineCalibrate.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    string config_dir = "../config/sim/";

    string intrinsic_file = config_dir + "intrinsic.yml";
    string stitch_file = config_dir + "stitch.yml";


    string image_names[] = {"front.jpg", "right.jpg", "back.jpg", "left.jpg"};
    string point_files[] = {"3_0.txt", "0-1.txt", "1-2.txt", "2-3.txt"};

    vector<FishEyeCam> vec_cam;
    StitchParam param;
    ReadFishEyeCamera(intrinsic_file, vec_cam);
    ReadStitchParam(stitch_file, param);

    OnlineCalibrate oc(param, vec_cam);

    vector<Mat> vec_src(4);
    for(int i=0; i<4; i++)
    {
        vec_src[i] = imread(config_dir + image_names[i]);
    }
    vector<OverlapPoints> points;
    oc.ExtractOverlapPoints(points, vec_src, param);


    points.clear();
    vector<vector<OverlapPoints>> vec_points(4);
    for(int i=0; i<4; i++)
    {
        oc.ReadOverlapPoints(config_dir+point_files[i], vec_points[i], i);
        oc.AddStPoints(vec_points[i], i);
        for(auto op:vec_points[i])
        {
            cout<<"add st points "<<op.p1<<", "<<op.p2<<endl;
        }
        points.insert(points.end(), vec_points[i].begin(), vec_points[i].end());
    }

    double roll, pitch, z;
    roll = pitch = z = 0.0;
    oc.RefinePlane(roll, pitch, z, points);
    cout<<"after refine roll: "<<roll<<", pitch: "<<pitch<<", and z: "<<z<<endl;

    return 0;
}
