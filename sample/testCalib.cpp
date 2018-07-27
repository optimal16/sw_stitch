#include "IOConfig.h"
#include <vector>
#include <iostream>
#include "Composition.h"
#include "Calibrate.h"
#include "WorldPlane.h"
#include "FindPoints.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <gflags/gflags.h>

using namespace std;
using namespace cv;

DEFINE_string(calib_file, "calib.yml", "calib config file");

int main(int argc, char** argv)
{
    google::ParseCommandLineFlags(&argc,&argv, true);
    Calibrate cb(FLAGS_calib_file);
    cb.Init();
    cb.Run();

    return 0;
}
