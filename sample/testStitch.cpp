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

using namespace std;
using namespace cv;

DEFINE_string(config_dir, "config_dir", "directory contain calib.yml, stitch.yml and intrinsic.yml");
DEFINE_string(output, "", "output stitch image");


int main(int argc, char** argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    string input_dir = FLAGS_config_dir;
    string image_names[] = {"front.jpg", "right.jpg", "back.jpg", "left.jpg"};
    vector<Mat> inputs;
    for(int i=0; i<4; i++)
    {
        Mat src = imread(input_dir+image_names[i]);
        inputs.push_back(src);
    }

    StitchInit(FLAGS_config_dir);
    Mat dst = StitchProcess(inputs);
    string output_filename = FLAGS_output;
    if(output_filename.empty())
        output_filename = input_dir + "stitch.jpg";
    imwrite(output_filename, dst);

    StitchUinit();

    return 0;
}
