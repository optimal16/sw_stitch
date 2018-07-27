#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <gflags/gflags.h>
#include <vector>

using namespace cv;
using namespace std;

DEFINE_int32(width, 9, "chessboard width");
DEFINE_int32(height, 6, "chessboard width");
DEFINE_int32(margin_width, 100, "margin width in mm");
DEFINE_int32(margin_height,  100, "margin height in mm");
DEFINE_int32(size,  60, "margin height in mm");
DEFINE_string(output, "chessboard.png", "output filename");

Mat genChessboard(int width, int height, int margin_width, int margin_height, int size)
{
    int cols = 2*margin_width+(width+1)*size;
    int rows = 2*margin_height+(height+1)*size;

    Mat chessboard = Mat::ones(rows, cols, CV_8U)*255;
    for(int j=0; j<height+1; j++)
    {
        for(int i=0; i<width+1; i++)
        {
            int flag = (i+j)%2;
            if(flag==1)
            {
                for(int r=margin_height+j*size; r<margin_height+(j+1)*size; r++)
                {
                    for(int c=margin_width+i*size; c<margin_width+(i+1)*size; c++)
                    {
                        chessboard.at<uchar>(r, c)=0;
                    }
                }
            }
        }
    }

    return chessboard;
}


int main(int argc, char** argv)
{
    google::ParseCommandLineFlags(&argc,&argv, true);

    Mat chessboard = genChessboard(FLAGS_width, FLAGS_height, FLAGS_margin_width, FLAGS_margin_height, FLAGS_size);

    imwrite(FLAGS_output, chessboard);

    return 0;
}
