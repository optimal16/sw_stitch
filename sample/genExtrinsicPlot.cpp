#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <gflags/gflags.h>
#include <vector>
#include <assert.h>

using namespace cv;
using namespace std;

DEFINE_string(input, "../data/base_plot.png", "base plot image");
DEFINE_double(lr2f, 1.5, "left right to front distance");
DEFINE_double(cy, 5.4, "car y distance");
DEFINE_double(cx, 4.2, "car y distance");
DEFINE_int32(margin_width, 200, "margin width in mm");
DEFINE_int32(margin_height,  200, "margin height in mm");
DEFINE_string(output, "extrinsic_plot.png", "output filename");

vector<Mat> genH(int width, int height)
{
    width -=1;
    height -=1;
    vector<Point2f> front_points = {Point2f(0.0f, 0.0f), Point2f(width, 0.0f), Point2f(0.0f, height), Point2f(width, height)};
    vector<Point2f> right_points = {Point2f(height, 0.0f), Point2f(height, width),Point2f(0.0f, 0.0f), Point2f(0.0f, width)};
    vector<Point2f> back_points = {Point2f(width, height),Point2f(0.0f, height), Point2f(width, 0.0f),  Point2f(0.0f, 0.0f)};
    vector<Point2f> left_points = {Point2f(0.0f, width), Point2f(0.0f, 0.0f), Point2f(height, width), Point2f(height, 0.0f)};

    Mat f2r = findHomography(front_points, right_points);
    Mat f2b = findHomography(front_points, back_points);
    Mat f2l = findHomography(front_points, left_points);

    vector<Mat> Hs = {f2r, f2b, f2l};
    return Hs;
}


Mat genExtrinsicPlot(Mat& src, double lr2f, double cy, double cx, int margin_width, int margin_height)
{
    const double base_height = 1.2f;
    const double base_width = 4.2f;
    if(base_width>cx)
    {
        cx = base_width;
        cout<<"cx is small than base width "<<cx<<", "<<base_width;
    }

    int shift_lr = round(0.1f/base_height*src.rows)+3;
    int shift_x = round((cx-base_width)/base_width*src.cols/2);
    cout<<"shift_lr "<<shift_lr<<endl;
    int rows = src.rows*2 + round(src.rows/base_height*cy) + 2*margin_height;
    int cols = round(cx/base_width*src.cols) + 2*margin_width + 2*shift_lr;
    
    //front
    Mat plot = Mat(rows, cols, CV_8UC3, Vec3b(255, 255, 255));
    src.copyTo(plot(Rect(margin_width+shift_x, margin_height, src.cols, src.rows)));

    //right
    Mat right, back, left;
    vector<Mat> Hs = genH(src.cols, src.rows);
    warpPerspective(src, right, Hs[0],  Size(src.rows, src.cols), CV_INTER_LINEAR);
    warpPerspective(src, back, Hs[1],  Size(src.cols, src.rows), CV_INTER_LINEAR);
    warpPerspective(src, left, Hs[2],  Size(src.rows, src.cols), CV_INTER_LINEAR);

    //imwrite("right.jpg", right);
    //imwrite("back.jpg", back);
    //imwrite("left.jpg", left);
    
    Point back_st = Point(margin_width+shift_x, rows-margin_height-src.rows);
    int st_lr = src.rows + margin_height + round((lr2f-1.5f)/base_height*src.rows);
    Point right_st = Point(cols-src.rows-margin_width-shift_lr, st_lr);
    Point left_st = Point(margin_width-shift_lr, st_lr);

    right.copyTo(plot(Rect(right_st, Size(src.rows, src.cols))));
    back.copyTo(plot(Rect(back_st, src.size())));
    left.copyTo(plot(Rect(left_st, Size(src.rows, src.cols))));

    return plot;
}


int main(int argc, char** argv)
{
    google::ParseCommandLineFlags(&argc,&argv, true);

    Mat base_plot = imread(FLAGS_input);
    Mat plot = genExtrinsicPlot(base_plot, FLAGS_lr2f, FLAGS_cy, FLAGS_cx, FLAGS_margin_width, FLAGS_margin_height);
    imwrite(FLAGS_output, plot);

    return 0;
}
