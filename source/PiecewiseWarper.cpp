#include "PiecewiseWarper.h"
#include <opencv2/calib3d.hpp>
#include <iostream>

using namespace cv;
using namespace std;

#define ToRad(degree) ((degree)/180.0f)*CV_PI

std::vector<cv::Point2f> ProjectFishEyeToSphere(std::vector<cv::Point2f> points, FishEyeCam& cam, float scale=1.0f)
{
    int nums = points.size();
    vector<Point2f> undistort(nums);
    Mat intensity = Mat::eye(3,3, CV_32F);
    fisheye::undistortPoints(points, undistort, cam.K_, cam.D_, intensity, intensity);
    for(int i=0; i<nums; i++)
    {
        float x_ = undistort[i].x;
        float y_ = undistort[i].y;
        float z_ = 1.0f;

        float u = scale * (atan2f(x_, z_) + CV_PI);
        float w = y_ / sqrtf(x_ * x_ + y_ * y_ + z_ * z_);
        float v = scale * (static_cast<float>(CV_PI) - acosf(w == w ? w : 0));
        undistort[i] = Point2f(u, v);
    }

    return undistort;
}

cv::Point2f ProjectFishEyeToSphere(cv::Point2f p, FishEyeCam& cam, float scale)
{
    vector<Point2f> points={p}, undistort;
    undistort = ProjectFishEyeToSphere(points, cam, scale);
    return undistort[0];
}

void BuildFishEyeToSphereMap(FishEyeCam& cam, cv::Mat& xmap, cv::Mat& ymap, float scale)
{
    int rows = (int)(scale*CV_PI);
    int cols = 2*rows;

    xmap = Mat::zeros(rows, cols, CV_32F);
    ymap = Mat::zeros(rows, cols, CV_32F);
    for(int r=0; r<rows; r++)
    {
        for(int c=0; c<cols; c++)
        {
            float u = c/scale-CV_PI;
            float v = r/scale;

            float sinv = sinf(static_cast<float>(CV_PI) - v);
            float x_ = sinv * sinf(u);
            float y_ = cosf(static_cast<float>(CV_PI) - v);
            float z_ = sinv * cosf(u);

            // TODO！！！
            if(z_>0)
            {
                x_=x_/z_;
                y_=y_/z_;
            }
            else
                continue;
            vector<Point2f> points={Point2f(x_, y_)};
            vector<Point2f> distort;
            fisheye::distortPoints(points, distort, cam.K_, cam.D_);
            xmap.at<float>(r,c) = distort[0].x;
            ymap.at<float>(r,c) = distort[0].y;
        }
    }

}

void BuildSphereToFishEyeMap(FishEyeCam& cam, cv::Mat& xmap, cv::Mat& ymap, float scale)
{
    xmap = Mat::zeros(cam.size_, CV_32F);
    ymap = Mat::zeros(cam.size_, CV_32F);

    for(int r=0; r<cam.size_.height; r++)
    {
        for(int c=0; c<cam.size_.width; c++)
        {
            Point2f res = ProjectFishEyeToSphere(Point2f(c, r), cam, scale);
            xmap.at<float>(r,c) = res.x;
            ymap.at<float>(r,c) = res.y;

        }
    }
}

Mat genRotMatrix(float rotx=0.0f, float roty=0.0f, float rotz=0.0f)
{
    float radx = ToRad(rotx);
    float rady = ToRad(roty);
    float radz = ToRad(rotz);
    Mat rx = (Mat_<double>(3,3)<<1.0, 0.0, 0.0, 0.0, cos(radx), -sin(radx), 0, sin(radx), cos(radx));
    Mat ry = (Mat_<double>(3,3)<<cos(rady), 0.0, sin(rady), 0.0, 1.0, 0.0, -sin(rady), 0.0, cos(rady));
    Mat rz = (Mat_<double>(3,3)<<cos(radz), -sin(radz), 0.0, sin(radz), cos(radz), 0.0, 0.0, 0.0, 1.0);
    Mat rot = rx*ry*rz;
    return rot;
}

Point2f rotPoints(Mat& R, Point2f p)
{
    float x = R.at<double>(0,0)*p.x+R.at<double>(0, 1)*p.y+R.at<double>(0, 2);
    float y = R.at<double>(1,0)*p.y+R.at<double>(1, 1)*p.y+R.at<double>(1, 2);
    float z = R.at<double>(2,0)*p.y+R.at<double>(2, 1)*p.y+R.at<double>(2, 2);
    Point2f res = Point2f(x/z, y/z);
    return res;
}

void BuildPiecewiseMap(FishEyeCam& cam, cv::Mat& xmap, cv::Mat& ymap, float focal_x, float focal_y, float x_view_degree, float y_view_degree, float rotx, float roty, float rotz)
{
    Mat R = genRotMatrix(rotx, roty, rotz);
    BuildPiecewiseMap(cam, xmap, ymap, focal_x, focal_y, x_view_degree, y_view_degree, R);
}

void BuildPiecewiseMap(FishEyeCam& cam, cv::Mat& xmap, cv::Mat& ymap, float focal_x, float focal_y, float x_view_degree, float y_view_degree, Mat& R)
{
    int width = round(focal_x*tan(ToRad(x_view_degree)/2))*2;
    int height = round(focal_y*tan(ToRad(y_view_degree)/2))*2;

    Size map_size = Size(width, height);
    xmap = Mat::zeros(map_size, CV_32F);
    ymap = Mat::zeros(map_size, CV_32F);

    Mat K = (Mat_<double>(3,3)<<focal_x, 0.0, width/2.0, 0.0, focal_y, height/2.0, 0.0, 0.0, 1.0);
    Mat r_kinv = R*K.inv();

    for(int r=0; r<height; r++)
    {
        for(int c=0; c<width; c++)
        {
            // image points to camera points
            float x = r_kinv.at<double>(0, 0)*c+r_kinv.at<double>(0, 1)*r+r_kinv.at<double>(0,2);
            float y = r_kinv.at<double>(1, 0)*c+r_kinv.at<double>(1, 1)*r+r_kinv.at<double>(1,2);
            float z = r_kinv.at<double>(2, 0)*c+r_kinv.at<double>(2, 1)*r+r_kinv.at<double>(2,2);

            // camera points to fisheye plane
            if(z>0)
            {
                x = x/z;
                y = y/z;
            }else
                continue;

            // fisheye plane to distort iamge points
            vector<Point2f> points={Point2f(x, y)};
            vector<Point2f> distort;
            fisheye::distortPoints(points, distort, cam.K_, cam.D_);
            xmap.at<float>(r,c) = distort[0].x;
            ymap.at<float>(r,c) = distort[0].y;
        }
    }
}

void BuildPiecewiseMap2(FishEyeCam& cam, cv::Mat& xmap, cv::Mat& ymap, float focal_x, float focal_y, float x_view_degree, float y_view_degree, Mat& R)
{
    int width = round(focal_x*tan(ToRad(x_view_degree)/2))*2;
    int height = round(focal_y*tan(ToRad(y_view_degree)/2))*2;

    Size map_size = Size(width, height);
    xmap = Mat::zeros(map_size, CV_32F);
    ymap = Mat::zeros(map_size, CV_32F);

    Mat K = (Mat_<double>(3,3)<<focal_x, 0.0, width/2.0, 0.0, focal_y, height/2.0, 0.0, 0.0, 1.0);
    Mat r_kinv = R*K.inv();

    for(int r=0; r<height; r++)
    {
        for(int c=0; c<width; c++)
        {
            // image points to camera points
            float x = r_kinv.at<double>(0, 0)*c+r_kinv.at<double>(0, 1)*r+r_kinv.at<double>(0,2);
            float y = r_kinv.at<double>(1, 0)*c+r_kinv.at<double>(1, 1)*r+r_kinv.at<double>(1,2);
            float z = r_kinv.at<double>(2, 0)*c+r_kinv.at<double>(2, 1)*r+r_kinv.at<double>(2,2);

            // camera points to fisheye plane
            if(z>0)
            {
                x = x/z;
                y = y/z;
            }else
                continue;

            // fisheye plane to distort iamge points
            vector<Point2f> points={Point2f(x, y)};
            vector<Point2f> distort;
            fisheye::distortPoints(points, distort, cam.K_, cam.D_);
            xmap.at<float>(r,c) = distort[0].x;
            ymap.at<float>(r,c) = distort[0].y;
        }
    }
}




void BuildPiecewise5QuadrantMap(FishEyeCam& cam, cv::Mat& xmap, cv::Mat& ymap, float focal, float x_view_degree, float y_view_degree, float rotx, float roty, float rotz)
{
    assert(x_view_degree<180.0f && y_view_degree<180.0f);

    float x_view_rest_degree = 180.0f-x_view_degree;
    float y_view_rest_degree = 180.0f-y_view_degree;

    // for mid, left, right, up, down map
    Mat extra_rot = genRotMatrix(rotx, roty, rotz);    
    float rx[] = {0.0f, 0.0f, 0.0f, 90.0f, -90.0f};
    float ry[] = {0.0f, -90.0f, 90.0f, 0.0f, 0.0f};
    float x_degree[] = {x_view_degree, x_view_rest_degree, x_view_rest_degree, x_view_degree, x_view_degree};
    float y_degree[] = {y_view_degree, y_view_degree, y_view_degree, y_view_rest_degree, y_view_rest_degree};
    float extra_focal_x = focal * tan(ToRad(x_view_degree)/2);
    float extra_focal_y = focal * tan(ToRad(y_view_degree)/2);
    //float focals_x[] = {focal, extra_focal_x, extra_focal_x, focal, focal};
    //float focals_y[] = {focal, focal, focal, extra_focal_y, extra_focal_y};
    float focals_x[] = {focal, focal, focal, focal, focal};
    float focals_y[] = {focal, focal, focal, focal, focal};

    cout<<"x_view degree "<<x_view_degree<<", extra x view degree "<<x_view_rest_degree<<endl;
    cout<<"focal: "<<focal<<", extra focal x "<<extra_focal_x<<", extra focal y "<<extra_focal_y<<endl;

    vector<Mat> vec_mapx, vec_mapy;
    vec_mapx.resize(5);
    vec_mapy.resize(5);
    for(int i=0; i<5; i++)
    {
        Mat R = extra_rot*genRotMatrix(rx[i], ry[i], rotz);
        BuildPiecewiseMap(cam, vec_mapx[i], vec_mapy[i], focals_x[i], focals_y[i], x_degree[i], y_degree[i], R);
    }

    // copy map to while map
    int mid_width = vec_mapx[0].cols;
    int mid_height = vec_mapx[0].rows;
    int rest_width = vec_mapx[1].cols;
    int rest_height = vec_mapx[3].rows;

    xmap = Mat::zeros(Size(mid_width+rest_width, mid_height+rest_height), CV_32F);
    ymap = Mat::zeros(xmap.size(), xmap.type());

    Rect mid_rect = Rect(rest_width/2, rest_height/2, mid_width, mid_height);
    Rect left_rect = Rect(0, rest_height/2, rest_width/2, mid_height);
    Rect right_rect = Rect(rest_width/2+mid_width, rest_height/2, rest_width/2, mid_height);
    Rect up_rect = Rect(rest_width/2, 0, mid_width, rest_height/2);
    Rect down_rect = Rect(rest_width/2, rest_height/2+mid_height, mid_width, rest_height/2);

    Rect ori_mid_rect = Rect(0, 0, mid_width, mid_height);
    Rect ori_left_rect = Rect(rest_width/2, 0, rest_width/2, mid_height);
    Rect ori_right_rect = Rect(0, 0, rest_width/2, mid_height);
    Rect ori_up_rect = Rect(0, rest_height/2, mid_width, rest_height/2);
    Rect ori_down_rect = Rect(0, 0, mid_width, rest_height/2);

    vector<Rect> roi_rects = {mid_rect, left_rect, right_rect, up_rect, down_rect};
    vector<Rect> ori_roi_rects = {ori_mid_rect, ori_left_rect, ori_right_rect, ori_up_rect, ori_down_rect};
    for(int i=0; i<5; i++)
    {
        vec_mapx[i](ori_roi_rects[i]).copyTo(xmap(roi_rects[i]));
        vec_mapy[i](ori_roi_rects[i]).copyTo(ymap(roi_rects[i]));
    }
}
