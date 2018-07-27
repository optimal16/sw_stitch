#include "WorldPlane.h"
#include <fstream>
#include <iostream>
#include <math.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

Size WorldPlane::GetOutputSize(float viewrange, float pix_dis)
{
    int pixes = round(viewrange/pix_dis);
    Size output_size = Size(pixes, pixes);
    return output_size;
}

vector<IndexPoint> WorldPlane::InitPoints(float left_right_to_front_distance, float car_y,float car_x, float viewrange, float pix_dis)
{
    this->left_right_to_front_distance_ = left_right_to_front_distance;
    this->car_y_ = car_y;
    this->car_x_ = car_x;
    this->viewrange_ = viewrange;
    this->pix_dis_ = pix_dis;

    Point2f mid_point = Point2f(viewrange/2.0f, viewrange/2.0f);
    vector<Point2f> front_points, right_points, back_points,left_points;

    WorldPlane::InitPatternPoints(front_points);

    float small_square = 0.2f;
    float big_square = 0.6f;
    std::cout<<"car_x: "<<car_x<<endl;
    // Point2f st_front = Point2f(0.0f, 0.0f);
    // Point2f st_right = Point2f(4*big_square+9*small_square, small_square+left_right_to_front_distance-big_square);
    // Point2f st_back = Point2f(4*big_square+9*small_square, 4*small_square+2*big_square+car_y);
    // Point2f st_left = Point2f(0, 3*big_square+10*small_square+left_right_to_front_distance);
    #if 0
    Point2f st_front = Point2f(0.0f, 0.0f);
    Point2f st_right = Point2f(4*big_square+9*small_square, small_square+left_right_to_front_distance-big_square);
    Point2f st_back = Point2f(4*big_square+9*small_square, 4*small_square+2*big_square+car_y);
    Point2f st_left = Point2f(0, 3*big_square+10*small_square+left_right_to_front_distance);
    #else
    // float car_x = 4.0;
    Point2f st_front = Point2f(0.0f, 0.0f);
    Point2f st_right = Point2f(2*big_square+9.5*small_square + car_x/2.0 , small_square+left_right_to_front_distance-big_square);
    Point2f st_back = Point2f(4*big_square+9*small_square, 4*small_square+2*big_square+car_y);
    Point2f st_left = Point2f(2 * big_square - 0.5 * small_square - car_x/2.0, 3*big_square+10*small_square+left_right_to_front_distance);
    #endif
    rotatePoints(front_points, right_points, 90.0f, st_right-st_front);
    rotatePoints(front_points, back_points, 180.0f, st_back-st_front);
    rotatePoints(front_points, left_points, 270.0f, st_left-st_front);

    int point_num = 32;
    index_points_.resize(point_num*4);

    vector<vector<Point2f>> vec_points = {front_points, right_points, back_points, left_points};
    float shift_x = (viewrange-4.2)/2.0f;
    float shift_y = (viewrange-car_y)/2.0f-big_square-2*small_square;
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<point_num; j++)
        {
            Point2f wpt = vec_points[i][j]+Point2f(shift_x, shift_y);
            Point pt = Point(wpt.x/pix_dis_, wpt.y/pix_dis_);
            index_points_[i*point_num+j] = IndexPoint{i*point_num+j, wpt-mid_point, pt};
        }
    }

    return index_points_;
}

void WorldPlane::rotatePoints(vector<Point2f>& points, vector<Point2f>& rot_points, float angle, cv::Point2f st)
{
    float rad = angle/180*3.1415926;
    float sinx = sin(rad);
    float cosx = cos(rad);

    int point_num = (int)points.size();
    for(int i=0; i<point_num; i++)
    {
        Point2f p = points[i];
        float x = -p.y*sinx+p.x*cosx;
        float y = p.y*cosx+p.x*sinx;
        rot_points.push_back(Point2f(x, y)+st);
    }
}

void WorldPlane::InitPatternPoints(vector<Point2f>& points)
{
    float small_square = 0.2f;
    float big_square = 0.6f;

    Point2f left_st = Point2f(small_square, small_square);
    Point2f mid_st = Point2f(2*big_square+2*small_square, small_square);
    Point2f right_st = Point2f(3*big_square+8*small_square, small_square);

    for(int r=0; r<4; r++)
    {
        for(int c=0; c<6; c++)
        {
            points.push_back(mid_st + Point2f(c*small_square, r*small_square));
        }
    }

    for(int r=0; r<2; r++)
    {
        for(int c=0; c<2; c++)
        {
            points.push_back(left_st + Point2f(c*big_square, r*big_square));
        }
    }

    for(int r=0; r<2; r++)
    {
        for(int c=0; c<2; c++)
        {
            points.push_back(right_st + Point2f(c*big_square, r*big_square));
        }
    }
}

vector<Point2f> WorldPlane::GetWorldPlanePoints(vector<int>& point_index)
{
    vector<Point2f> plane_points;
    for(auto index:point_index)
        plane_points.push_back(index_points_[index].pt_);
    return plane_points;
}

vector<Point3f> WorldPlane:: GetWorldPoints(vector<int>& point_index)
{
    vector<Point3f> world_points;
    for(auto index:point_index)
    {
        Point2f wpt = index_points_[index].wpt_;
        world_points.push_back(Point3f(wpt.x, wpt.y, 0.0f));
    }
    return world_points;
}

void WorldPlane::Test(float lr2f, float cy,float cx, float vr, float pd)
{
    vector<IndexPoint> index_points = InitPoints(lr2f, cy, cx, vr, pd);
    int expand_ratio = 5;
    Size output_size = GetOutputSize(vr, pd)*expand_ratio;
    cout<<"output size "<<output_size<<endl;
    Mat plot = Mat(output_size, CV_8UC3, Vec3b(255,255,255));
    for(auto ip:index_points)
    {
        Point p = Point(ip.pt_.x, ip.pt_.y)*expand_ratio;
        cout<<"world point "<<ip.wpt_<<endl;
        circle(plot, p, 2, Scalar(0, 0, 255), 2, CV_AA);
        putText(plot, to_string(ip.index_), p, cv::FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255 , 0), 2);
    }
    imwrite("world_plane.jpg", plot);
}

Mat WorldPlane::Plot(int scale)
{
    Size output_size = GetOutputSize(viewrange_, pix_dis_)*scale;
    cout<<"output size "<<output_size<<endl;
    Mat plot = Mat(output_size, CV_8UC3, Vec3b(255,255,255));
    for(auto ip:index_points_)
    {
        Point p = Point(ip.pt_.x, ip.pt_.y)*scale;
        circle(plot, p, 1, Scalar(0, 0, 255), 1, CV_AA);
    }

    float calc_small_x = index_points_[1].wpt_.x-index_points_[0].wpt_.x;
    float calc_small_y = index_points_[6].wpt_.y-index_points_[0].wpt_.y;

    float calc_car_x = index_points_[30].wpt_.x-index_points_[27].wpt_.x - 2*calc_small_x;
    float calc_left_right_to_front = index_points_[119].wpt_.y-index_points_[27].wpt_.y - 2*calc_small_y;
    float calc_car_y = index_points_[87].wpt_.y - index_points_[18].wpt_.y -2*calc_small_y;

    putText(plot,string("car_x ") + to_string(calc_car_x), Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255 , 0), 2);
    putText(plot,string("lr2f ") + to_string(calc_left_right_to_front), Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255 , 0), 2);
    putText(plot,string("car_y ") + to_string(calc_car_y), Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255 , 0), 2);

    return plot;
}
