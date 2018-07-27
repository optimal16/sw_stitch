#include <math.h>
#include "Composition.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <assert.h>
#include <iostream>

using namespace std;
using namespace cv;

void GenerateHWarpMapAndMask(cv::Mat& map, cv::Mat& mask, FishEyeCam& cam, cv::Mat& H, cv::Size output_size)
{
    int point_num = output_size.width*output_size.height;
    vector<Point2f> image_points(point_num);
    mask = Mat::zeros(output_size, CV_8U);
    Mat pic = imread("../debug/left.jpg",1);
    Mat Hinv = H.inv();
//    cout<<"Hinv "<<Hinv<<endl;
    for(int r=0; r<output_size.height; r++)
    {
        for(int c=0; c<output_size.width; c++)
        {
            Point3f p = Point3f(c, r, 1.0f);
            float scale = Hinv.at<double>(2, 0)*p.x + Hinv.at<double>(2, 1)*p.y + Hinv.at<double>(2,2)*p.z;
            float x = (Hinv.at<double>(0, 0)*p.x + Hinv.at<double>(0, 1)*p.y + Hinv.at<double>(0,2)*p.z)/scale;
            float y = (Hinv.at<double>(1, 0)*p.x + Hinv.at<double>(1, 1)*p.y + Hinv.at<double>(1,2)*p.z)/scale;
            if(scale<0.0f)
            {
                x=y=-10000.0f;
            }else
                mask.at<uchar>(r, c)=255;

            image_points[r*output_size.width+c] = Point2f(x, y);
        }
    }

    vector<Point2f> disort_points;
    fisheye::distortPoints(image_points, disort_points, cam.K_, cam.D_);

    // cv::Mat map_ = Mat::zeros(output_size, CV_32UC2);
    map = Mat::zeros(output_size, CV_32FC2);
    for(int r=0; r<map.rows; r++)
    {
        for(int c=0; c<map.cols; c++)
        {
            Point2f p = disort_points[r*map.cols+c];
            map.at<Vec2f>(r, c) = Vec2f(p.x, p.y);
            //if((mask.at<uchar>(r, c)=255) && (p.x<0.0 || p.x>cam.size.width-1 || p.y<0.0 || p.y>cam.size.height-1))
            if(p.x<0.0 || p.x>cam.size_.width-1 || p.y<0.0 || p.y>cam.size_.height-1)
                mask.at<uchar>(r, c)=0;
        }
    }
    imwrite("../debug/test_mask.png",mask);
    std::cout<<"map channel: "<<map.type()<<endl;
    // map.convertTo(map_,CV_32UC2,1,0);
    // std::cout<<"map_ channel: "<<map_.type()<<endl;
    // std::cout<<"map_ channel: "<<map_<<endl;
    // imwrite("../debug/test_map.png",map_);

}

void CalcInnerRect(vector<Mat>& vec_masks, Rect& inner_rect)
{
    assert(vec_masks.size()==4);
    Size output_size = vec_masks[0].size();
    assert(output_size.height>0 && output_size.width>0);
    Mat whole_mask = Mat::zeros(output_size, CV_8U);
    whole_mask = vec_masks[0] | vec_masks[1] | vec_masks[2] | vec_masks[3];

    // calc top y
    int minx=0, maxx=0, miny=0, maxy=0;
    for(int r=0; r<whole_mask.rows; r++)
    {
        uchar* ptr = whole_mask.ptr<uchar>(r);
        bool jump = false;
        for(int c=0; c<whole_mask.cols; c++)
        {
            if(ptr[c]==0)
            {
                miny = r-1;
                jump = true;
                break;
            }
            
        }
        if(jump)
            break;
    }

    if(miny<0)
    {
        cout<<"calc inner rect, miny map error"<<endl;
        return;
    }
    
    // calc left x
    for(int c=0; c<whole_mask.cols; c++)
    {
        bool jump = false;
        for(int r=miny; r<whole_mask.rows; r++)
        {
            if(whole_mask.at<uchar>(r, c)==0)
            {
                minx= c-1;
                jump = true;
                break;
            }
            
        }
        if(jump)
            break;
    }

    // calc bot y
    for(int r=whole_mask.rows-1; r>miny; r--)
    {
        uchar* ptr = whole_mask.ptr<uchar>(r);
        bool jump = false;
        for(int c=0; c<whole_mask.cols; c++)
        {
            if(ptr[c]==0)
            {
                maxy = r+1;
                jump = true;
                break;
            }
        }
        if(jump)
            break;
    }
    
    if(maxy>whole_mask.rows-1)
    {
        cout<<"clac inner rect, maxy map error"<<endl;
    }

    // calc right x
    for(int c=whole_mask.cols-1; c>minx; c--)
    {
        bool jump = false;
        for(int r=0; r<whole_mask.rows; r++)
        {
            if(whole_mask.at<uchar>(r,c)==0)
            {
                maxx = c+1;
                jump = true;
                break;
            }
        }
        if(jump)
            break;
    }
 
    inner_rect = Rect(minx, miny, maxx-minx+1, maxy-miny+1);
    cout<<"inner_rect is "<<inner_rect<<endl;

    // TODO debug
    Mat plot;
    cvtColor(whole_mask, plot, CV_GRAY2BGR);
    rectangle(plot, inner_rect.tl(), inner_rect.br(), Scalar(0, 0, 255));
    imwrite("whole_mask.jpg", plot);
}

Size GenSeparateMask(vector<Mat>& vec_masks, vector<Point>& vec_st_points, Size output_size, Rect inner_rect, int extend_off, int cut_off, int lean_pix)
{
    if(lean_pix<0)
    {
        cout<<"lean_pix set to 0"<<endl;
        lean_pix = 0;
    }

    Rect bounding_rect = Rect(inner_rect.x-extend_off, inner_rect.y-extend_off, inner_rect.width+2*extend_off, inner_rect.height+2*extend_off);

    Rect outer_rect = Rect(cut_off, cut_off, output_size.width-2*cut_off, output_size.height-2*cut_off);

    Point tl = Point(bounding_rect.x-lean_pix, outer_rect.y);
    Point tr = Point(bounding_rect.br().x+lean_pix, outer_rect.y);
    Point bl = Point(bounding_rect.x-lean_pix, outer_rect.br().y);
    Point br = Point(bounding_rect.br().x+lean_pix, outer_rect.br().y);

    int left_minx = tl.x;
    int left_maxx = bounding_rect.x;
    if(tl.x>bounding_rect.x)
    {
        left_minx = bounding_rect.x;
        left_maxx = tl.x;
    }

    int right_minx = bounding_rect.br().x;
    int right_maxx = tr.x;
    if(tr.x<bounding_rect.br().x)
    {
        right_minx = tr.x;
        right_maxx = bounding_rect.br().x;
    }

    Rect left_rect = Rect(cut_off, cut_off, left_maxx-cut_off, outer_rect.height);
    Rect front_rect = Rect(left_minx, cut_off, right_maxx-left_minx, bounding_rect.y-cut_off);
    Rect right_rect = Rect(right_minx, cut_off, outer_rect.br().x-right_minx, outer_rect.height);
    Rect back_rect = Rect(left_minx, bounding_rect.br().y, right_maxx-left_minx, outer_rect.br().y-bounding_rect.br().y);

    auto line=[&](float x1, float y1, float x2, float y2, float x, float y){ return (y2-y1)*(x-x1)-(x2-x1)*(y-y1); };

    auto line1 = [&](float x, float y){ return line(tl.x, tl.y, bounding_rect.x, bounding_rect.y, x, y); };
    auto line2 = [&](float x, float y){ return line(tr.x, tr.y, bounding_rect.br().x, bounding_rect.y, x, y); };
    auto line3 = [&](float x, float y){ return line(bounding_rect.x, bounding_rect.br().y, bl.x, bl.y, x ,y); };
    auto line4 = [&](float x, float y){ return line(bounding_rect.br().x, bounding_rect.br().y, br.x, br.y, x, y); };

    vector<Rect> vec_rects = {front_rect, right_rect, back_rect, left_rect};
    vec_st_points = {front_rect.tl(), right_rect.tl(), back_rect.tl(), left_rect.tl()};
    vec_masks.resize(4);
    for(int i=0; i<4; i++)
        vec_masks[i]=Mat(vec_rects[i].size(), CV_8U, 255);

    // front
    for(int r=0; r<vec_masks[0].rows; r++)
    {
        for(int c=0; c<vec_masks[0].cols; c++)
        {
            int x = c+front_rect.x;
            int y = r+front_rect.y;
            if(line1(x, y)<0 || line2(x,y)>0)
                vec_masks[0].at<uchar>(r, c)=0;
        }
    }

    // right
    for(int r=0; r<vec_masks[1].rows; r++)
    {
        for(int c=0; c<vec_masks[1].cols; c++)
        {
            int x = c+right_rect.x;
            int y = r+right_rect.y;
            if(line2(x, y)<0 || line4(x, y)<0)
                vec_masks[1].at<uchar>(r, c)=0;
        }
    }

    // back
    for(int r=0; r<vec_masks[2].rows; r++)
    {
        for(int c=0; c<vec_masks[2].cols; c++)
        {
            int x = c+back_rect.x;
            int y = r+back_rect.y;
            if(line3(x, y)<0 || line4(x, y)>0)
                vec_masks[2].at<uchar>(r, c)=0;
        }
    }

    // left
    for(int r=0; r<vec_masks[3].rows; r++)
    {
        for(int c=0; c<vec_masks[3].cols; c++)
        {
            int x = c+left_rect.x;
            int y = r+left_rect.y;
            if(line1(x, y)>0 || line3(x, y)>0)
                vec_masks[3].at<uchar>(r, c)=0;
        }
    }
    
    // TODO just debug image
    Mat plot = Mat::zeros(output_size, CV_8UC3);
    rectangle(plot, inner_rect.tl(), inner_rect.br(), Scalar(0, 255, 0), 1);
    rectangle(plot, outer_rect.tl(), outer_rect.br(), Scalar(255, 255, 0), 1);
    rectangle(plot, bounding_rect.tl(), bounding_rect.br(), Scalar(0, 255, 255), 1);
    rectangle(plot, tl, br, Scalar(0, 0, 255), 1);
    rectangle(plot, front_rect.tl(), front_rect.br(), Scalar(255, 255, 255), 1);
    rectangle(plot, right_rect.tl(), right_rect.br(), Scalar(255, 255, 255), 1);
    rectangle(plot, back_rect.tl(), back_rect.br(), Scalar(255, 255, 255), 1);
    rectangle(plot, left_rect.tl(), left_rect.br(), Scalar(255, 255, 255), 1);

    circle(plot, tl, 1, Scalar(0, 0, 255), 1, CV_AA);
    circle(plot, tr, 1, Scalar(0, 0, 255), 1, CV_AA);
    circle(plot, bl, 1, Scalar(0, 0, 255), 1, CV_AA);
    circle(plot, br, 1, Scalar(0, 0, 255), 1, CV_AA);

    imwrite("separate_plot.jpg", plot);
    
    cout<<"separate mask front "<<vec_rects[0]<<endl;
    cout<<"separate mask right"<<vec_rects[1]<<endl;
    cout<<"separate mask back"<<vec_rects[2]<<endl;
    cout<<"separate mask left"<<vec_rects[3]<<endl;

    return outer_rect.size();
}

void SeparateMap(vector<Mat>& vec_sep_maps, vector<Mat>& vec_maps, vector<Mat>& vec_masks, vector<Point>& vec_st_points)
{
    vec_sep_maps.resize(4);
    for(int i=0; i<4; i++)
    {
        Rect rect = Rect(vec_st_points[i], vec_masks[i].size());
        vec_maps[i](rect).copyTo(vec_sep_maps[i], vec_masks[i]);
    }
}

void RoundMap(vector<Mat>& vec_round_maps, vector<Mat>& vec_maps)
{
    int num = vec_maps.size();
    assert(num==4);
    vec_round_maps.resize(4);
    for(int i=0; i<num; i++)
    {
        vec_maps[i].convertTo(vec_round_maps[i], CV_32SC2, 1.0);
    }
}

void RemapWidthIndexMap(Mat& dst, vector<Mat>& inputs, vector<Mat>& vec_maps, vector<Mat>& vec_masks, vector<Point>& vec_st_points, Size output_size)
{
    if(dst.size()!=output_size)
        dst = Mat::zeros(output_size, CV_8UC3);
    
    for(int i=0; i<4; i++)
    {
        Point tl = vec_st_points[i] - vec_st_points[3];
        Size map_size = vec_maps[i].size();
        //Point br = tl + Point(map_size.width-1, map_size.height-1); 
        for(int r=0; r<map_size.height; r++)
        {
            uchar* ptr_mask = vec_masks[i].ptr<uchar>(r);
            Vec3b* ptr_dst = dst.ptr<Vec3b>(r+tl.y);
            Vec2i* ptr_map = vec_maps[i].ptr<Vec2i>(r);
            for(int c=0; c<map_size.width; c++)
            {
                if(ptr_mask[c]==255)
                {
                    Vec2i v = ptr_map[c];
                    ptr_dst[c+tl.x] = inputs[i].at<Vec3b>(v[1], v[0]);
                }
            }
        }

        imwrite("stitch_"+to_string(i)+".jpg", dst);
    }
}


void GenerateHWarpMapAndMask(cv::Mat& map, cv::Mat& mask, FishEyeCam& cam, cv::Mat& r, cv::Mat& t, float viewrange, float pix_dis)
{
    int pixes = round(viewrange/pix_dis);
    int mid = pixes/2;
    Size output_size = Size(pixes, pixes);
    mask = Mat::zeros(output_size, CV_8U);
    map = Mat::zeros(output_size, CV_32FC2);

    Mat R;
    Rodrigues(r, R);

    int point_num = pixes*pixes;
    vector<Point2f> image_points(point_num);
    for(int r=0; r<output_size.height; r++)
    {
        for(int c=0; c<output_size.width; c++)
        {
            Point3f p = Point3f(1.0f*(c-mid)/pixes*viewrange, 1.0f*(r-mid)/pixes*viewrange, 0.0f);
            float scale = R.at<double>(2, 0)*p.x + R.at<double>(2, 1)*p.y + R.at<double>(2,2)*p.z + t.at<double>(2, 0);
            float x = (R.at<double>(0, 0)*p.x + R.at<double>(0, 1)*p.y + R.at<double>(0,2)*p.z + t.at<double>(0, 0))/scale;
            float y = (R.at<double>(1, 0)*p.x + R.at<double>(1, 1)*p.y + R.at<double>(1,2)*p.z + t.at<double>(1, 0))/scale;
            if(scale<0.0f)
            {
                x=y=-10000.0f;
            }else
                mask.at<uchar>(r, c)=255;

            image_points[r*output_size.width+c] = Point2f(x, y);
        }
    }

    vector<Point2f> disort_points;
    fisheye::distortPoints(image_points, disort_points, cam.K_, cam.D_);

    map = Mat::zeros(output_size, CV_32FC2);
    for(int r=0; r<map.rows; r++)
    {
        for(int c=0; c<map.cols; c++)
        {
            Point2f p = disort_points[r*map.cols+c];
            map.at<Vec2f>(r, c) = Vec2f(p.x, p.y);
            //if((mask.at<uchar>(r, c)=255) && (p.x<0.0 || p.x>cam.size.width-1 || p.y<0.0 || p.y>cam.size.height-1))
            if(p.x<0.0 || p.x>cam.size_.width-1 || p.y<0.0 || p.y>cam.size_.height-1)
                mask.at<uchar>(r, c)=0;
        }
    }
    // imwrite("../debug/test_map.jpg",map);

}
