#include "OnlineCalibrate.h"
#include "Composition.h"
#include <iostream>
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace cv;
using namespace std;

void Mat2Vec(Mat& m, vector<double>& vec)
{
    int num = m.rows*m.cols;
    if(num==0)
        return;

    vec.resize(num);
    for(int r=0; r<m.rows; r++)
    {
        for(int c=0; c<m.cols; c++)
        {
            vec[r*m.cols+c] = m.at<double>(r, c); 
        }
    }
}


OnlineCalibrate::OnlineCalibrate(StitchParam& param, vector<FishEyeCam>& vec_cam)
{
    vec_r_.resize(4);
    vec_t_.resize(4);
    vec_map_.resize(4);
    vec_mask_.resize(4);
    vec_left_map_.resize(4);
    vec_right_map_.resize(4);
    vec_cam_ = vec_cam;
    
    viewrange_ = param.viewrange_;
    pix_dis_ = param.pix_dis_;

    Rect inner_rect = param.inner_rect_;
    int pixes = round(viewrange_/pix_dis_);
    Rect tl_rect = Rect(0, 0, inner_rect.x, inner_rect.y);
    Rect tr_rect = Rect(inner_rect.br().x, 0, pixes-inner_rect.br().x, inner_rect.y);
    Rect br_rect = Rect(inner_rect.br().x, inner_rect.br().y, pixes-inner_rect.br().x, pixes-inner_rect.br().y);
    Rect bl_rect = Rect(0, inner_rect.br().y, inner_rect.x, pixes-inner_rect.br().y);

    vec_rect_ = {tl_rect, tr_rect, br_rect, bl_rect};

    for(int i=0; i<4; i++)
    {
        Mat vecR = param.Rs_[i];
        Mat R;
        Rodrigues(vecR, R);
        Mat2Vec(R, vec_r_[i]);
        Mat2Vec(param.Ts_[i], vec_t_[i]);
        GenerateHWarpMapAndMask(vec_map_[i], vec_mask_[i], vec_cam_[i], param.Rs_[i], param.Ts_[i], viewrange_, pix_dis_);
    }

    for(int i=0; i<4; i++)
    {
        int prev = (i+3)%4;
        vec_map_[prev](vec_rect_[i]).copyTo(vec_left_map_[i]);
        vec_map_[i](vec_rect_[i]).copyTo(vec_right_map_[i]);
    }
}

OnlineCalibrate::~OnlineCalibrate()
{
    vec_r_.clear();
    vec_t_.clear();
    vec_map_.clear();
    vec_mask_.clear();
    vec_cam_.clear();
}

void OnlineCalibrate::RefinePlane(double& roll, double& pitch, double& z, vector<OverlapPoints>& points)
{
    
    
}

void OnlineCalibrate::ExtractOverlapPoints(vector<OverlapPoints>& points, vector<Mat>& vec_src, StitchParam& param)
{

    Ptr<ORB> orb = ORB::create();

    for(int i=0; i<4; i++)
    {
        int prev = (i+3)%4;
        Mat left, right;
        remap(vec_src[prev], left, vec_left_map_[i], Mat(), CV_INTER_LINEAR);
        remap(vec_src[i], right, vec_right_map_[i], Mat(), CV_INTER_LINEAR);

        imwrite(to_string(i)+"_0.jpg", left);
        imwrite(to_string(i)+"_1.jpg", right);

        vector<KeyPoint> kp1, kp2;
        Mat dp1, dp2;
        orb->detectAndCompute(left, Mat(), kp1, dp1);
        orb->detectAndCompute(right, Mat(), kp2, dp2);
       
        Mat plotKp1, plotKp2; 
        drawKeypoints(left, kp1, plotKp1);
        drawKeypoints(right, kp2, plotKp2);
        imwrite(to_string(i)+"_0_kp.jpg", plotKp1);
        imwrite(to_string(i)+"_1_kp.jpg", plotKp2);

        vector<DMatch> matches;
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
        matcher->match(dp1, dp2, matches);
        cout<<"find out total "<<matches.size()<<" matches"<<endl;

        Mat showMatch;
        drawMatches(left, kp1, right, kp2, matches, showMatch);
        imwrite(to_string(i)+"_match.jpg", showMatch);
    }

}

void readPoints(string line, Point2f p1, Point2f p2)
{

    vector<float> data;
    int index=line.find_first_of(",;");
    while(index!=(int)string::npos)
    {
        float d= atof(line.substr(0, index).c_str());
        line = line.substr(index+1);
        data.push_back(d);
        index = line.find_first_of(",;");
    }
   
    p1.x = data[0];
    p1.y = data[1];
    p2.x = data[2];
    p2.y = atof(line.c_str());
}

int OnlineCalibrate::ReadOverlapPoints(string filename, vector<OverlapPoints>& points, int pos)
{
    ifstream ifile;
    ifile.open(filename, ios::in);
    if(!ifile.is_open())
    {
        cout<<"read overlap points file failed! "<<filename<<endl;
        return -1;
    }

    points.clear();
    string line;
    while(getline(ifile, line))
    {
        Point2f p1, p2;
        if(line.empty())
            continue;
        readPoints(line, p1, p2);
        cout<<"read points "<<p1<<", "<<p2<<endl;
        points.push_back(OverlapPoints{p1, p2, pos});
    }

    ifile.close();
    return 0;
}

void OnlineCalibrate::AddStPoints(vector<OverlapPoints>& points, Point st)
{
    for(auto op:points)
    {
        op.p1 += Point2f(st.x, st.y);
        op.p2 += Point2f(st.x, st.y);
    }
}

void OnlineCalibrate::AddStPoints(vector<OverlapPoints>& points, int pos)
{
    Point st = vec_rect_[pos].tl();
    AddStPoints(points, st);
}
