#include "Calibrate.h"
#include "Composition.h"
#include "HomographyCost.h"
#include "ceres/rotation.h"
#include <opencv2/calib3d.hpp>
#include "filedir.h"

using namespace std;
using namespace cv;
using namespace cv::fisheye;

Calibrate::Calibrate(string config_file)
{
    this->config_file_ = config_file;
    image_size_ = Size(1280, 720);
    output_size_ = Size(600, 600);
    images_.resize(4);
    debug_ = true;
}

void Calibrate::Init()
{
    // load calib param
    if(ReadCalibParam(config_file_, calib_param_)!=0)
        exit(-1);
    
    stitch_param_.Set(calib_param_);
    // load intrinsic param
    if(ReadFishEyeCamera(calib_param_.intrinsic_file_, fisheye_cams_)!=0)
        exit(-1);

    // load wold plan information and image point information 
    wplane_.InitPoints(calib_param_.left_right_to_front_distance_, calib_param_.car_y_, calib_param_.car_x_, calib_param_.viewrange_, calib_param_.pix_dis_);
    output_size_ = wplane_.GetOutputSize(calib_param_.viewrange_, calib_param_.pix_dis_);

    string points_file[] = {FRONT_POINTS_FILE, RIGHT_POINTS_FILE, BACK_POINTS_FILE, LEFT_POINTS_FILE};
    vec_image_points_.clear();
    for(int i=0; i<4; i++)
    {
        vector<Point2f> load_points;
        if(CFindPoints::ReadImagePoints(calib_param_.image_points_dir_+points_file[i], load_points)!=0)
            exit(-1);

        vector<ImagePoints> image_points ;
        CFindPoints::IndexFindPoints(load_points, image_points, CameraPos(i));
        vec_image_points_.push_back(image_points);
        cout<<"for "<<i<<", load points "<<load_points.size()<<", point 0 "<<load_points[0]<<endl;
    }

    cout<<"Load calibrate information success!"<<endl<<endl;

    // load debug image, if success, enter debug mode 
    string image_files[] = {FRONT_IMAGE, RIGHT_IMAGE, BACK_IMAGE, LEFT_IMAGE};
    for(int i=0; i<4; i++)
    {
        images_[i] = imread(calib_param_.image_points_dir_ + image_files[i]);
        if(images_[i].empty())
        {
            debug_ = false;
            break;
        }
    }

    if(debug_)
    {
        image_size_ = images_[0].size();
        cout<<"load debug image success, enter debug mode!"<<endl;

        string debug_dir = calib_param_.image_points_dir_ + "calib/";
        MkDirs(debug_dir);

        Mat plot_plane = wplane_.Plot();
        imwrite(debug_dir+"plane.jpg", plot_plane);
    }
}

Calibrate::~Calibrate()
{
    vec_image_points_.clear();
    fisheye_cams_.clear();
    images_.clear();
}

double computer_reproject_error(cv::Mat& H, std::vector<cv::Point2f> image_points, std::vector<cv::Point2f> project_points, double& max_err, double& average_err)
{
    double total_err = 0.0;
    int image_points_size = image_points.size();
    int max_index=-1;
    max_err = 0.0f;
    Point2f max_err_ip_pp, max_err_pp;
    for (int i=0;  i<image_points_size;  i++) 
    {
        cv::Point2f point;
        cv::Point2f ip = image_points[i];
        double scale = H.at<double>(2, 0)*ip.x + H.at<double>(2, 1)*ip.y + 1.0;
        point.x = (H.at<double>(0, 0)*ip.x + H.at<double>(0, 1)*ip.y+H.at<double>(0, 2))/scale;
        point.y = (H.at<double>(1, 0)*ip.x + H.at<double>(1, 1)*ip.y+H.at<double>(1, 2))/scale;
        Point2f diff_point = point - project_points[i];
        double err = sqrt(diff_point.dot(diff_point));//cv::norm(point, project_points[i], NORM_L2);
        if(max_err<err)
        {
            max_err = err;
            max_index = i;
            max_err_ip_pp = point;
            max_err_pp = project_points[i];
        }
        total_err += err;   
    }
    cout<<endl<<"max err point "<<max_index<<", real p "<<max_err_pp<<", while warp point "<<max_err_ip_pp<<endl;
    average_err = total_err/image_points_size;
    return total_err;
} 

void optimizeHomography(vector<vector<Point2f>>& vec_undistort_points, vector<vector<Point2f>>& vec_warp_points, vector<vector<int>>& vec_points_index, vector<Mat>& init_homograpy, vector<Mat>& Hs)
{
    //联合优化homography参数
    double H_transform[32];
    for(int i = 0; i < 4; ++i)
    {
        for(int j = 0; j < 8; ++j)
        {
            int ww = j%3, hh = j/3;
            H_transform[8*i+j] = init_homograpy[i].at<double>(hh, ww);
        }       
    }

    ceres::Problem problem;
    for(int i = 0; i <4; ++i)
        for(int j = 0; j < (int)vec_undistort_points[i].size(); ++j)
        {
            ceres::CostFunction* cost_function = GlobalCost::Create(vec_warp_points[i][j].x, vec_warp_points[i][j].y, vec_undistort_points[i][j].x, vec_undistort_points[i][j].y, i, false);
            problem.AddResidualBlock(cost_function, NULL, H_transform);
        }

    int simi_count = 0;
    for(int i = 0; i < 4; ++i)
        for(int j = 0; j < (int)vec_undistort_points[i].size(); ++j)
        {
            int next_index = (i+1)%4;
            for(int k = 0; k < (int)vec_undistort_points[next_index].size(); ++k)
            {
                if(vec_points_index[i][j] == vec_points_index[next_index][k])
                {
                    ceres::CostFunction* cost_function2 = GlobalCost::Create(vec_undistort_points[i][j].x, vec_undistort_points[i][j].y, vec_undistort_points[next_index][k].x, vec_undistort_points[next_index][k].y, vec_warp_points[i][j].x, vec_warp_points[i][j].y, i, true);
                    problem.AddResidualBlock(cost_function2, NULL, H_transform);  
                    simi_count++;               
                }
            }
        } 
    std::cout<<"total simi count: "<<simi_count<<std::endl;

    ceres::Solver::Options ceres_options;
    ceres_options.linear_solver_type = ceres::DENSE_QR;
    ceres_options.num_threads = 1;
    ceres_options.minimizer_progress_to_stdout = false;
    ceres_options.max_num_iterations = 100;
    ceres_options.function_tolerance = 0.01;
    ceres::Solver::Summary summary;
    ceres::Solve(ceres_options, &problem, &summary);

    Hs.resize(4);
    for(int i = 0; i < 4; ++i)
    {
        double max_err, average_err, total_err;
        total_err = computer_reproject_error(init_homograpy[i], vec_undistort_points[i], vec_warp_points[i], max_err, average_err);
        std::cout<<"image size: "<<vec_undistort_points[i].size()<<", computer_reproject_error before: "<<total_err<<", and max err "<<max_err<<", average err "<<average_err<<std::endl;
        Hs[i] = Mat::zeros(3,3,CV_64F);
        for(int j = 0; j < 8; ++j)
        {
            int ww = j%3, hh = j/3;
            Hs[i].at<double>(hh, ww) = H_transform[8*i+j];
        }
        Hs[i].at<double>(2, 2) = 1.0;
        total_err = computer_reproject_error(Hs[i], vec_undistort_points[i], vec_warp_points[i], max_err, average_err);
        std::cout<<"image size: "<<vec_undistort_points[i].size()<<", computer_reproject_error after: "<<total_err<<", and max err "<<max_err<<", average err "<<average_err<<std::endl;
    }
}

void calibrateExtrinsic(vector<Point3f>& world_points, vector<Point2f>& undistort_points, Mat& R, Mat& T)
{

    cv::solvePnP(world_points, undistort_points, cv::Mat::eye(3, 3, CV_64F), cv::noArray(), R, T);
    //Mat matR;
    //Rodrigues(R, matR);
    //
    //vector<Point2f> points;
    //int num = world_points.size();
    //for(int i=0; i<num; i++)
    //{
    //    Point3f wp = world_points[i];
    //    float scale = wp.x*matR.at<double>(2, 0) + wp.y*matR.at<double>(2, 1) + wp.z*matR.at<double>(2, 2)+T.at<double>(2,0);
    //    float x =(wp.x*matR.at<double>(0, 0) + wp.y*matR.at<double>(0, 1) + wp.z*matR.at<double>(0, 2)+T.at<double>(0,0))/scale;
    //    float y =(wp.x*matR.at<double>(1, 0) + wp.y*matR.at<double>(1, 1) + wp.z*matR.at<double>(1, 2)+T.at<double>(1,0))/scale;
    //    cout<<"world point is "<<wp<<", calc x, y is "<<x<<", "<<y<<" and orgin is "<<undistort_points[i]<<endl;
    //}
}

void Calibrate::Run()
{
    string debug_dir = calib_param_.image_points_dir_ + "calib/";
    if(debug_)
    {
        //MkDirs(debug_dir);
        for(int i=0; i<4; i++)
        {
            Mat plot = CFindPoints::PlotCorners(images_[i], vec_image_points_[i]);
            imwrite(debug_dir + "corner_plot_" + to_string(i) + ".jpg", plot);
        }
    }

    // calc init H from undistort plane to wolrd points
    vector<vector<Point2f>> vec_origin_points(4), vec_undistort_points(4), vec_warp_points(4);
    vector<vector<Point3f>> vec_world_points(4);
    vector<vector<int>> vec_points_index(4);
    vector<Mat> init_H(4), optimized_H;
    cout<<endl;
    for(int i=0; i<4; i++)
    {
        vec_points_index[i] = CFindPoints::ImagePoints2Points(vec_image_points_[i], vec_origin_points[i]);
        fisheye::undistortPoints(vec_origin_points[i], vec_undistort_points[i], fisheye_cams_[i].K_, fisheye_cams_[i].D_, Matx33d::eye(), Matx33d::eye());
        vec_warp_points[i] = wplane_.GetWorldPlanePoints(vec_points_index[i]);
        init_H[i] = findHomography(vec_undistort_points[i], vec_warp_points[i], LMEDS);
        cout<<"undistort point size "<<vec_undistort_points[i].size()<<", warp point size "<<vec_warp_points[i].size()<<", warp back point "<<vec_warp_points[i][0]<<endl;

        vec_world_points[i] = wplane_.GetWorldPoints(vec_points_index[i]);
        calibrateExtrinsic(vec_world_points[i], vec_undistort_points[i], stitch_param_.Rs_[i], stitch_param_.Ts_[i]);
    }

    if(debug_)
    {
        for(int i=0; i<4; i++)
        {
            Mat warp_map, calib_map, calib_mask, warp_dst, calib_dst;
            GenerateHWarpMapAndMask(warp_map, calib_mask, fisheye_cams_[i], init_H[i], output_size_);
            cout<<"origin point "<<vec_origin_points[i][0]<<", warp point "<<vec_warp_points[i][0]<<", map warp point "<<warp_map.at<Vec2f>(vec_warp_points[i][0].y, vec_warp_points[i][0].x)<<endl;
            remap(images_[i], warp_dst, warp_map, Mat(), CV_INTER_LINEAR);
            imwrite(debug_dir+"warp_mask_"+to_string(i)+".jpg", calib_mask);
            imwrite(debug_dir+"warp_"+to_string(i)+".jpg", warp_dst);
        }

        for(int i=0; i<4; i++)
        {
            Mat warp_map, calib_map, calib_mask, warp_dst, calib_dst;
            GenerateHWarpMapAndMask(warp_map, calib_mask, fisheye_cams_[i], stitch_param_.Rs_[i], stitch_param_.Ts_[i], stitch_param_.viewrange_, stitch_param_.pix_dis_);
            remap(images_[i], warp_dst, warp_map, Mat(), CV_INTER_LINEAR);
            imwrite(debug_dir+"extrinsic_mask_"+to_string(i)+".jpg", calib_mask);
            imwrite(debug_dir+"extrinsic_"+to_string(i)+".jpg", warp_dst);
        }
    }

    optimizeHomography(vec_undistort_points, vec_warp_points, vec_points_index, init_H, optimized_H);
    for(int i=0; i<4; i++)
        stitch_param_.Hs_[i] = optimized_H[i];

    vector<Mat> vec_masks(4);
    for(int i=0; i<4; i++)
    {
        Mat warp_map, calib_map , warp_dst, calib_dst;
        GenerateHWarpMapAndMask(warp_map, vec_masks[i], fisheye_cams_[i], optimized_H[i], output_size_);
        if(debug_)
        {
            remap(images_[i], warp_dst, warp_map, Mat(), CV_INTER_LINEAR);
            imwrite(debug_dir+"warp_optimized_"+to_string(i)+".jpg", warp_dst);
        }
    }

    CalcInnerRect(vec_masks, stitch_param_.inner_rect_);

    WriteStitchParam(calib_param_.stitch_file_, stitch_param_);
}

