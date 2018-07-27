#ifndef WORLD_PLANE_H
#define WORLD_PLANE_H

#include <opencv2/core.hpp>
#include <string>

struct IndexPoint
{
    int index_;
    cv::Point2f wpt_;       // in mm;
    cv::Point pt_;     // in pix
};

struct WorldPlane
{

    std::vector<IndexPoint> InitPoints(float left_right_to_front_distance_, float car_y_,float car_x_, float viewrange_, float pix_dis_);
    
    std::vector<cv::Point2f> GetWorldPlanePoints(std::vector<int>& point_index);
    std::vector<cv::Point3f> GetWorldPoints(std::vector<int>& point_index);
    cv::Size GetOutputSize(float viewrange_, float pix_dis_);
    cv::Mat Plot(int scale=1);
    void Test(float lr2f=1.5f, float cy=5.5f, float cx=4.0f, float vr=12.0f, float pd=0.02f);

    static void InitPatternPoints(std::vector<cv::Point2f>& points);

private:


    void rotatePoints(std::vector<cv::Point2f>& points, std::vector<cv::Point2f>& rot_points, float angle, cv::Point2f st);

    std::vector<IndexPoint> index_points_;
    float left_right_to_front_distance_;
    float car_y_;
    float car_x_;
    float viewrange_;
    float pix_dis_;
};

#endif
