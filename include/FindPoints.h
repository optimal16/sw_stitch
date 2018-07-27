#ifndef FIND_POINTS_H
#define FIND_POINTS_H

#include "IOConfig.h"
#include "CameraPos.h"

#define LEFT_POINTS_FILE "left.txt"
#define RIGHT_POINTS_FILE "right.txt"
#define FRONT_POINTS_FILE "front.txt"
#define BACK_POINTS_FILE "back.txt"

#define FRONT_IMAGE "front.jpg"
#define RIGHT_IMAGE "right.jpg"
#define BACK_IMAGE "back.jpg"
#define LEFT_IMAGE "left.jpg"

struct ImagePoints
{
    int index_;
    cv::Point2f pt_;
};

class CFindPoints
{
private:
    float left_right_to_front_distance_;
    float car_y_;
    float car_x_;
    cv::Mat src_;
    CameraPos pos_;
    FishEyeCam cam_;

public:
    CFindPoints(cv::Mat& src, FishEyeCam& cam, CameraPos pos, float left_right_to_front_distance=1.5f, float car_y=5.15f, float car_x=2.2f);//4.0f 2.2f
    int FindPoints(std::vector<cv::Point2f>& points);

    static int ReadImagePoints(std::string filename, std::vector<cv::Point2f>& points);
    static int WriteImagePoints(std::string filename, std::vector<cv::Point2f>& points);
    static void IndexFindPoints(std::vector<cv::Point2f>& points, std::vector<ImagePoints>& image_points, CameraPos pos);
    static cv::Mat PlotCorners(cv::Mat& src, std::vector<cv::Point2f>& points);
    static cv::Mat PlotCorners(cv::Mat& src, std::vector<ImagePoints>& image_points);
    static std::vector<int> ImagePoints2Points(std::vector<ImagePoints>& image_points, std::vector<cv::Point2f>& points);

private:

    int findInnerPoints(cv::Mat& src, FishEyeCam& cam, std::vector<cv::Point2f>& points, cv::Size chessboard_size = cv::Size(6, 4));

    int refineOuterPoints(cv::Mat& src, std::vector<cv::Point2f>& points, std::vector<cv::Point2f>& outer_points, FishEyeCam& cam, float left_right_to_front_distance, float car_y,float car_x, CameraPos pos);

    std::vector<cv::Point2f> mergeInnerOuterPoints(std::vector<cv::Point2f>& inner_points, std::vector<cv::Point2f>& outer_points);

    int selectBoundingChessboardCorners(std::vector<cv::Point2f>& corners, cv::Mat& src);
    
    cv::Point2f warpPoint(cv::Point2f p, cv::Mat& H);
};


#endif
