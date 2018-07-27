#include "FindPoints.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <fstream>
#include <iostream>
#include <assert.h>
#include "WorldPlane.h"

using namespace cv;
using namespace std;

CFindPoints::CFindPoints(Mat& src, FishEyeCam& cam, CameraPos pos, float left_right_to_front_distance, float car_y, float car_x)
{
    src_ = src;
    cam_ = cam;
    left_right_to_front_distance_ = left_right_to_front_distance;
    car_y_ = car_y;
    car_x_ = car_x;
    pos_ = pos;
}

int CFindPoints::FindPoints(vector<Point2f>& points)
{
    vector<Point2f> inner_points, outer_points;
    if(findInnerPoints(src_, cam_, inner_points)!=0)
    {
        cout<<"find inner points failed!"<<endl;
        return -1;
    }

    if(pos_==CameraPos::FRONT || pos_==CameraPos::BACK)
    {
        points = inner_points;
        cout<<"find point success!"<<endl;
        return 0;
    }

    if(refineOuterPoints(src_, inner_points, outer_points, cam_, left_right_to_front_distance_, car_y_, car_x_, pos_)!=0)
    {
        cout<<"find outer points failed!"<<endl;
        return -1;
    }

    points = mergeInnerOuterPoints(inner_points, outer_points);
    cout<<"find point success!"<<endl;
    return 0;
}


Mat CFindPoints::PlotCorners(Mat& src, vector<Point2f>& points)
{
    Mat plot = src.clone();
    for(auto p:points)
        circle(plot, Point(p.x, p.y), 2, Scalar(0, 0, 255), 1);
    return plot;
}

Mat CFindPoints::PlotCorners(Mat& src, vector<ImagePoints>& image_points)
{
    Mat plot = src.clone();
    for(auto ip:image_points)
        circle(plot, Point(ip.pt_.x, ip.pt_.y), 2, Scalar(0, 0, 255), 1);
    return plot;
}

vector<int> CFindPoints::ImagePoints2Points(vector<ImagePoints>& image_points, vector<Point2f>& points)
{
    vector<int> point_index;
    points.clear();
    for(auto ip:image_points)
    {
        point_index.push_back(ip.index_);
        points.push_back(ip.pt_);
    }
    return point_index;
}

static Point2f parseString2Point2f(string line)
{
    int pos = line.find_first_of(",");
    size_t lpos = line.find_first_of("[");
    if(lpos==string::npos)
        lpos = 0;
    size_t rpos = line.find_first_of("]");
    if(rpos==string::npos)
        rpos = line.size()-1;
    float x = atof(line.substr(lpos+1, pos).c_str());
    float y = atof(line.substr(pos+1, rpos-pos-1).c_str());
    return Point2f(x, y);
}

int CFindPoints::ReadImagePoints(std::string filename, vector<Point2f>& points)
{
    ifstream ifile;
    ifile.open(filename, ios::in);
    if(!ifile.is_open())
    {
        cout<<"open file failed! "<<filename<<endl;
        return -1;
    }

    string line;
    while(getline(ifile, line))
    {
        if(line.empty())
            continue;
        Point2f p = parseString2Point2f(line);
        points.push_back(p);
    }
    return 0;
}

int CFindPoints::WriteImagePoints(string filename, vector<Point2f>& points)
{
    ofstream ofile;
    ofile.open(filename, ios::out);
    if(!ofile.is_open())
    {
        cout<<"write file failed! "<<filename<<endl;
        return -1;
    }

    for(auto p:points)
        ofile<<p<<endl;
    ofile.close();
    return 0;
}

void CFindPoints::IndexFindPoints(vector<Point2f>& points, vector<ImagePoints>& image_points, CameraPos pos)
{
    int num = (int)points.size();
    assert(num==32 || num==40);
    image_points.resize(num);

    if(pos==CameraPos::FRONT)
    {
        for(int i=0; i<32; i++)
        {
            image_points[i] = ImagePoints{i, points[i]};
        }
    }

    if(pos==CameraPos::RIGHT)
    {
        int st = 32;
        for(int i=0; i<24; i++)
        {
            image_points[i] = ImagePoints{i+st, points[i]};
        }

        vector<int> index = {29, 31, 28, 30, 90, 88, 91, 89};
        for(int i=0; i<(int)index.size(); i++)
        {
            image_points[24+i] = ImagePoints{index[i], points[24+i]};
        }

        if(num==32)         // right only 32 point
            return;

        st = 56;
        for(int i=0; i<8; i++)
        {
            image_points[32+i] = ImagePoints{st+i, points[32+i]};
        }
    }        

    if(pos==CameraPos::BACK)
    {
        int st = 64;
        for(int i=0; i<32; i++)
        {
            image_points[i] = ImagePoints{st+i, points[i]};
        }
    }

    if(pos==CameraPos::LEFT)
    {
        int st = 96;
        for(int i=0; i<24; i++)
        {
            image_points[i] = ImagePoints{i+st, points[i]};
        }

        vector<int> index = {93, 95, 92, 94, 26, 24, 27, 25};
        for(int i=0; i<(int)index.size(); i++)
        {
            image_points[24+i] = ImagePoints{index[i], points[24+i]};
        }

        if(num==32)     // left only 32 point
            return;

        st = 120;
        for(int i=0; i<8; i++)
        {
            image_points[32+i] = ImagePoints{st+i, points[32+i]};
        }
    }

}

Point2f CFindPoints::warpPoint(Point2f p, Mat &H)
{
    float scale = H.at<double>(2, 0)*p.x + H.at<double>(2, 1)*p.y+H.at<double>(2,2);
    float x = (H.at<double>(0, 0)*p.x + H.at<double>(0, 1)*p.y + H.at<double>(0, 2))/scale;
    float y = (H.at<double>(1, 0)*p.x + H.at<double>(1, 1)*p.y + H.at<double>(1, 2))/scale;
    return Point2f(x, y);
}

int CFindPoints::findInnerPoints(Mat& src, FishEyeCam& cam, vector<Point2f>& points, Size chessboard_size)
{ 
    // find chessboard corners
    vector<Point2f> corners;
    int flags = CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FILTER_QUADS;
    bool found = findChessboardCorners(src, chessboard_size, corners, flags);

    vector<Point2f> bounding_points;
    if(found)
    {
        bounding_points = {corners[0], corners[5], corners[18], corners[23]};
    }else{
        if(selectBoundingChessboardCorners(bounding_points, src)!=0)
            return -1;
    }

    // undistort bounding corners
    vector<Point2f> undistort_bounding_points;
    Matx33d eye_matrix = Matx33d::eye();
    fisheye::undistortPoints(bounding_points, undistort_bounding_points, cam.K_, cam.D_, eye_matrix, eye_matrix);

    // get warp points
    vector<Point2f> world_points;
    WorldPlane::InitPatternPoints(world_points);
    int nums = world_points.size();
    for(int i=0; i<nums; i++)
    {
        world_points[i] = world_points[i]*100;
    }
    vector<Point2f> warp_points = {world_points[0], world_points[5], world_points[18], world_points[23]};
    
    // warp plane to undistort point
    Mat H = findHomography(warp_points, undistort_bounding_points);
    vector<Point2f> outer_warp_points = {world_points[25], world_points[28], world_points[27], world_points[30]};
    vector<Point2f> outer_warp_undistort_points, outer_origin_points;
    for(auto p:outer_warp_points)
        outer_warp_undistort_points.push_back(warpPoint(p, H));

    fisheye::distortPoints(outer_warp_undistort_points, outer_origin_points, cam.K_, cam.D_);
    cornerSubPix(src, outer_origin_points, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 50, 0.1));
    fisheye::undistortPoints(outer_origin_points, outer_warp_undistort_points, cam.K_, cam.D_, eye_matrix, eye_matrix);

    H = findHomography(outer_warp_points, outer_warp_undistort_points);
    
    vector<Point2f> all_undistort_points;
    for(auto p:world_points)
        all_undistort_points.push_back(warpPoint(p, H));

    fisheye::distortPoints(all_undistort_points, points, cam.K_, cam.D_);
    cornerSubPix(src, points, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 50, 0.1));

    return 0;
}

int CFindPoints::refineOuterPoints(Mat& src, vector<Point2f>& points, vector<Point2f>& outer_points, FishEyeCam& cam, float left_right_to_front_distance, float car_y,float car_x, CameraPos pos)
{
    if(!(pos==CameraPos::LEFT || pos==CameraPos::RIGHT))
    {
        cout<<"refine outer points error, pos not left or right"<<endl;
        return -1;
    }

    outer_points.resize(8);
    vector<Point2f> all_points(40);
    vector<ImagePoints> all_index_points;
    IndexFindPoints(all_points, all_index_points, pos);

    WorldPlane wplane;
    wplane.InitPoints(left_right_to_front_distance, car_y, car_x, 12.0f, 0.02f);

    vector<int> vec_inner_index, vec_outer_index, vector_all_index;
    for(int i=0; i<40; i++)
    {
        if(i<24 || i>31)
            vec_inner_index.push_back(all_index_points[i].index_);
        else
            vec_outer_index.push_back(all_index_points[i].index_);
        vector_all_index.push_back(all_index_points[i].index_);
    }

    vector<Point2f> vec_inner_plane_points = wplane.GetWorldPlanePoints(vec_inner_index);
    vector<Point2f> vec_outer_plane_points = wplane.GetWorldPlanePoints(vec_outer_index);

    vector<Point2f> vec_undisort_inner_points;
    Matx33d eye_matrix = Matx33d::eye();
    fisheye::undistortPoints(points, vec_undisort_inner_points, cam.K_, cam.D_, eye_matrix, eye_matrix);
    Mat H = findHomography(vec_inner_plane_points, vec_undisort_inner_points);
    
    vector<Point2f> vec_undisort_outer_points;
    for(auto p:vec_outer_plane_points)
        vec_undisort_outer_points.push_back(warpPoint(p, H));

    fisheye::distortPoints(vec_undisort_outer_points, outer_points, cam.K_, cam.D_);
    cornerSubPix(src, outer_points, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 50, 0.1));
    return 0;
}

vector<Point2f> CFindPoints::mergeInnerOuterPoints(vector<Point2f>& inner_points, vector<Point2f>& outer_points)
{
    int inner_size = inner_points.size();
    int outer_size = outer_points.size();
    assert(inner_size==32 && outer_size==8);

    vector<Point2f> vec_res;
    vec_res.insert(vec_res.end(), inner_points.begin(), inner_points.begin()+24);
    vec_res.insert(vec_res.end(), outer_points.begin(), outer_points.end());
    vec_res.insert(vec_res.end(), inner_points.begin()+24, inner_points.end());
    return vec_res;
}



/*=======================================================*/

#define DEFAULT_POINT  (Point(-1, -1))
typedef struct SelectParam
{
    SelectParam(string wnd_name, Mat& src){
        this->wnd_name = wnd_name;
        //this->src = src;
        resize(src, this->src, src.size()/2);
        wnd_size = src.size()/2;
        roi = Rect(0, 0, wnd_size.width, wnd_size.height);
        tag=0;
        for (int i=0; i<4; i++)
            points[i] = DEFAULT_POINT;
    }

    Mat src;
    Point points[4];
    int tag;
    string wnd_name;
    Rect roi;
    Size wnd_size;
}SParam;

static int slamp(int x, int minv, int maxv)
{
    int res = x > minv ? x : minv;
    res = res > maxv ? maxv : res;
    return res;
}

static void drawLine(Mat& src, Point p1, Point p2, Scalar color, int thickness=2)
{
    if(p1==DEFAULT_POINT || p2==DEFAULT_POINT)
        return;
    line(src, p1, p2, color, thickness);
}

void drawPoint(Mat& src, Point pt, int thickness){
    if (pt == DEFAULT_POINT || pt.x < 0 || pt.y < 0 || pt.x>=src.cols || pt.y >=src.rows)
        return;
    circle(src, pt, thickness, Scalar(0, 0, 255), thickness);
}

static Mat getROIImage(SParam& param){
    if (param.src.cols == 0)
        return Mat::zeros(100, 100, CV_8UC3);
    Mat tmp = param.src(param.roi).clone();
    int thickness = 2;
    if (tmp.cols > 500)
        thickness = tmp.cols / 400 * 2;
    for (Point pt : param.points)
        drawPoint(tmp, pt - param.roi.tl(), thickness);

    return tmp;
}

static Mat showROIPoint(SParam* ptr, Point p=DEFAULT_POINT)
{
    if(p!=DEFAULT_POINT)
        ptr->points[ptr->tag]=ptr->roi.tl()+p;
    return getROIImage(*ptr);
}

static Mat showBoundingArea(SParam& param)
{
    Mat plot = param.src.clone();
    Scalar color(0, 0, 255);
    drawLine(plot, param.points[0], param.points[1], color);
    drawLine(plot, param.points[1], param.points[2], color);
    drawLine(plot, param.points[2], param.points[3], color);
    drawLine(plot, param.points[0], param.points[3], color);
    return plot;
}

static void resizeRect(Size orisize, Rect& roi, Point midp, int bin)
{
    Point tl = roi.tl();
    float ratio = 1.5f;
    if (bin > 0)
        ratio = 1 / 1.5f;
    Point new_tl = midp + tl - midp*ratio;
    roi.width = slamp(int(roi.width*ratio), 2, orisize.width);
    roi.height = slamp(int(roi.height*ratio), 2, orisize.height);
    roi.x = slamp(new_tl.x, 0, orisize.width - roi.width);
    roi.y = slamp(new_tl.y, 0, orisize.height - roi.height);
}

static void on_mouse(int event, int x, int y, int flag, void* param)
{
    SParam* ptr = (SParam*)param;
    if (event == CV_EVENT_MOUSEWHEEL)		//
    {
        int delta = flag >> 16;

        resizeRect(ptr->wnd_size, ptr->roi, Point(x, y), delta);
        Mat tmp = showROIPoint(ptr);
        imshow(ptr->wnd_name, tmp);
    }
    if (event == CV_EVENT_LBUTTONDOWN)	//
    {
        Mat tmp = showROIPoint(ptr, Point(x,y));
        imshow(ptr->wnd_name, tmp);
    }
    if (event == CV_EVENT_MOUSEMOVE && (flag & CV_EVENT_LBUTTONDOWN))	//
    {
        Mat tmp = showROIPoint(ptr, Point(x, y));
        imshow(ptr->wnd_name, tmp);
    }
}

static int checkPoints(SParam& param)
{
    for(auto p:param.points)
    {
        if(p==DEFAULT_POINT)
            return false;
    }
    return true;
}

int CFindPoints::selectBoundingChessboardCorners(vector<Point2f>& corners, Mat& src)
{
    string wnd_name = "Select Bouding Chessboard Corners";
    namedWindow(wnd_name, WINDOW_NORMAL);
    SParam param(wnd_name, src);
    imshow(wnd_name, showBoundingArea(param));
    resizeWindow(wnd_name, src.size()/2);
    setMouseCallback(wnd_name, on_mouse, (void*)&param);

    cout<<"start select bounding points!"<<endl;
    while(true)
    {
        int key = waitKey(0);
        switch (key) {
        case '1':
            param.tag=0;
            setWindowTitle(wnd_name, "select the top-left point!");
            break;
        case '2':
            param.tag=1;
            setWindowTitle(wnd_name, "select the top-right point!");
            break;
        case '3':
            param.tag=2;
            setWindowTitle(wnd_name, "select the bot-left point!");
            break;
        case '4':
            param.tag=3;
            setWindowTitle(wnd_name, "select the bot_right point!");
            break;
        default:
            break;
        }

        if (key == 27 || key=='q')
        {
            if(checkPoints(param))
                destroyWindow(wnd_name);
            else
                setWindowTitle(wnd_name, "not all points selected!");
            break;
        }

    }

    if(checkPoints(param))
    {
        cout<<"select bounding points success!"<<endl;
        corners.resize(4);
        for(int i=0; i<4; i++)
        {
            Point p = param.points[i]*2;
            corners[i] = Point2f(p.x, p.y);
        }
        cornerSubPix(src, corners, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 50, 0.1));

        // TODO debug plot
        {
            Mat select_plot = src.clone();
            for(auto p:corners)
                circle(select_plot, Point(p.x, p.y), 2, Scalar(0, 0, 255), 2);
            imwrite("select_plot.jpg",select_plot);
        }

        return 0;
    }

    cout<<"select bounding points failed!"<<endl;
    return -1;

}

