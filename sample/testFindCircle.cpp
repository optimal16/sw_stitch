#include<cstdio>
#include<opencv2/core.hpp>
#include<opencv2/opencv.hpp>
#include<iostream>
#include<fstream>
#include<string>
#include"FindCircle.h"

using namespace std;
using namespace cv;

void FindCircle::getPic(cv::Mat image,float width,float height,int circle_size, string path){
    int circle_size_ = circle_size*10; 
    int rect_size = 6 * circle_size_; 
    int h = height*image.cols;
    int w = width*image.rows;

    
    cout<< "width: "<<width<<"height: "<<height<<endl;
    ofstream worldpoint( path +"/worldpoint.txt");
    cv::Mat image1 = cv::Mat(cv::Size(rect_size,rect_size),CV_8U, cv::Scalar(255,255,255));
    
    cv::circle(image1,cv::Point2d(rect_size/4,rect_size/4),circle_size_,cv::Scalar(0,0,0),-1);
    cv::circle(image1,cv::Point2d(3*rect_size/4,rect_size/4),circle_size_,cv::Scalar(0,0,0),-1);
    cv::circle(image1,cv::Point2d(rect_size/4,3*rect_size/4),circle_size_,cv::Scalar(0,0,0),-1);
    cv::circle(image1,cv::Point2d(3*rect_size/4,3*rect_size/4),circle_size_,cv::Scalar(0,0,0),-1);

    cout<<"[0]:("<<rect_size/4+w <<" ,"<<rect_size/4 + h<<")"<<endl;
    cout<<"[1]:("<<3*rect_size/4+w <<" ,"<<rect_size/4 + h<<")"<<endl;
    cout<<"[2]:("<<rect_size/4+w <<" ,"<<3*rect_size/4 + h<<")"<<endl;
    cout<<"[3]:("<<3*rect_size/4+w <<" ,"<<3*rect_size/4 + h<<")"<<endl;
    cout<<"width = "<<width*20.0<<" height = "<<height*20.0<<endl;

    worldpoint<<"[0]:("<<rect_size/4+w <<" ,"<<rect_size/4 + h<<")\n";
    worldpoint<<"[1]:("<<3*rect_size/4+w <<" ,"<<rect_size/4 + h<<")\n";
    worldpoint<<"[2]:("<<rect_size/4+w <<" ,"<<3*rect_size/4 + h<<")\n";
    worldpoint<<"[3]:("<<3*rect_size/4+w <<" ,"<<3*rect_size/4 + h<<")\n";
    worldpoint<<"width = "<<width*20.0<<" height = "<<height*20.0<<"\n";
    worldpoint.close();
    
    cv::Rect rect = cv::Rect(width*image.rows,height*image.cols,rect_size,rect_size);
    image1.copyTo(image(rect));
    
    cv::imwrite("../config/single.png",image1);
    cv::imwrite(path+"/circle-w_"+ std::to_string((width*20.0)).substr(0,std::to_string((width*20.0)).size()-5)+
        "_h_"+ std::to_string((height*20.0)).substr(0,std::to_string(height*20).size()-5)+".png",image);
}

void FindCircle::getCircle(cv::Mat image_color,float width,float height,string path){
    cv::Mat image_gray = image_color.clone();
    cv::cvtColor(image_color,image_gray,COLOR_RGB2GRAY);
    // cv::imwrite("../debug/test/gray_test.png",image_gray);
    
    cv::SimpleBlobDetector::Params params;
    params.filterByInertia = false;
    params.filterByConvexity = false;
    params.filterByColor = false;
    params.filterByCircularity = false;
    params.filterByArea = true;
    params.minArea = 100;
    params.maxArea = 1500;
    // params.maxArea = 150000;
    std::vector<cv::Point2f> corners;
    bool found  = findCirclesGrid(image_gray,
                                  cv::Size(2,2),
                                  corners,
                                //   cv::CALIB_CB_SYMMETRIC_GRID + cv::CALIB_CB_CLUSTERING,
                                  cv::CALIB_CB_SYMMETRIC_GRID + cv::CALIB_CB_CLUSTERING,
                                  cv::SimpleBlobDetector::create(params));

    for(size_t i = 0; i < corners.size(); i++)
    {
        /* code */
        cv::circle(image_color,corners[i],1,cv::Scalar(0,0,255),-1);
        cv::putText(image_color, std::to_string(i), 
                            corners[i],cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
        cout<< corners[i] <<endl;
    }
    ofstream circlepoint( path +"/circlepoint.txt");
    circlepoint<<"[0]:("<<corners[0].x <<" ,"<<corners[0].y<<")\n";
    circlepoint<<"[1]:("<<corners[1].x <<" ,"<<corners[1].y<<")\n";
    circlepoint<<"[2]:("<<corners[2].x <<" ,"<<corners[2].y<<")\n";
    circlepoint<<"[3]:("<<corners[3].x <<" ,"<<corners[3].y<<")\n";
    circlepoint<<"width = "<<width*20.0<<" height = "<<height*20.0<<"\n";
    circlepoint.close();


    cv::drawChessboardCorners(image_color, cv::Size(1,1), corners, found); 
     
    cv::imwrite(path+"/color_test.png",image_color);
    std::cout<<"size:"<<corners.size()<<std::endl;
}

int main(int argc, char const *argv[])
{
    /* code */   


    FindCircle findcircle;
    cv::Mat image = cv::Mat(cv::Size(12000,12000),CV_8U, cv::Scalar(255,255,255));
    float width_str = atof(argv[1]);
    float height_str = atof(argv[2]);
    float width = width_str/20.00;
    float height = height_str/20.00;
    
    int circle_size = atoi(argv[3]);
    string pic_name = argv[4];

    string path = "../config/circle/"+std::to_string(width*20.0).substr(0,to_string(width*20.0).size()-5) 
        + "-" + std::to_string(height*20.0).substr(0,to_string(height*20.0).size()-5);
    std::cout << path << std::endl;
    string cmd_command = "mkdir " + path;
    //path
    system(cmd_command.c_str());

    string path_left = path + "/left";
    string path_front = path + "/front";
    string path_back = path + "/back";

//得到前和左
    #if 0
    cv::Mat image_front = cv::imread("../config/circle/"+pic_name+"-f.jpg",1);
    cv::Mat image_left = cv::imread("../config/circle"+pic_name+"-l.jpg",1);

    cmd_command = "mkdir " + path_left;
    system(cmd_command.c_str());
    cmd_command = "mkdir " + path_front;
    system(cmd_command.c_str());

    cv::imwrite(path_front+"/"+pic_name+"-f.jpg",image_front);
    cv::imwrite(path_left+"/"+pic_name+"-l.jpg",image_left);

    findcircle.getPic(image,width,height,circle_size,path);
    findcircle.getCircle(image_left,width*20,height*20,path_left);
    findcircle.getCircle(image_front,width*20,height*20,path_front);
//得到后和左
    #else
    cv::Mat image_back = cv::imread("../config/circle/"+pic_name+"-b.jpg",1);
    cv::Mat image_left = cv::imread("../config/circle/"+pic_name+"-l.jpg",1);

    cmd_command = "mkdir " + path_left;
    system(cmd_command.c_str());
    cmd_command = "mkdir " + path_back;
    system(cmd_command.c_str());

    cv::imwrite(path_back+"/"+pic_name+"-b.jpg",image_back);
    cv::imwrite(path_left+"/"+pic_name+"-l.jpg",image_left);

    findcircle.getPic(image,width,height,circle_size,path);
    findcircle.getCircle(image_back,width*20,height*20,path_back);
    findcircle.getCircle(image_left,width*20,height*20,path_left);

    #endif
    
    
    
    
    return 0;
}