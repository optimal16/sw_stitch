#include<cstdio>
#include<opencv2/core.hpp>
#include<opencv2/opencv.hpp>
#include<iostream>
#include<string>

class FindCircle{
    
    public:
        // Test();
        void getPic(cv::Mat image,float width,float height,int circle_size, std::string path);
        void getCircle(cv::Mat image_color,float width,float height, std::string path);
        void run();
    private:
        cv::Mat image;
        int circle_size;
};