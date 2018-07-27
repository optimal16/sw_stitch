#include "opencv2/opencv.hpp"
#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <string>
#include <time.h>

#include "HomographyCost.h"

//#include "ffmpeg_audio_video_decoder.h"
#include "Calibrate.h"
#include "Composition.h"
#include "Image2Ground.h"

using namespace std;
using namespace cv;

pair<string, string> ProcessFileName(const string& file_name)
{
    auto index = file_name.find("_");
    string first, second;
    if(index != string::npos)
    {
        first = file_name.substr(0, index);
        second = file_name.substr(index+1, file_name.size()-index-5);
    }
    return make_pair(first, second);
}

std::vector<std::string> ListDir2(string src_dir) {
 std::vector<std::string> files;
 #ifdef WIN32
  _finddata_t file;
  long lf;
  if ((lf = _findfirst(src_dir.c_str(), &file)) == -1) {
    cout << src_dir << " not found!!!" << endl;
  } else {
    while (_findnext(lf, &file) == 0) {
      if (strcmp(file.name, ".") == 0 || strcmp(file.name, "..") == 0) continue;
      auto res = ProcessFileName(file.name);
      files[res.first] = res.second;
    }
  }
  _findclose(lf);
 #endif

 #ifdef __linux__
  DIR *dir;
  struct dirent *ptr;

  if ((dir = opendir(src_dir.c_str())) == NULL) {
    LOG(ERROR) << "Open dir error...: " << src_dir;
    exit(1);
  }
  while ((ptr = readdir(dir)) != NULL) {
    if (strcmp(ptr->d_name, ".") == 0 ||
        strcmp(ptr->d_name, "..") == 0) {  // cur  or parent
      continue;
    } else if (ptr->d_type == 8) {  // file
      //auto res = ProcessFileName(ptr->d_name);
      //files[res.first] = res.second;
      files.push_back(ptr->d_name);
    } else if (ptr->d_type == 10) {  // link file
      continue;
    } else if (ptr->d_type == 4) {  // dir
      // files.push_back(ptr->d_name);
      continue;
    }
  }
  closedir(dir);
 #endif
  sort(files.begin(), files.end());
  return files;
}


#if 1
map<string, string> ListDir(string src_dir) {
  map<string, string> files;
 #ifdef WIN32
  _finddata_t file;
  long lf;
  if ((lf = _findfirst(src_dir.c_str(), &file)) == -1) {
    cout << src_dir << " not found!!!" << endl;
  } else {
    while (_findnext(lf, &file) == 0) {
      if (strcmp(file.name, ".") == 0 || strcmp(file.name, "..") == 0) continue;
      auto res = ProcessFileName(file.name);
      files[res.first] = res.second;
    }
  }
  _findclose(lf);
 #endif

 #ifdef __linux__
  DIR *dir;
  struct dirent *ptr;

  if ((dir = opendir(src_dir.c_str())) == NULL) {
    LOG(ERROR) << "Open dir error...: " << src_dir;
    exit(1);
  }
  while ((ptr = readdir(dir)) != NULL) {
    //std::cout<<"name: "<<ptr->d_name<<", type: "<<ptr->d_type<<std::endl;
    if (strcmp(ptr->d_name, ".") == 0 ||
        strcmp(ptr->d_name, "..") == 0) {  // cur  or parent
      continue;
    } else if (ptr->d_type == 10) {  // link file
      continue;
    } else if (ptr->d_type == 4) {  // dir
      // files.push_back(ptr->d_name);
      continue;
    } else //if (ptr->d_type == 8) 
    {  // file
      auto res = ProcessFileName(ptr->d_name);
      files[res.first] = res.second;
    }
  }
  closedir(dir);
 #endif
  //sort(files.begin(), files.end());
  return files;
}
#endif

int main(int argc, char **argv)
{
    cv::Mat front = cv::imread("/home/huam/data/svs_data/4-11/good2/camera01/1504658781069_101_1.jpg");
    cv::Mat right = cv::imread("/home/huam/data/svs_data/4-11/good2/camera02/1504658654983_86_2.jpg");
    cv::Mat back = cv::imread("/home/huam/data/svs_data/4-11/good2/camera03/1504658713149_93_3.jpg");
    cv::Mat left = cv::imread("/home/huam/data/svs_data/4-11/good2/camera00/1504657936051_0_0.jpg");

    std::vector<cv::Mat> inputs;
    inputs.push_back(front); inputs.push_back(right); inputs.push_back(back); inputs.push_back(left);
    std::vector<Mat> outputs(inputs.size());

    #if 1
    std::string config_dir="../config/";
    std::string extrinsic_yml = config_dir + "extrinsic.yml";
    std::string intrinsic_yml = config_dir + "intrinsic.yml";
    CalibrateOptions option(intrinsic_yml);
    CalibratePtr cal_ptr(new Calibrate(option));
    cal_ptr->run(inputs);
    cal_ptr->outputParams(extrinsic_yml);

    Composition* com = new Composition(config_dir);
    Rect rects_[4];
    cv::Mat com_res = com->run(inputs);
    cv::imwrite("../debug/res_poisson.png", com_res); 
    //delete cal_ptr;
    //delete com;

    cv::Point2f points[] = {cv::Point2f(0, 0), cv::Point2f(0, 282), cv::Point2f(0, 563),
                            cv::Point2f(563,0), cv::Point2f(563, 282), cv::Point2f(563, 563),
                            cv::Point2f(282, 0), cv::Point2f(282, 563)};
//    for(int i = 0; i < sizeof(points)/sizeof(points[0]); ++i)
//    {
//        cv::Point2f p_res;
//        com->image2ground(points[i], p_res);
//        std::cout<<points[i]<<" "<<p_res<<std::endl;
//
//        com->camera2ground(points[i], CAMERA_POS(0), p_res);
//        //std::cout<<points[i]<<" "<<p_res<<std::endl;
//        // double diff = 18.0;
//        // int x = int(points[i].x + 0.5 + diff), y = int(points[i].y + 0.5 + diff);
//        // int camera_index = com->composition_mask_.at<uchar>(y, x);
//
//        // float px = com->sv_to_image_.at<Vec2f>(y, x)[0], py = com->sv_to_image_.at<Vec2f>(y, x)[1];
//        // com->camera2ground(cv::Point2f(px, py), CAMERA_POS(camera_index), p_res);
//        // std::cout<<"res2: "<<cv::Point2f(px, py)<<" "<<p_res<<std::endl;        
//    }
//
    #endif

    #if 1
    std::string image_file_path = "/home/huam/data/svs_data/mjh/2018-04-11-17-50-45/";
    std::vector<string> image_names[4];
    //std::string camera_to_pos[4] = {"camera03", "camera00", "camera02", "camera01"};
    std::string camera_to_pos[4] = {"camera01", "camera02", "camera03", "camera00"};
    //std::string camera_to_pos[4] = {"camera00", "camera01", "camera02", "camera03"};    
    int min_image_num = INT_MAX;

    time_t cur_time;
    time(&cur_time);
    std::stringstream ss;
    ss << cur_time;
    string save_file_path = "../result/";//"../debug/"+ss.str()+"/";
    //mkdir(save_file_path.c_str(), 0777);

    std::vector<long> timestamps[4];

    for(int i = 0; i < 4; ++i)
    {
        image_names[i] = ListDir2(image_file_path + camera_to_pos[i]);
        min_image_num = std::min(min_image_num, (int)image_names[i].size());
        timestamps[i].resize(image_names[i].size());
        for(size_t index = 0; index < image_names[i].size(); ++index)
        {
            timestamps[i][index] = std::stoll(image_names[i][index].substr(0, 13));
        }
        sort(timestamps[i].begin(), timestamps[i].end());
    }  
    int image_start_index = 0;
    int image_total_num = 100000;
    if(argc >= 2) image_start_index = atoi(argv[1]);
    if(argc >= 3) image_total_num = atoi(argv[2]);

    std::cout<<"min: "<<min_image_num<<", start: "<<image_start_index<<", total: "<<image_total_num<<std::endl;
    for(size_t i = image_start_index; i < min_image_num && i < image_start_index + image_total_num; ++i)
    {
        //if(i % 15 != 0) continue;
        long timestamp = timestamps[0][i];
        //std::cout<<"timestamp: "<<timestamp<<std::endl;
        bool flag = true;
        std::vector<long> times;
        times.push_back(timestamp);
        int threshold = 5;
        for(int t = 1; t < 4; ++t)
        {
            int index = std::lower_bound(timestamps[t].begin(), timestamps[t].end(), timestamp)-timestamps[t].begin();
            if(index >= 0 && abs(timestamps[t][index] - timestamp) < threshold)
            {
                times.push_back(timestamps[t][index]);
                continue;
            }
            if(index < timestamps[t].size()-1 && abs(timestamps[t][index+1]-timestamp)<threshold)
            {
                times.push_back(timestamps[t][index+1]);
                continue;
            }
        }
        if(times.size() == 4)
        {
            inputs.clear();
            std::string name = "";//std::to_string(times[0]);
            bool flag = true;
            for(int t = 0; t < 4; ++t)
            {
                string name11 = image_file_path+camera_to_pos[t]+"/"+std::to_string(times[t])
                            +"_"+std::to_string(i)
                            +"_"+camera_to_pos[t][camera_to_pos[t].size()-1]
                            +".jpg";
                cv::Mat image = cv::imread(name11);
                if(image.empty())
                {
                  std::cout<<"name: "<<name11<<std::endl;
                  flag = false;
                  continue;
                }
                inputs.push_back(image);
                name = name + std::to_string(times[t]) + "_";
            }
            if(flag)
            {
              cv::Mat com_res = com->run(inputs);
              //std::cout<<"name: "<<name<<std::endl;
              cv::imwrite(save_file_path+name+".png", com_res);
            }
        } 
        else std::cout<<"name: "<<times[0]<<std::endl;
        
    }    
    #endif

    return 0;
}
