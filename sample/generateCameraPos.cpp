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
#include <fstream>

#include "SurroundView.h"
#include "HomographyCost.h"

#include "Calibrate.h"
#include "Composition.h"
#include "Image2Ground.h"
#include "ceres/ceres.h"

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

std::vector<std::string> ListDir(string src_dir) {
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

struct CameraPos
{
    Eigen::Vector3d pos;
    Eigen::Quaterniond rotation;
    double timestamp;
};

map<long long, CameraPos> readCameraPos(const std::string& camera_file)
{
    map<long long, CameraPos> res;
    ifstream file;
    file.open(camera_file, ios::in);
    if(!file.is_open())
    {
        std::cerr<<"can not open camera_file: "<<camera_file<<std::endl;
        return res;
    }
    
    double timestamp;
    double camera_data[7];
    std::string line_str;

    while(getline(file, line_str))//(!file.eof())
    {
        sscanf(line_str.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf\n", &timestamp, camera_data, camera_data+1, camera_data+2, camera_data+3,
                    camera_data+4, camera_data+5, camera_data+6 ); 
        timestamp *= 1000;
        CameraPos camera_pos;
        camera_pos.pos = Eigen::Vector3d(camera_data[0], camera_data[1], camera_data[2]);
        camera_pos.rotation = Eigen::Quaterniond(camera_data[3], camera_data[4], camera_data[5], camera_data[6]);
        camera_pos.timestamp = timestamp;
        long long tt = timestamp;
        res[tt] = camera_pos;
    }
    file.close();
    return res;
} 

int main(int argc, char **argv)
{
    #if 1
    string all_files_path = "/home/huam/data/diff/3/2018-02-11-21-45-30/img_front/";
    string image_files_path = "/home/huam/Downloads/result_index/";
    vector<string> all_files = ListDir(all_files_path);
    vector<string> image_files = ListDir(image_files_path);
    std::string new_image_path = "/home/huam/work/SurroundView/debug/segment_data/";
    map<int, string> id_to_timestamp;
    for(auto file : all_files)
    {
      int index = std::stoi(file.substr(0, 13));
      id_to_timestamp[index] = file.substr(14, 13);
    }

    for(size_t i = 0; i < image_files.size(); ++i)
    {
      string index_str = image_files[i].substr(0, 13);
      int index = std::stoi(index_str);
      string timestamp = all_files[index];
      cv::Mat image = cv::imread(image_files_path + image_files[i]);
      cv::resize(image, image, cv::Size(564, 1000));
      string save_path = new_image_path + id_to_timestamp[index] + image_files[i].substr(image_files[i].size()-4, 4);
      //cv::imwrite(save_path, image);
      std::cout<<index<<" "<<id_to_timestamp[index] + image_files[i].substr(image_files[i].size()-4, 4)<<std::endl;
    };
    #endif

    #if 0
    vector<string> image_files = ListDir(argv[1]);
    map<long long, CameraPos> res2 = readCameraPos(argv[2]);

    CalibrateOptions options;
    cv::Size output_size = options.output_size;
    output_size.width -= 36;

    double viewrange = options.viewrange;

    double f = options.output_size.width / options.viewrange;
    cv::Mat k = (cv::Mat_<double>(3, 3) << f, 0.0, output_size.width/2.0, 0.0, f, output_size.height/2.0, 0.0, 0.0, 1.0);
    std::cout<<k<<std::endl;
    #endif

    return 0;
}
