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
#include <glog/logging.h>
#include <gflags/gflags.h>
#include "HomographyCost.h"

//#include "ffmpeg_audio_video_decoder.h"
#include "Calibrate.h"
#include "Composition.h"
#include "Image2Ground.h"

using namespace std;
using namespace cv;
using namespace google;

DEFINE_string(input_dir, "", "dir contain camera00 camera01 result....");
DEFINE_int32(start_frame, 0, "start frame id");
DEFINE_int32(frame_num, 1, "default end frame id");
DEFINE_string(output_dir, "", "output dir");
DEFINE_string(config_dir, "", "config dir contain intrinsic and extrinsic yml");

pair<string, string> ProcessFileName(const string& file_name)
{
    auto index = file_name.find("_");
    string frame_id, time_stamp;
    if(index != string::npos)
    {
        time_stamp= file_name.substr(0, index);
        frame_id= file_name.substr(index+1, file_name.size()-index-7);
    }
    return make_pair(frame_id, time_stamp);
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

int loadImage(string input_dir, int frame_id, int frame_num, map<string, string>& mapImage, vector<vector<Mat>>& vecSrc)
{
    vecSrc.clear();
    for(int f=frame_id; f<frame_id+frame_num; f++)
    {
        string strFrameId=to_string(f);
        if(mapImage.find(strFrameId)==mapImage.end())
        {
            LOG(ERROR)<<"no frame id "<<f<<" in the parser map";
            return -1;
        }
        vector<Mat> vecTmp(4);
        for(int i=0; i<4; i++)
        {
            string strImageFile=input_dir+"camera0" + to_string(i) + "/" + mapImage[strFrameId] +"_" + strFrameId+ "_" + to_string(0) +".jpg";
            vecTmp[i]=imread(strImageFile);
            if(vecTmp[i].empty())
            {
                return -2;
                LOG(ERROR)<<"load image file error, "<<strImageFile;
            }
        }
        vecSrc.push_back(vecTmp);
    }

    return 0;
}

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc,&argv, true);
    google::InitGoogleLogging(argv[0]);

    string input_dir=FLAGS_input_dir;
    int start_frame=FLAGS_start_frame;
    int frame_num=FLAGS_frame_num;
    string output_dir=FLAGS_output_dir;
    string config_dir=FLAGS_config_dir;
    if(config_dir.empty())
        config_dir="/home/palfu/Data/AS552/config/";

    if(input_dir.empty())
        input_dir="/home/palfu/Data/AS552/extrinsic/2018-06-01-22-05-47_origin_image/";

    LOG(INFO)<<"input dir is "<<input_dir;
    LOG(INFO)<<"config dir is "<<config_dir;
    LOG(INFO)<<"output_dir is "<<output_dir;

    map<string, string> mapImage=ListDir(input_dir+"camera00/") ;
    if(frame_num==-1)
        frame_num=mapImage.size();

    LOG(INFO)<<"numbers of image is "<<mapImage.size()<<endl;

    Composition* com = new Composition(config_dir);
    
    vector<vector<Mat>> vecSrc;
    loadImage(input_dir, start_frame, frame_num, mapImage, vecSrc);
    cout<<"load frame num is "<<vecSrc.size()<<endl;
    int count=0;
    for(auto data:vecSrc)
    {
        Mat dst =  com->run(data);
        imwrite("dst_"+to_string(count++)+".jpg", dst);
    }

    cout<<"frame num is "<<frame_num<<endl;
    cout<<"average sv_remap time cost is "<<com->sv_remap_time/frame_num<<endl;
    cout<<"average index remap time cost is "<<com->index_remap_time/frame_num<<endl;
    google::ShutdownGoogleLogging();
    return 0;
}
