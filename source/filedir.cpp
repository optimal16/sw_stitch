#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <iostream>
#include <map>
#include <string.h>
#include "filedir.h"

using namespace std;

pair<int, string> ProcessFileName(const string& file_name)
{
    auto index = file_name.find("_");
    int frame_id;
    string time_stamp;
    if(index != string::npos)
    {
        time_stamp= file_name.substr(0, index);
        frame_id= atoi(file_name.substr(index+1, file_name.size()-index-7).c_str());
    }
    return make_pair(frame_id, time_stamp);
}

#if 1
map<int, string> MapListDir(string src_dir) {
  map<int, string> files;
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
    cout << "Open dir error...: " << src_dir <<endl;
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

vector<string> ListDir(string dir, string tag)
{
    
    map<int, string> map_list = MapListDir(dir);
    cout<<"list jpg file num "<<map_list.size();


    vector<string> res(map_list.size());
    int index=0;
    for(auto p:map_list)
    {
        res[index++] =  dir + p.second + "_" + to_string(p.first) + tag + ".jpg";
    }

    return res;
}


string GetBaseName(string filename)
{
    int index = filename.find_last_of("\\/");
    string basename = filename.substr(index+1);
    return basename;
}

void MkDirs(string dir)
{
    string cmd = string("mkdir -p ") + dir;
    int res = system(cmd.c_str());
    if(res==0)
        cout<<"create dir "<<dir<<endl;
    else
        cout<<"create dir "<<dir<<" failed!"<<endl;
}
