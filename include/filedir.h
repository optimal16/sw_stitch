#ifndef FILEDIR_H
#define FILEDIR_H

#include <vector>
#include <string>

std::vector<std::string> ListDir(std::string dir, std::string tag=std::string("_0"));

std::string GetBaseName(std::string filename);

void MkDirs(std::string dir);

#endif
