#ifndef PARKING_MAP_STITCH_H
#define PARKING_MAP_STITCH_H

#include <string>
#include <vector>
#include <opencv2/core.hpp>

/*
 * init sitich, load config file to setup.
 * 1. config_dir, input directory contain config files of specific car  
 * output, 0 for sucess, other error
 */
int StitchInit(std::string config_dir);

/*
 * do stitch.
 * 1. input, four Mat images
 * output stitch image
 */
cv::Mat StitchProcess(std::vector<cv::Mat>& inputs);

/*
 * release stitch resource
 */
int StitchUinit();

#endif

