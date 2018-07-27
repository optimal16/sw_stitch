#include "opencv2/opencv.hpp"
#include <fstream>
//#include <direct.h>
//#include <io.h>  
//#include <windows.h>
#include <iostream>
#include <vector>
#include <algorithm>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string>

#include "FindCorners.h"
#include "SurroundView.h"

#include "opencv2/ccalib.hpp"
#include "opencv2/ccalib/omnidir.hpp"

using namespace std;
using namespace cv;

bool CheckPath(const std::string& filename)
{
	int pos = -1;
	while (true)
	{
		int temp_pos1 = filename.find("\\", pos + 1);
		int temp_pos2 = filename.find("/", pos + 1);
		if (temp_pos1 > 0 && temp_pos2 > 0)
			pos = temp_pos1 < temp_pos2 ? temp_pos1 : temp_pos2;
		else if (temp_pos1 > 0)
			pos = temp_pos1;
		else if (temp_pos2 > 0)
			pos = temp_pos2;
		else
			break;
		std::string path = filename.substr(0, pos);
		if (access(path.c_str(), 0) != -1)
			continue;
		if (mkdir(path.c_str(), 0777) == 0)
			continue;
		else
			return false;
	}
	return true;
}

int main(int argc, char** argv)
{
#if 0
	int image_count = 0;
	printf("Input image num: ");
	scanf("%d", &image_count);
	std::string input_img_path(argv[1]);
	std::string output_dir("../output/calibration_result.txt");
	std::string filestorage_dir("../config/gmsl_right_2.yml");
	CheckPath(output_dir);
	ofstream fout(output_dir.c_str()); 
    std::string circle_path = input_img_path + "/circle/";
    std::string undistort_path = input_img_path + "/undistort/";
    mkdir(circle_path.c_str(), 0777);
    mkdir(undistort_path.c_str(), 0777);

    printf("start detect corners...\n"); 
 
    Size board_size = Size(9,6);             
    vector<Point2f> corners;                 
    vector<vector<Point2f> >  corners_Seq;    
    vector<Mat>  image_Seq;
	int successImageNum = 0;				
	vector<bool> mask;
    int count = 0;
    std::vector<int> can_indexs;
    for( int i = 0;  i != image_count ; i++)
    {
		printf("Frame # %d...\n", i);
		char filename[128] = { 0 };
		sprintf(filename, "%d.jpeg", i);
        if (access((input_img_path+filename).c_str(),0) == -1) continue;
        cv::Mat image = imread(input_img_path + filename);
        cout<<i<<", image name: "<<input_img_path+filename<<std::endl;
        /* start detect corners */   
        Mat imageGray;
        cvtColor(image, imageGray , CV_RGB2GRAY);
        bool patternfound = findChessboardCorners(image, board_size, corners,
        CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE+CALIB_CB_FAST_CHECK );
        if (!patternfound)   
        {   
			mask.push_back(false);
            cout<<"can not find chessboard corners!\n"; 
            remove((input_img_path+filename).c_str());
            continue;
            exit(1);   
        } 
        else
        {   
            can_indexs.push_back(i);
			mask.push_back(true);
            // improve the found corners' coordinate accuracy for chessboard
            cornerSubPix(imageGray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            /* draw corners */
            Mat imageTemp = image.clone();
            for (int j = 0; j < corners.size(); j++)
            {
                circle( imageTemp, corners[j], 10, Scalar(0,0,255), 2, 8, 0);
            }
            count = count + corners.size();
			successImageNum = successImageNum + 1;
            corners_Seq.push_back(corners);

			char output_filename[512] = { 0 };
            std::string sss = circle_path + "%d.png";
			sprintf(output_filename, sss.c_str(), i);
			CheckPath(std::string(output_filename));
			imwrite(output_filename, imageTemp);
        }  
        image_Seq.push_back(image);
    }   
    cout<<"corners detection done!\n"; 
    for(auto index:can_indexs)
        cout<<index<<" ";
    cout<<std::endl;
    /************************************************************************  
           camera calibration 
    *************************************************************************/   
    cout<<"start calibration"<<endl;  
	Size square_size = Size(20,20);     
	vector<vector<Point3f> >  object_Points;    
    Mat image_points = Mat(1, count, CV_32FC2, Scalar::all(0));  
    vector<int>  point_counts;                                                         

	
	for (int t = 0; t<successImageNum; t++)
    {
        vector<Point3f> tempPointSet;
        for (int i = 0; i<board_size.height; i++)
        {
            for (int j = 0; j<board_size.width; j++)
            {
                Point3f tempPoint;
                tempPoint.x = j*square_size.width;
                tempPoint.y = i*square_size.height;
                tempPoint.z = 0;
                tempPointSet.push_back(tempPoint);
            }
        }
        object_Points.push_back(tempPointSet);
    }
	for (int i = 0; i< successImageNum; i++)
    {
        point_counts.push_back(board_size.width*board_size.height);
    }

    std::vector<cv::Vec3d> rotation_vectors;                           
    std::vector<cv::Vec3d> translation_vectors;  
    printf("omni calibration...\n");
    #if 0
    cv::Mat K, xi, D, idx;
    int omni_flags = 0;
    cv::TermCriteria critia(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 0.0001);
    cv::omnidir::calibrate(object_Points, corners_Seq, image_Seq[0].size(),
                         K, xi, D, rotation_vectors, translation_vectors,
                         omni_flags, critia, idx);

    cv::FileStorage omni_cameraParamFile("../config/mei_front.yml", cv::FileStorage::WRITE);
    if (!omni_cameraParamFile.isOpened()) {
        std::cout<< "Failed to open save file: " << filestorage_dir << std::endl;
        return -1;
    }
    omni_cameraParamFile << "image_width" << image_Seq[0].cols;
    omni_cameraParamFile << "image_height" << image_Seq[0].rows;
    omni_cameraParamFile << "camera_matrix" << K;
    omni_cameraParamFile << "xi" << xi;
    omni_cameraParamFile << "D" <<D;
    omni_cameraParamFile << "idx" <<idx;
    std::cout << "Successfully saved camera parameter file: " << filestorage_dir << std::endl;
    omni_cameraParamFile.release();

    //save undistortion image
    for(int i = 0; i < image_count; ++i){
		char filename[128] = { 0 };
		sprintf(filename, "%d.png", i);
        if (access((input_img_path+filename).c_str(),0) == -1) continue;
        cv::Mat image = imread(input_img_path + filename);
        cv::Mat output;
        cv::Size new_size(image.cols, image.rows);
        cv::Matx33f knew = cv::Matx33f(new_size.width/6, 0, 0,
               0, new_size.height/6, 0,
               0, 0, 1);
        cv::omnidir::undistortImage(image, output, K, D, xi, cv::omnidir::RECTIFY_CYLINDRICAL, knew, new_size);
        cv::imwrite(undistort_path+"undistort_"+std::to_string(i)+".png", output);
    }  
    for(int i = 0; i < 4; ++i)
    {
        cv::Mat image = imread("/home/huam/data/360_all_image/camera_0"+std::to_string(i)+"_00060.png");
        cv::Mat output;
        cv::Size new_size(image.cols, image.rows);
        cv::Matx33f knew = cv::Matx33f(new_size.width/6, 0, 0,
               0, new_size.height/6, 0,
               0, 0, 1);
        cv::omnidir::undistortImage(image, output, K, D, xi, cv::omnidir::RECTIFY_CYLINDRICAL, knew, new_size);
        cv::imwrite(undistort_path+std::to_string(i)+".png", output);
    }  
    return 0;
    #endif

	printf("calibration...\n");
    Size image_size = image_Seq[0].size();
    //cv::Matx33d intrinsic_matrix;
    cv::Mat_<double>intrinsic_matrix(3,3);
    cv::Vec4d distortion_coeffs;     
                 
    int flags = 0;
    flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
    flags |= cv::fisheye::CALIB_CHECK_COND;
    flags |= cv::fisheye::CALIB_FIX_SKEW;
	// flags |= cv::fisheye::CALIB_FIX_K3;
	// flags |= cv::fisheye::CALIB_FIX_K4;
    fisheye::calibrate(object_Points, corners_Seq, image_size, intrinsic_matrix, 
		distortion_coeffs, rotation_vectors, translation_vectors, flags, cv::TermCriteria(3, 20, 1e-6));
    cout<<"calibration done!\n";   

#if 0
    /************************************************************************  
           �Զ�������������  
    *************************************************************************/   
    cout<<"��ʼ���۶�����������������"<<endl;   
    double total_err = 0.0;                   /* ����ͼ���ƽ�������ܺ� */   
    double err = 0.0;                        /* ÿ��ͼ���ƽ����� */   
    vector<Point2f>  image_points2;             /****   �������¼���õ���ͶӰ��    ****/   
 
    for (int i=0;  i<image_count;  i++) 
    {
		printf("���� Image: %d...\n", i+1);
		if (!mask[i])
			continue;
        vector<Point3f> tempPointSet = object_Points[i];
        /****    ͨ���õ������������������Կռ����ά���������ͶӰ���㣬�õ��µ�ͶӰ��     ****/
		fisheye::projectPoints(tempPointSet, image_points2, rotation_vectors[i], translation_vectors[i], intrinsic_matrix, distortion_coeffs);
        /* �����µ�ͶӰ��;ɵ�ͶӰ��֮������*/  
        vector<Point2f> tempImagePoint = corners_Seq[i];
        Mat tempImagePointMat = Mat(1,tempImagePoint.size(),CV_32FC2);
        Mat image_points2Mat = Mat(1,image_points2.size(), CV_32FC2);
        for (size_t i = 0 ; i != tempImagePoint.size(); i++)
        {
            image_points2Mat.at<Vec2f>(0,i) = Vec2f(image_points2[i].x, image_points2[i].y);
            tempImagePointMat.at<Vec2f>(0,i) = Vec2f(tempImagePoint[i].x, tempImagePoint[i].y);
        }
        err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
        total_err += err/=  point_counts[i];   
    }    
    cout<<"������ɣ�"<<endl;   
#endif

    /************************************************************************  
           ���涨����  
    *************************************************************************/   
    cout<<"save calibration results..."<<endl;        

    fout<<"intrinsic_matrix: "<<endl;   
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
			fout << intrinsic_matrix(i, j) << "\t";
		fout << endl;
	}
	fout << endl;

	fout << "distortion_coeffs: " << endl;
	for (int i = 0; i < 4; ++i)
		fout << distortion_coeffs[i] << "\t";
	fout << endl;
    cout<<"save calibration results done!"<<endl; 


    cv::FileStorage cameraParamFile(filestorage_dir, cv::FileStorage::WRITE);
    if (!cameraParamFile.isOpened()) {
        std::cout<< "Failed to open save file: " << filestorage_dir << std::endl;
        return -1;
    }
    cameraParamFile << "image_width" << image_Seq[0].cols;
    cameraParamFile << "image_height" << image_Seq[0].rows;
    cameraParamFile << "camera_matrix" << intrinsic_matrix;
    cv::Mat_<double>distortion_matrix(1, 4);
    for(int i = 0; i < 4; ++i) distortion_matrix(0, i) = distortion_coeffs[i];
    cameraParamFile << "distortion_coefficients" << distortion_matrix;
    std::cout << "Successfully saved camera parameter file: " << filestorage_dir << std::endl;
    cameraParamFile.release();

    //save undistortion image
    for(int i = 0; i < image_count; ++i){
		char filename[128] = { 0 };
		sprintf(filename, "%d.png", i);
        if (access((input_img_path+filename).c_str(),0) == -1) continue;
        cv::Mat image = imread(input_img_path + filename);
        cv::Mat output;
        cv::fisheye::undistortImage(image, output, intrinsic_matrix, 
        distortion_coeffs, intrinsic_matrix);
        cv::imwrite(undistort_path+"undistort_"+std::to_string(i)+".png", output);
    }
    /************************************************************************  
           ��ʾ������  
    *************************************************************************/
#else
    cv::Mat image = imread("..\\test\\0_Y.jpg");
	Size image_size = image.size();
	vector<Mat>  image_Seq;
	image_Seq.push_back(image);
	int image_count = image_Seq.size();
	cv::Matx33d intrinsic_matrix; 
	cv::Vec4d distortion_coeffs;
	intrinsic_matrix(0, 0) = 1189.97;
	intrinsic_matrix(0, 1) = 0;
	intrinsic_matrix(0, 2) = 1353.85;
	intrinsic_matrix(1, 0) = 0;
	intrinsic_matrix(1, 1) = 1192.32;
	intrinsic_matrix(1, 2) = 1043.8;
	intrinsic_matrix(2, 0) = 0;
	intrinsic_matrix(2, 1) = 0;
	intrinsic_matrix(2, 2) = 1;

	distortion_coeffs[0] = 0.058165;
	distortion_coeffs[1] = 0.00227125;
	distortion_coeffs[2] = 0;
	distortion_coeffs[3] = 0;

#endif
  
    Mat map1 = Mat(image_size,CV_32FC1);
    Mat map2 = Mat(image_size,CV_32FC1);
    Mat R = Mat::eye(3,3,CV_32F);
    cout<<"�������ͼ��"<<endl;
	fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, 
		intrinsic_matrix, image_size, CV_32FC1, map1, map2);
  
	printf("image_size: %d * %d\n", image_size.height, image_size.width);

/*
#if 0
	float max1 = INT_MAX, max2 = INT_MAX, min1 = -INT_MAX, min2 = -INT_MAX;
	for (int i = 0; i<image_size.height; ++i)
	{
		if (min1 < map1.at<float>(i, 0))
			min1 = map1.at<float>(i, 0);
		if (max1 > map1.at<float>(i, image_size.width - 1))
			max1 = map1.at<float>(i, image_size.width - 1);
	}

	for (int j = 0; j<image_size.width; ++j)
	{
		if (min2 < map2.at<float>(0, j))
			min2 = map2.at<float>(0, j);
		if (max2 > map2.at<float>(image_size.height - 1, j))
			max2 = map2.at<float>(image_size.height - 1, j);
	}

	for (int i = 0; i < image_size.height; ++i)
	{
		for (int j = 0; j < image_size.width; ++j)
		{
			map1.at<float>(i, j) = (map1.at<float>(i, j) - min1) / (max1 - min1) * image_size.width;
			map2.at<float>(i, j) = (map2.at<float>(i, j) - min2) / (max2 - min2) * image_size.height;
		}
	}
#endif
  cv::Mat t = imread("../input/111.png");
  //cv::imshow("111", t);
  //cv::waitKey();
  image_Seq.clear();
  image_Seq.push_back(t);
    for (int i = 0 ; i != image_Seq.size(); i++)
    {
        cout<<"Frame #"<<i+1<<"..."<<endl;
        Mat newCameraMatrix = Mat(3,3,CV_32FC1,Scalar::all(0));
        Mat t = image_Seq[i].clone();
        cv::remap(image_Seq[i],t,map1, map2, INTER_LINEAR);
		char name[512];
		sprintf(name, "..\\output\\undistort\\img%d_undistort.jpg", i+1);
		CheckPath(string(name));
        imwrite(name,t);
    }
    cout<<"�������"<<endl;
*/
    return 0;

}

int main2(int argc, char ** argv)
{
    #if 0
    VideoCapture capture;
    int fourcc = CV_FOURCC('H','2','6','4');
    std::cout<<"fourcc: "<<fourcc<<std::endl;
    capture.open("/media/huam//E4BA-118A/F20171229115218_L_t(norm).mp4");//E4BA-118A/F20171229114417_F_t(norm).mp4");// /home/huam/data/FFOutput/3/video_fourth.mp4");

    //capture.set(CV_CAP_PROP_FOURCC, CV_FOURCC('H', '2', '6', '4'));

    double rate = capture.get(CV_CAP_PROP_FPS);//获取视频文件的帧率  
    int delay = cvRound(1000.000 / rate);  
  
    int count = 42;

    if (!capture.isOpened())//判断是否打开视频文件  
    {  
        std::cout<<"cannot open video"<<std::endl;
        return -1;  
    }           
    else  
    {  
        while (true)  
        {  
            Mat frame;  
            capture >> frame;//读出每一帧的图像  
            if (frame.empty()) break;  
            imshow("处理前视频", frame);  
            //processiamge(frame);
            int key = cv::waitKey();
            if(key == 's') 
            {
                cv::imwrite("/home/huam/data/360_left_camera_data/"+std::to_string(count++)+".png", frame);
            }
            //imshow("处理后视频", frame);  
            waitKey(delay);  
        }  
    }  
#else
    VideoCapture capture[4];
    string base_file_path = "/home/huam/video/2/F20180111221621_";
    string video_name_string[4] = {"F", "R", "B", "L"};
    for(int i = 0; i < 4; ++i)
        capture[i].open(base_file_path + video_name_string[i] + "_t(norm).mp4");

    for(int i = 0; i < 4; ++i)
    {
        char file_name[1024];
        double rate = capture[0].get(CV_CAP_PROP_FPS);//获取视频文件的帧率  
        int delay = cvRound(1000.000 / rate); 
        int count = 0;
        if (!capture[i].isOpened())//判断是否打开视频文件  
        {  
            std::cout<<"cannot open video"<<std::endl;
            return -1;  
        }           
        else  
        {  
            while (true)  
            {  
                Mat frame;  
                capture[i] >> frame;//读出每一帧的图像  
                if (frame.empty()) break;  
                //imshow("处理前视频", frame);  
                //processiamge(frame);
                //int key = cv::waitKey();
                //if(key == 's') 
                {
                    //cv::imwrite("/home/huam/data/"+std::to_string(count++)+".png", frame);
                }
                //imshow("处理后视频", frame);
                int interval = 1;
                if(count % interval == 0)
                {
                    sprintf(file_name, "/home/huam/data/test_image_timestamp/camera_%02d_%05d.png", i, count/interval);
                    cv::imwrite(file_name, frame);
                }    
                count++;
                //if(count >= 550) break;
                waitKey(delay);  
            }  
        } 
    }
#endif

    return 0; 
}

