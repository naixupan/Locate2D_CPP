#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include "utils.h"

class Locator
{
public:
	Locator();
	~Locator();
};

/*
手眼标定模块功能分析：
1、读取机械臂坐标文件；
2、读取标定板图像，检测标定板角点；
3、计算机械臂位姿（gripper2base)
4、计算标定板到相机坐标系下位姿（mark2camera）
5、手眼标定计算相机到机械臂末端位姿（camera2gripper）
6、手眼标定测试，计算标定板在基坐标系下的坐标（mark2base）
*/

class Calibrator
{
public:
	Calibrator(int cali_type,std::string& robot_poses_path,std::string& image_folder_path,
				std::string& parameter_save_path, std::string& image_save_path);
	~Calibrator() {};
	void calibrate_eye_in_hand();
	void calibrate_camera();
	void calibrate_test();
	void compute_relative_pose_mat();

private:
	std::string parameter_save_path;		// 参数保存路径
	std::string image_save_path;
	std::string image_folder_path;
	std::string robot_poses_path;
	int calibrate_type;
	cv::Size patternsize;
	double squareSize;

	cv::Mat read_robot_poses();


};