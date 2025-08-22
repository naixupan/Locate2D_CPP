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
���۱궨ģ�鹦�ܷ�����
1����ȡ��е�������ļ���
2����ȡ�궨��ͼ�񣬼��궨��ǵ㣻
3�������е��λ�ˣ�gripper2base)
4������궨�嵽�������ϵ��λ�ˣ�mark2camera��
5�����۱궨�����������е��ĩ��λ�ˣ�camera2gripper��
6�����۱궨���ԣ�����궨���ڻ�����ϵ�µ����꣨mark2base��
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
	std::string parameter_save_path;		// ��������·��
	std::string image_save_path;
	std::string image_folder_path;
	std::string robot_poses_path;
	int calibrate_type;
	cv::Size patternsize;
	double squareSize;

	cv::Mat read_robot_poses();


};