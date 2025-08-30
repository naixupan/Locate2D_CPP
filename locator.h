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
				std::string& parameter_save_path, std::string& image_save_path, cv::Size patternsize,
				double squareSize);
	~Calibrator() {};
	void CalibrateEyeInHand();
	void CalibrateCamera(int image_number);
	void CalibrateTest();
	void ComputeRelativePoseMat();

private:
	std::string parameter_save_path;		// ��������·��
	std::string image_save_path;
	std::string image_folder_path;
	std::string robot_poses_path;
	int calibrate_type;
	cv::Size patternsize;
	double squareSize;
	cv::Size image_size;
	int pose_number;
	//std::vector<cv::Point2f> corners;
	std::vector<std::vector<cv::Point2f>> imagePoints;
	//int image_number;
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;

	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;

	cv::Mat RT_camera2gripper;

	std::vector<cv::Mat>R_gripper2base, T_gripper2base;
	std::vector<cv::Mat>R_mark2camera, T_mark2camera;

	//���۱궨�����������
	void ReadRobotPoses(cv::Mat& output_mat);
	void DetectChessBoard(int image_number);


};