#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cctype> 

using namespace std;

// ��������ڲ�
/*
���룺
@camera_parameter_path������ڲ�·��
�����
@camera_patameter(Mat*)��ָ��
*/

struct RobotPose {
	std::string identifier = " ";  // �����ֶδ洢ǰ׺��ʶ
	double x = 0, y = 0, z = 0;
	double rx = 0, ry = 0, rz = 0;
};

struct EulerAngles {
	double roll;
	double pitch;
	double yaw;
};

void LoadCameraParameter(string camera_patameter_path);
void LoadCalibrateMatrix(string matrix_path);
void ReadRobotPoses(string poses_path, cv::Mat& poses_Mat);
cv::Mat EulerAngleToRotateMatrix(const cv::Mat& eulerAngle, const string& seq);
cv::Mat RotateMatrixToEulerAngle(const cv::Mat& R_Matrix);

cv::Mat R_T2HomogeneousMatrix(const cv::Mat& R, const cv::Mat& T);
void HomogeneousMatrix2R_T(cv::Mat& HomoMtr, cv::Mat& R, cv::Mat& T);
cv::Mat Pose2HomogenousMatrix(std::vector<float>& pose);
cv::Mat HomogeneousMatrix2Pose(cv::Mat& HmgMatrix);
void DetectChessBoard();
void DetectArUcoMarks();
void ComputeCameraPose();

