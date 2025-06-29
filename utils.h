#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>

using namespace std;

// 加载相机内参
/*
输入：
@camera_parameter_path：相机内参路径
输出：
@camera_patameter(Mat*)，指针
*/

void LoadCameraParameter(string camera_patameter_path);
void LoadCalibrateMatrix(string matrix_path);
void ReadRobotPoses(string poses_path);
cv::Mat EulerAngleToRotateMatrix(const cv::Mat& eulerAngle, const string& seq);
cv::Mat RotateMatrixToEulerAngle(const cv::Mat& R_Matrix);

cv::Mat R_T2HomogeneousMatrix(const cv::Mat& R, const cv::Mat& T);
void HomogeneousMatrix2R_T(cv::Mat& HomoMtr, cv::Mat& R, cv::Mat& T);
cv::Mat Pose2HomogenousMatrix(std::vector<float>& pose);
cv::Mat HomogeneousMatrix2Pose(cv::Mat& HmgMatrix);
void DetectChessBoard();
void DetectArUcoMarks();
void ComputeCameraPose();

