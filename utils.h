#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>

using namespace std;

// ��������ڲ�
/*
���룺
@camera_parameter_path������ڲ�·��
�����
@camera_patameter(Mat*)��ָ��
*/

void LoadCameraParameter(string camera_patameter_path);
void LoadCalibrateMatrix(string matrix_path);
void ReadRobotPoses(string poses_path);
cv::Mat EulerAngleToRotateMatrix(const cv::Mat& eulerAngle, const string& seq);
void RotateMatrixToEulerAngle();

cv::Mat R_T2HomogeneousMatrix(const cv::Mat& R, const cv::Mat& T);
void HomogeneousMatrix2R_T(cv::Mat& HomoMtr, cv::Mat& R, cv::Mat& T);
void Pose2HomogenousMatrix();
void HomogeneousMatrix2Pose();
void DetectChessBoard();
void DetectArUcoMarks();
void ComputeCameraPose();

