#include "utils.h"

/****************************
* @description 将旋转矩阵与平移向量合成为齐次矩阵
* @param cv::Mat& R 3*3旋转矩阵
* @param cv::Mat& T 3*1平移矩阵
* @return Mat		4*4齐次矩阵
****************************/
cv::Mat R_T2HomogeneousMatrix(const cv::Mat& R, const cv::Mat& T)
{
	cv::Mat HomoMtr;
	cv::Mat_<double> R1 = (cv::Mat_<double>(4, 3) <<
		R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
		R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
		R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
		0, 0, 0
		);
	cv::Mat_<double> T1 = (cv::Mat_<double>(4, 1) <<
		T.at<double>(0, 0),
		T.at<double>(1, 0),
		T.at<double>(2, 0),
		1);
	cv::hconcat(R1, T1, HomoMtr);
	return HomoMtr;
}

/****************************
* @description 将齐次矩阵分解为旋转矩阵和平移矩阵
* @param Mat		4*4齐次矩阵
* @param cv::Mat& R 输出3*3旋转矩阵
* @param cv::Mat& T 输出3*1平移矩阵
* @return None
****************************/
void HomogeneousMatrix2R_T(cv::Mat& HomoMtr, cv::Mat& R, cv::Mat& T)
{
	cv::Rect R_rect(0, 0, 3, 3);	//从(0,0)开始，创建一个宽度为3，高度为3的矩形
	cv::Rect T_rect(3, 0, 1, 3);	//从(3,0)开始，创建一个宽度为1，高度为3的矩形
	R = HomoMtr(R_rect);	//提取矩形区域的子矩阵
	T = HomoMtr(T_rect);
}

/****************************
* @description 将欧拉角转换为旋转矩阵
* @param cv::Mat& eulerAngle	欧拉角，1*3矩阵，角度值
* @param string& seq			指定欧拉角的排列顺序，目前只有xyz
* @return cv::Mat				返回3*3旋转矩阵
****************************/
cv::Mat EulerAngleToRotateMatrix(const cv::Mat& eulerAngle, const string& seq)
{
	CV_Assert(eulerAngle.rows == 1 && eulerAngle.cols == 3);	//检查欧拉角矩阵是否正确
	eulerAngle /= (180 / CV_PI);
	cv::Matx13d m(eulerAngle);	// 等价于cv::Matx<double, 1, 3> m(eulerAngle)
	auto rx = m(0, 0), ry = m(0, 1), rz = m(0, 2);
	auto rxs = sin(rx), rxc = cos(rx);
	auto rys = sin(ry), ryc = cos(ry);
	auto rzs = sin(rx), rzc = cos(rz);

	cv::Mat Rotx = (cv::Mat_<double>(3, 3) <<
		1, 0, 0,
		0, rxc, -rxs,
		0, rxs, rxc);
	cv::Mat Roty = (cv::Mat_<double>(3, 3) <<
		ryc, 0, rys,
		0, 1, 0,
		-rys, 0, ryc);
	cv::Mat Rotz = (cv::Mat_<double>(3, 3) <<
		rzc, -rzs, 0,
		rzs, rzc, 0,
		0, 0, 1);
	// 按照旋转顺序合成旋转矩阵
	cv::Mat rotMat;
	if (seq == "xyz") rotMat = Rotz * Roty * Rotx;
	else if (seq == "zyx") rotMat = Rotx * Roty * Rotz;
	else if (seq == "yzx") rotMat = Rotx * Rotz * Roty;
	else if (seq == "yxz") rotMat = Rotz * Rotx * Roty;
	else if (seq == "xzy") rotMat = Roty * Rotz * Rotx;
	else
	{
		cout << "欧拉角旋转顺序错误！" << endl;
	}

	return rotMat;
}

