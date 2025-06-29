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

/****************************
* @description 将旋转矩阵转化为欧拉角
* @param cv::Mat& R_Matrix		3*3旋转矩阵
* @param string& seq			指定欧拉角的排列顺序，目前只有xyz
* @return cv::Mat				返回1*3欧拉角坐标
****************************/
cv::Mat RotateMatrixToEulerAngle(const cv::Mat& R_Matrix)
{
	constexpr double EPSILON = 1e-12;
	constexpr double PI = 3.14159265358979323846;

	double yaw;   // 偏航角 (绕Z轴旋转)
	double pitch; // 俯仰角 (绕Y轴旋转)
	double roll;  // 滚转角 (绕X轴旋转)

	// 矩阵元素访问
	double r11 = R_Matrix.at<double>(0, 0);
	double r12 = R_Matrix.at<double>(0, 1);
	double r13 = R_Matrix.at<double>(0, 2);
	double r21 = R_Matrix.at<double>(1, 0);
	double r22 = R_Matrix.at<double>(1, 1);
	double r23 = R_Matrix.at<double>(1, 2);
	double r31 = R_Matrix.at<double>(2, 0);
	double r32 = R_Matrix.at<double>(2, 1);
	double r33 = R_Matrix.at<double>(2, 2);

	if (std::abs(r31) > 1.0 - EPSILON) {
		yaw = 0;
		pitch = -std::copysign(PI / 2, r31);
		roll = std::atan2(-r12, r11);
	}
	else {
		// 标准情况
		yaw = std::atan2(r21, r11);
		pitch = std::asin(-r31);
		roll = std::atan2(r32, r33);
	}
	
	cv::Mat EulerAngle = cv::Mat_<double>(1, 3) << roll, pitch, yaw;

	return EulerAngle;
}


/****************************
* @description					将机械臂位姿转化为齐次矩阵
* @param vector<float>& pose	机械臂位姿，x,y,z,rz,ry,rz
* @return cv::Mat				返回4*4齐次矩阵
****************************/
cv::Mat Pose2HomogenousMatrix(std::vector<float>& pose)
{
	vector<double> t_pose(pose.begin(), pose.begin() + 3);
	//vector<float> r_pose(pose.begin() + 3, pose.end());
	
	cv::Mat r_pose = (cv::Mat_<double>(1, 3) << pose[3], pose[4], pose[5]);
	for (int i = 0; i < 3; ++i)
	{
		t_pose[i] /= 1000;
	}
	cv::Mat t_pose_ = (cv::Mat_<double>(3, 1) << t_pose[0], t_pose[1], t_pose[2]);

	cv::Mat r_matrix = EulerAngleToRotateMatrix(r_pose,"xyz");

	cv::Mat HmgMat = cv::Mat::eye(4, 4, CV_64F);
	cv::Rect R_range(0, 0, 3, 3);
	cv::Rect T_range(3, 0, 1, 3);
	t_pose_.copyTo(HmgMat(T_range));
	r_matrix.copyTo(HmgMat(R_range));
	return HmgMat;
}
/****************************
* @description					将齐次矩阵转化为机械臂位姿
* @param cv::Mat				4*4齐次矩阵
* @return cv::Mat	返回位姿，x,y,z,rz,ry,rz
****************************/
cv::Mat HomogeneousMatrix2Pose(cv::Mat& HmgMatrix)
{
	cv::Mat R_Matrix, T_vector, R_pose;
	HomogeneousMatrix2R_T(HmgMatrix, R_Matrix, T_vector);
	R_pose = RotateMatrixToEulerAngle(R_Matrix);


	//cv::Mat pose = (cv::Mat_<double>(1,6)<<T_vector[0], T_vector[1], T_vector[2], R_pose[0], R




	

}

