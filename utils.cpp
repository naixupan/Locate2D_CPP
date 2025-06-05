#include "utils.h"

/****************************
* @description ����ת������ƽ�������ϳ�Ϊ��ξ���
* @param cv::Mat& R 3*3��ת����
* @param cv::Mat& T 3*1ƽ�ƾ���
* @return Mat		4*4��ξ���
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
* @description ����ξ���ֽ�Ϊ��ת�����ƽ�ƾ���
* @param Mat		4*4��ξ���
* @param cv::Mat& R ���3*3��ת����
* @param cv::Mat& T ���3*1ƽ�ƾ���
* @return None
****************************/
void HomogeneousMatrix2R_T(cv::Mat& HomoMtr, cv::Mat& R, cv::Mat& T)
{
	cv::Rect R_rect(0, 0, 3, 3);	//��(0,0)��ʼ������һ�����Ϊ3���߶�Ϊ3�ľ���
	cv::Rect T_rect(3, 0, 1, 3);	//��(3,0)��ʼ������һ�����Ϊ1���߶�Ϊ3�ľ���
	R = HomoMtr(R_rect);	//��ȡ����������Ӿ���
	T = HomoMtr(T_rect);
}

/****************************
* @description ��ŷ����ת��Ϊ��ת����
* @param cv::Mat& eulerAngle	ŷ���ǣ�1*3���󣬽Ƕ�ֵ
* @param string& seq			ָ��ŷ���ǵ�����˳��Ŀǰֻ��xyz
* @return cv::Mat				����3*3��ת����
****************************/
cv::Mat EulerAngleToRotateMatrix(const cv::Mat& eulerAngle, const string& seq)
{
	CV_Assert(eulerAngle.rows == 1 && eulerAngle.cols == 3);	//���ŷ���Ǿ����Ƿ���ȷ
	eulerAngle /= (180 / CV_PI);
	cv::Matx13d m(eulerAngle);	// �ȼ���cv::Matx<double, 1, 3> m(eulerAngle)
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
	// ������ת˳��ϳ���ת����
	cv::Mat rotMat;
	if (seq == "xyz") rotMat = Rotz * Roty * Rotx;
	else if (seq == "zyx") rotMat = Rotx * Roty * Rotz;
	else if (seq == "yzx") rotMat = Rotx * Rotz * Roty;
	else if (seq == "yxz") rotMat = Rotz * Rotx * Roty;
	else if (seq == "xzy") rotMat = Roty * Rotz * Rotx;
	else
	{
		cout << "ŷ������ת˳�����" << endl;
	}

	return rotMat;
}

