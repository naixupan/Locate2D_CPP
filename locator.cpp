#include "locator.h"
/*################标定模块函数实现##############*/
// 标定模块构造函数
/*
构造函数用于初始化相关变量，包括手眼标定类型，机械臂位姿文件路径，图像路径等等。
*/
Calibrator::Calibrator(int cali_type, std::string& robot_poses_path, std::string& image_folder_path,
	std::string& parameter_save_path, std::string& image_save_path, cv::Size patternsize, double squareSize)
{
	this->calibrate_type = cali_type;
	this->robot_poses_path = robot_poses_path;
	this->image_folder_path = image_folder_path;
	this->parameter_save_path = parameter_save_path;
	this->image_save_path = image_save_path;
	this->patternsize = patternsize;
	this->squareSize = squareSize;
}

void Calibrator::ReadRobotPoses(cv::Mat& output_mat)
{
	std::string line;
	RobotPose pose = {};
	std::vector<RobotPose> poses;
	std::ifstream file(this->robot_poses_path);
	if (!file.is_open())
	{
		std::cerr << "无法打开文件：" << this->robot_poses_path << std::endl;
		return;
	}
	while (std::getline(file, line))
	{
		if (line.empty() || line[0] == '#') continue;
		std:; istringstream iss(line);	//从字符串中读取数据
		iss >> pose.identifier >> pose.x >> pose.y >> pose.z >> pose.rx >> pose.ry >> pose.rz;
		if (pose.x != 0) poses.push_back(pose);
	}
	std::cout << "读取坐标数量：" << poses.size() << std::endl;
	output_mat.create(poses.size(), 6, CV_64F);
	for (int i = 0; i < output_mat.rows; ++i) {
		output_mat.at<double>(i, 0) = poses[i].x / 1000;
		output_mat.at<double>(i, 1) = poses[i].y / 1000;
		output_mat.at<double>(i, 2) = poses[i].z / 1000;
		output_mat.at<double>(i, 3) = poses[i].rx;
		output_mat.at<double>(i, 4) = poses[i].ry;
		output_mat.at<double>(i, 5) = poses[i].rz;
	}
}

void Calibrator::DetectChessBoard(int image_number)
{
	std::vector<cv::Point2f> corners;
	for (int i = 0; i < image_number; ++i)
	{
		std::string image_path = this->image_folder_path + "image" + std::to_string(i + 1) + ".bmp";
		std::string save_path = this->image_save_path + "image" + std::to_string(i + 1) + ".png";
		cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
		if (image.empty())
		{
			std::cerr << "Read image failed !" << std::endl;
			return;
		}
		cv::Mat gray, img;
		cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
		//std::vector<cv::Point2f> corners;
		bool patternFound = cv::findChessboardCorners(gray, this->patternsize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
			+ cv::CALIB_CB_FAST_CHECK);
		if (patternFound)
			cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
				cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
		imagePoints.push_back(corners);
		cv::drawChessboardCorners(image, patternsize, cv::Mat(corners), patternFound);
		cv::imwrite(save_path, image);
		std::cout << "图像：" << image_path << "   已保存：" << save_path << std::endl;
		this->image_size = gray.size();
	}
}
void Calibrator::CalibrateEyeInHand()
{
	/*
	手眼标定工作流程
	1、读取机械臂坐标；
	2、将机械臂坐标转化为位姿矩阵gripper2base；
	3、检测棋盘格角点，并转化为位姿矩阵mark2camera；
	4、手眼标定计算，输出位姿矩阵camera2gripper；
	5、手眼标定测试（可选）。
	*/
	// 1.read robot poses
	cv::Mat robot_poses;
	this->ReadRobotPoses(robot_poses);
	pose_number = robot_poses.rows;
	// 2.pose -> matrix
	cv::Mat rotate_matrix;
	for (int i = 0; i < robot_poses.rows; ++i)
	{
		cv::Mat euler_angle = (cv::Mat_<double>(1, 3) <<
			robot_poses.at<double>(i, 3), robot_poses.at<double>(i, 4), robot_poses.at<double>(i, 5)
			);
		cv::Mat translation_vector = (cv::Mat_<double>(3, 1) <<
			robot_poses.at<double>(i, 0), robot_poses.at<double>(i, 1), robot_poses.at<double>(i, 2)
			);
		std::string seq = "xyz";
		EulerAngleToRotateMatrix(euler_angle, seq, rotate_matrix);
		R_gripper2base.push_back(rotate_matrix);
		T_gripper2base.push_back(translation_vector);
	}
	//3.detect corners
	DetectChessBoard(robot_poses.rows);
	CalibrateCamera(robot_poses.rows);
	cv::Mat rotationMatrix;
	for (int i = 0; i < robot_poses.rows; ++i)
	{
		cv::Mat translateMatrix = (cv::Mat_<double>(3, 1) <<
			tvecs[i].at<double>(0, 1), tvecs[i].at<double>(0, 2), tvecs[i].at<double>(0, 3));
		cv::Rodrigues(rvecs[i], rotationMatrix);
		R_mark2camera.push_back(rotationMatrix);
		T_mark2camera.push_back(translateMatrix);
	}
	cv::Mat R_camera2gripper, T_camera2gripper;
	cv::calibrateHandEye(R_gripper2base, T_gripper2base, R_mark2camera, T_mark2camera, R_camera2gripper, T_camera2gripper, cv::CALIB_HAND_EYE_TSAI);
	std::cout << "标定结果：" << std::endl;
	std::cout << "旋转矩阵：" << R_camera2gripper << std::endl;
	std::cout << "平移向量：" << T_camera2gripper << std::endl;
	RT_camera2gripper = R_T2HomogeneousMatrix(R_camera2gripper, T_camera2gripper);
	std::cout << "齐次矩阵：\n" << RT_camera2gripper << std::endl;
	cv::FileStorage fs("./Parameter/camera2gripperMat.yaml", cv::FileStorage::WRITE);
	fs << "matrix" << RT_camera2gripper;
	fs.release();
}

void Calibrator::CalibrateCamera(int image_number)
{
	cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	distCoeffs = cv::Mat::zeros(1, 8, CV_64F);
	//std::vector<cv::Mat> rvecs, tvecs;
	std::vector<cv::Point3f> obj;
	std::vector<std::vector<cv::Point3f>>objectPoints;
	//使用高精度模型
	int flags = cv::CALIB_RATIONAL_MODEL;
	for (int i = 0; i < patternsize.height; ++i)
	{
		for (int j = 0; j < patternsize.width; ++j)
		{
			obj.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
		}
	}
	//创建多幅图像对象点
	for (int i = 0; i < image_number; ++i)
	{
		objectPoints.push_back(obj);
	}
	double rms = cv::calibrateCamera(objectPoints, imagePoints, image_size, cameraMatrix, distCoeffs, rvecs, tvecs, flags);
	std::cout << "标定误差 (RMS): " << rms << " 像素" << std::endl;
	std::cout << "相机内参矩阵:\n" << cameraMatrix << std::endl;
	std::cout << "畸变系数: " << distCoeffs.t() << std::endl;
	cv::FileStorage fs("./Parameter/camera_params.yaml", cv::FileStorage::WRITE);
	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;
	fs << "image_size" << image_size;
	fs << "reprojection_error" << rms;
	fs.release();
}

void Calibrator::CalibrateTest()
{
	std::vector<cv::Mat> Homo_gripper2baseVec;
	std::vector<cv::Mat> Homo_mark2cameraVec;
	std::vector<cv::Mat> Homo_mark2baseVec;
	std::vector<cv::Mat> outputPoses;
	cv::Mat Homo_gripper2base, Homo_mark2camera, Homo_mark2base;
	cv::Mat tempHomoMat;
	cv::Mat outputPose;
	for (int i = 0; i < pose_number; ++i)
	{
		Homo_gripper2base = R_T2HomogeneousMatrix(R_gripper2base[i], T_gripper2base[i]);
		Homo_mark2camera = R_T2HomogeneousMatrix(R_mark2camera[i], T_mark2camera[i]);
		Homo_gripper2baseVec.push_back(Homo_gripper2base);
		Homo_gripper2baseVec.push_back(Homo_mark2camera);
		cv::gemm(Homo_gripper2base, RT_camera2gripper, 1.0, cv::noArray(), 0, tempHomoMat);
		cv::gemm(tempHomoMat, Homo_mark2camera, 1.0, cv::noArray(), 0, Homo_mark2base);
		Homo_mark2baseVec.push_back(Homo_mark2base);
		outputPose = HomogeneousMatrix2Pose(Homo_mark2base);
		outputPoses.push_back(outputPose);
		std::cout << "Mark点在机械臂基坐标系下坐标：" << "\nX:" << outputPose.at<double>(0, 0) << "  Y:" << outputPose.at<double>(0, 1) << " Z:"
			<< outputPose.at<double>(0, 2) << " Rx:" << outputPose.at<double>(0, 3) << " Ry:" << outputPose.at<double>(0, 4) << " Rz:"
			<< outputPose.at<double>(0, 5) << std::endl;
	}
}