#include "locator.h"
/*################�궨ģ�麯��ʵ��##############*/
// �궨ģ�鹹�캯��
/*
���캯�����ڳ�ʼ����ر������������۱궨���ͣ���е��λ���ļ�·����ͼ��·���ȵȡ�
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
		std::cerr << "�޷����ļ���" << this->robot_poses_path << std::endl;
		return;
	}
	while (std::getline(file, line))
	{
		if (line.empty() || line[0] == '#') continue;
		std:; istringstream iss(line);	//���ַ����ж�ȡ����
		iss >> pose.identifier >> pose.x >> pose.y >> pose.z >> pose.rx >> pose.ry >> pose.rz;
		if (pose.x != 0) poses.push_back(pose);
	}
	std::cout << "��ȡ����������" << poses.size() << std::endl;
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
		std::cout << "ͼ��" << image_path << "   �ѱ��棺" << save_path << std::endl;
		this->image_size = gray.size();
	}
}
void Calibrator::CalibrateEyeInHand()
{
	/*
	���۱궨��������
	1����ȡ��е�����ꣻ
	2������е������ת��Ϊλ�˾���gripper2base��
	3��������̸�ǵ㣬��ת��Ϊλ�˾���mark2camera��
	4�����۱궨���㣬���λ�˾���camera2gripper��
	5�����۱궨���ԣ���ѡ����
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
	std::cout << "�궨�����" << std::endl;
	std::cout << "��ת����" << R_camera2gripper << std::endl;
	std::cout << "ƽ��������" << T_camera2gripper << std::endl;
	RT_camera2gripper = R_T2HomogeneousMatrix(R_camera2gripper, T_camera2gripper);
	std::cout << "��ξ���\n" << RT_camera2gripper << std::endl;
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
	//ʹ�ø߾���ģ��
	int flags = cv::CALIB_RATIONAL_MODEL;
	for (int i = 0; i < patternsize.height; ++i)
	{
		for (int j = 0; j < patternsize.width; ++j)
		{
			obj.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
		}
	}
	//�������ͼ������
	for (int i = 0; i < image_number; ++i)
	{
		objectPoints.push_back(obj);
	}
	double rms = cv::calibrateCamera(objectPoints, imagePoints, image_size, cameraMatrix, distCoeffs, rvecs, tvecs, flags);
	std::cout << "�궨��� (RMS): " << rms << " ����" << std::endl;
	std::cout << "����ڲξ���:\n" << cameraMatrix << std::endl;
	std::cout << "����ϵ��: " << distCoeffs.t() << std::endl;
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
		std::cout << "Mark���ڻ�е�ۻ�����ϵ�����꣺" << "\nX:" << outputPose.at<double>(0, 0) << "  Y:" << outputPose.at<double>(0, 1) << " Z:"
			<< outputPose.at<double>(0, 2) << " Rx:" << outputPose.at<double>(0, 3) << " Ry:" << outputPose.at<double>(0, 4) << " Rz:"
			<< outputPose.at<double>(0, 5) << std::endl;
	}
}