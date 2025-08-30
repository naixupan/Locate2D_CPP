#include <iostream>
#include "utils.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cctype> // for std::isdigit

void demo_calibrate()
{
    std::vector<cv::Mat>R_gripper2base, T_gripper2base;
    std::vector<cv::Mat>R_mark2camera, T_mark2camera;
    std::cout.precision(12);
    //1。Read robot poses
    cv::Mat outputMat;
    std::string pose_path = "D:/CPlusPlusCode/Locate2D_CPP/images/Calibrate/CalibrateImages20250626_1/poses.txt";
    ReadRobotPoses(pose_path, outputMat);
    std::cout << outputMat << std::endl;
    //2.Transfrom poses to matrix
    cv::Mat rotate_mat;
    for (int i = 0; i < outputMat.rows; ++i)
    {
        cv::Mat euler_angle = (cv::Mat_<double>(1, 3) <<
            outputMat.at<double>(i, 3), outputMat.at<double>(i, 4), outputMat.at<double>(i, 5)
            );
        cv::Mat translation_vector = (cv::Mat_<double>(3, 1) <<
            outputMat.at<double>(i, 0),
            outputMat.at<double>(i, 1),
            outputMat.at<double>(i, 2)
            );
        std::string seq = "xyz";
        rotate_mat = EulerAngleToRotateMatrix(euler_angle, seq);
        std::cout << "欧拉角：" << euler_angle << std::endl;
        std::cout << "旋转矩阵" << rotate_mat << std::endl;
        R_gripper2base.push_back(rotate_mat);
        T_gripper2base.push_back(translation_vector);
    }
    //3.Load chess board images
    std::string image_folder = "D:/CPlusPlusCode/Locate2D_CPP/images/Calibrate/CalibrateImages20250626_1/";
    std::string save_folder = "D:/CPlusPlusCode/Locate2D_CPP/images/ResultImages/CalibrateImages20250626_1/";
    cv::Size patternsize(19, 17);
    double squareSize = 0.0015;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    cv::Size image_size;
    for (int i = 0; i < outputMat.rows; ++i)
    {
        std::string image_path = image_folder + "image" + std::to_string(i + 1) + ".bmp";
        std::string save_path = save_folder + "image" + std::to_string(i + 1) + ".png";

        cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
        if (image.empty()) {
            std::cerr << "无法读取图像" << std::endl;
            return -1;
        }
        cv::Mat gray;
        cv::Mat img;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        bool patternfound = cv::findChessboardCorners(gray, patternsize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
            + cv::CALIB_CB_FAST_CHECK);
        if (patternfound)
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
        imagePoints.push_back(corners);
        cv::drawChessboardCorners(image, patternsize, cv::Mat(corners), patternfound);
        cv::imwrite(save_path, image);
        std::cout << "图像：" << image_path << "   已保存：" << save_path << std::endl;
        image_size = gray.size();
    }
    std::cout << "图像尺寸：" << image_size << std::endl;
    //4.Analisy board images
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(1, 8, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<cv::Point3f> obj;
    std::vector<std::vector<cv::Point3f>>objectPoints;
    //int flags = cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_K3;
    int flags = cv::CALIB_RATIONAL_MODEL;
    //Create one object point
    for (int i = 0; i < patternsize.height; ++i)
    {
        for (int j = 0; j < patternsize.width; ++j)
        {
            obj.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        }
    }
    //create many object point
    for (int i = 0; i < outputMat.rows; ++i)
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
    //5.Compute board pose
    for (int i = 0; i < outputMat.rows; ++i)
    {
        std::cout << "标定板->相机 旋转矩阵:" << rvecs[i] << std::endl;
        std::cout << "             平移向量:" << tvecs[i] << std::endl;
        cv::Mat rotationMatrix;
        cv::Mat translateMatrix = (cv::Mat_<double>(3, 1) <<
            tvecs[i].at<double>(0, 1), tvecs[i].at<double>(0, 2), tvecs[i].at<double>(0, 3));
        cv::Rodrigues(rvecs[i], rotationMatrix);
        R_mark2camera.push_back(rotationMatrix);
        T_mark2camera.push_back(translateMatrix);
    }
    //6.Calibrate
    cv::Mat R_camera2gripper, T_camera2gripper;
    cv::Mat RT_camera2gripper;
    cv::calibrateHandEye(R_gripper2base, T_gripper2base, R_mark2camera, T_mark2camera, R_camera2gripper, T_camera2gripper, cv::CALIB_HAND_EYE_TSAI);
    std::cout << "标定结果：" << std::endl;
    std::cout << "旋转矩阵：" << R_camera2gripper << std::endl;
    std::cout << "平移向量：" << T_camera2gripper << std::endl;
    RT_camera2gripper = R_T2HomogeneousMatrix(R_camera2gripper, T_camera2gripper);
    std::cout << "齐次矩阵：\n" << RT_camera2gripper << std::endl;
    //7.Calibrate test
    std::cout << "########### 手眼标定测试 ###########" << std::endl;
    std::vector<cv::Mat> Homo_gripper2baseVec;
    std::vector<cv::Mat> Homo_mark2cameraVec;
    std::vector<cv::Mat> Homo_mark2baseVec;
    std::vector<cv::Mat> outputPoses;
    for (int i = 0; i < outputMat.rows; ++i)
    {
        cv::Mat Homo_gripper2base, Homo_mark2camera, Homo_mark2base;
        cv::Mat tempHomoMat;
        cv::Mat outputPose;
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




int main()
{
    
    return 0;
}