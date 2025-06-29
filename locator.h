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


class Calibrator
{
public:
	Calibrator();
	~Calibrator();
	void calibrate_eye_in_hand();
	void calibrate_camera();
	void calibrate_test();

private:
	std::string save_path;
	std::string image_folder_path;
	std::string robot_poses_path;

	cv::Mat read_robot_poses();


};