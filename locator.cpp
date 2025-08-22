#include "locator.h"



/*################�궨ģ�麯��ʵ��##############*/
// �궨ģ�鹹�캯��
Calibrator::Calibrator(int cali_type, std::string& robot_poses_path, std::string& image_folder_path,
	std::string& parameter_save_path, std::string& image_save_path)
{
	this->calibrate_type = cali_type;
	this->robot_poses_path = robot_poses_path;
	this->image_folder_path = image_folder_path;
	this->parameter_save_path = parameter_save_path;
	this->image_save_path = image_save_path;
}