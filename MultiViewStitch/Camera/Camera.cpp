#include "Camera.h"
#include "../Common/Utils.h"
#include <fstream>
#include <iostream>

Eigen::Matrix4f Camera::GetObjAbsTransformGL(){
	Eigen::Matrix4f glMatrix = Eigen::Matrix4f::Identity();
	glMatrix.block(0, 0, 3, 3) = R.cast<float>();
	glMatrix.block(0, 3, 3, 1) = t.cast<float>();
	glMatrix.row(1) = -glMatrix.row(1);
	glMatrix.row(2) = -glMatrix.row(2);
	return glMatrix;
}

void Camera::GetFrustumGL(float znear, float &left, float &right, float &bottom, float &top){
	float cx = K(0, 2);
	float cy = K(1, 2);
	float fx = K(0, 0);
	float fy = K(1, 1);
	left = cx / fx * znear;
	top = cy / fy * znear; //0.00304
	right = (w - cx) / cx * left; //0.00539
	bottom = (h - cy) / cy * top; //0.00303
	left = -left;//-0.00541
	bottom = -bottom;//-0.00303
}

Eigen::Matrix4f Camera::GetProjectGL(float left, float right, float bottom, float top, float znear, float zfar){
	Eigen::Matrix4f projMatrix = Eigen::Matrix4f::Zero();
	projMatrix(0, 0) = 2 * znear / (right - left); //right - left = 0.0108; 1.852
	projMatrix(1, 1) = 2 * znear / (top - bottom); //top - bottom = 0.00607; 3.295
	projMatrix(2, 0) = (right + left) / (right - left);
	projMatrix(2, 1) = (top + bottom) / (top - bottom);
	projMatrix(2, 2) = -(zfar + znear) / (zfar - znear);
	projMatrix(2, 3) = -1;
	projMatrix(3, 2) = -2 * zfar * znear / (zfar - znear);
	return projMatrix;
}

void Camera::GetCamCoordFromImg(const int u, const int v, const double d, Eigen::Vector3d &p3d) const{
	p3d[0] = (u - K(0, 2)) * d / K(0, 0);
	p3d[1] = (v - K(1, 2)) * d / K(1, 1);
	p3d[2] = d;
}
void Camera::GetImgCoordFromCam(const Eigen::Vector3d &p3d, int &u, int &v) const{
	u = (int)(K(0, 0) * p3d[0] / p3d[2] + K(0, 2) + 0.5);
	v = (int)(K(1, 1) * p3d[1] / p3d[2] + K(1, 2) + 0.5);
}

void Camera::GetWorldCoordFromImg(const int u, const int v, const double d, Eigen::Vector3d &p3d) const{
	Eigen::Vector3d tmp;
	GetCamCoordFromImg(u, v, d, tmp);
	GetWorldCoordFromCam(tmp, p3d);
}
void Camera::GetImgCoordFromWorld(const Eigen::Vector3d &p3d, int &u, int &v) const{
	Eigen::Vector3d p3d_c;
	GetCamCoordFromWorld(p3d, p3d_c);
	GetImgCoordFromCam(p3d_c, u, v);
}

void Camera::GetWorldCoordFromCam(const Eigen::Vector3d &p3d_c, Eigen::Vector3d &p3d_w) const{
	Eigen::Vector3d tmp = p3d_c;
	tmp[0] -= t[0]; tmp[1] -= t[1]; tmp[2] -= t[2];
	p3d_w[0] = R(0, 0) * tmp[0] + R(1, 0) * tmp[1] + R(2, 0) * tmp[2];
	p3d_w[1] = R(0, 1) * tmp[0] + R(1, 1) * tmp[1] + R(2, 1) * tmp[2];
	p3d_w[2] = R(0, 2) * tmp[0] + R(1, 2) * tmp[1] + R(2, 2) * tmp[2];
}
void Camera::GetCamCoordFromWorld(const Eigen::Vector3d &p3d_w, Eigen::Vector3d &p3d_c) const{
	p3d_c[0] = R(0, 0) * p3d_w[0] + R(0, 1) * p3d_w[1] + R(0, 2) * p3d_w[2] + t[0];
	p3d_c[1] = R(1, 0) * p3d_w[0] + R(1, 1) * p3d_w[1] + R(1, 2) * p3d_w[2] + t[1];
	p3d_c[2] = R(2, 0) * p3d_w[0] + R(2, 1) * p3d_w[1] + R(2, 2) * p3d_w[2] + t[2];
}

std::vector<Camera> LoadCalibrationFromActs(std::string filename){
	std::cout << "LoadCalibrationFromActs" << std::endl;
	std::cout << filename << std::endl;
	std::ifstream ifs(filename.c_str(), std::ifstream::in);
	if (!ifs.is_open()){
		std::cerr << "Open File " << filename << " Failed in File = " << __FILE__  << ", Line = " << __LINE__ << std::endl;
		exit(-1);
	}
	int start, end, step;
	std::vector<Camera> vcamera;
	Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
	
	while (ifs.peek() != EOF){
		char line[4096];
		ifs.getline(line, 4096);
		std::string str(line);
		if(str.size() == 0 || str[0] == '#') continue;
		if (str == "<intrinsic parameter>"){
			ifs.getline(line, 1024);
			std::vector<std::string> strs = Split(std::string(line), ' ');
			std::vector<float> intri;
			for (int i = 0; i < (int)strs.size(); ++i){
				intri.push_back(std::atof(strs[i].c_str()));
			}
			K(0, 0) = intri[0]; K(1, 1) = intri[1];
			K(0, 2) = intri[2]; K(1, 2) = intri[3];
			K(2, 2) = 1.0;
		}
		else if (str == "<Camera Track>"){
			Eigen::Matrix3d R;
			Eigen::Vector3d t;
			char fstr[128], row[4][128];
			for (; start <= end; start += step){
				ifs.getline(line, 128);
				ifs.getline(fstr, 128);
				for (int i = 0; i < 4; ++i){
					ifs.getline(row[i], 128);
				}
				ifs.getline(line, 128);
				std::vector<std::string> srow;
				srow = Split(std::string(row[0]), ' ');
				R(0, 0) = std::atof(srow[0].c_str());
				R(0, 1) = std::atof(srow[1].c_str());
				R(0, 2) = std::atof(srow[2].c_str());
				t[0] = std::atof(srow[3].c_str());

				srow = Split(std::string(row[1]), ' ');
				R(1, 0) = std::atof(srow[0].c_str());
				R(1, 1) = std::atof(srow[1].c_str());
				R(1, 2) = std::atof(srow[2].c_str());
				t[1] = std::atof(srow[3].c_str());

				srow = Split(std::string(row[2]), ' ');
				R(2, 0) = std::atof(srow[0].c_str());
				R(2, 1) = std::atof(srow[1].c_str());
				R(2, 2) = std::atof(srow[2].c_str());
				t[2] = std::atof(srow[3].c_str());

				Camera cam;
				cam.SetK(K);
				cam.SetRT(R, t);
				cam.SetW((int)(2 * (K(0, 2) + 0.5)));
				cam.SetH((int)(2 * (K(1, 2) + 0.5)));

				if (K(0, 0) > 0)	cam.ValidateModel();
				vcamera.push_back(cam);
			}
			break;
		}
		else{
			int pos = (int)str.find_first_of(':', 0);
			if (pos != -1){
				std::string tmp = str.substr(0, pos);
				if (tmp == "start")		start = std::atoi(str.substr(pos + 1, str.size() - pos - 1).c_str());
				else if (tmp == "step")	step = std::atoi(str.substr(pos + 1, str.size() - pos - 1).c_str());
				else if (tmp == "end")	end = std::atoi(str.substr(pos + 1, str.size() - pos - 1).c_str());
			}
		}
	}
	ifs.close();
	std::cout << vcamera.size() << std::endl;
	std::cout << "Load Calibration Parameters done!" << std::endl;
	return vcamera;
}