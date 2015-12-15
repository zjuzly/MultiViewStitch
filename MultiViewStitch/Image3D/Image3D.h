#ifndef IMAGE3D_H
#define IMAGE3D_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <opencv2\opencv.hpp>
#include "../Camera/Camera.h"

class Image3D{
public:
	Image3D(){}
	void LoadModel(const std::string imgpath, const std::string rawpath, const Camera &cam);
	int GetTexIndex(int view, int x, int y) const{ return texIndex[view][y * cam.W() + x]; }
	bool IsValid(int x, int y) const{ return valid[y * cam.W() + x]; }
	bool InMask(int x, int y) const{ return !!(mask.at<uchar>(y, x)); }
	Eigen::Vector3d GetPoint(int x, int y) const{ return point3d[y * cam.W() + x]; }
	Eigen::Vector3d GetPointCam(int x, int y) const{
		Eigen::Vector3d p3d_c;
		cam.GetCamCoordFromWorld(point3d[y * cam.W() + x], p3d_c);
		return p3d_c;
	}
	Camera GetCamera() const{ return cam; }
	int GetWidth() const{ return cam.W(); }
	int GetHeight() const{ return cam.H(); }
private:
	void LoadDepthMap(const std::string rawpath, std::vector<double> &depth);
	void SolveUnProjectionD(const std::vector<double> &depth);
	void GenNewViews();
private:
	int frmNo;
	cv::Mat image;
	cv::Mat mask;
	Camera cam;
	std::string path;
	std::vector<std::vector<int>> texIndex;
	std::vector<Eigen::Vector3d> point3d;
	std::vector<bool> valid;
	
};

#endif