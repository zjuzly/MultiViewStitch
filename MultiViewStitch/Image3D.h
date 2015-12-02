#ifndef IMAGE3D_H
#define IMAGE3D_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cv.h>
#include <highgui.h>
#include "Camera.h"

class Image3D{
public:
	Image3D(int viewCount_ = 1, int axis_ = 0, double rotAngle_ = 0.0){ frmNo = 0; viewCount = viewCount_; axis = axis_; rotAngle = rotAngle_; }
	void LoadModel(const std::string imgpath, const std::string rawpath, const Camera &cam);
	int GetViewCount() const{ return viewCount; }
	int GetTexIndex(int view, int x, int y) const{ return texIndex[view][y * cam.W() + x]; }
	bool IsValid(int x, int y) const{ return valid[y * cam.W() + x]; }
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
	int viewCount;
	int axis;
	double rotAngle;
	cv::Mat image;
	Camera cam;
	std::string path;
	std::vector<std::vector<int>> texIndex;
	std::vector<Eigen::Vector3d> point3d;
	std::vector<bool> valid;
	
};

#endif