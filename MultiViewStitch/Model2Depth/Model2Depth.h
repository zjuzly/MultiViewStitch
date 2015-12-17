#ifndef MODEL2DEPTH_H
#define MODEL2DEPTH_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <Eigen/Dense>

#include "../Common/Utils.h"
#include "../Camera/Camera.h"

class Depth2Model;

class Model2Depth{
public:
	static void SetInput(
		const std::vector<Eigen::Vector3d> &points_,
		const std::vector<Eigen::Vector3d> &normals_,
		const std::vector<int> &facets_,
		const std::vector<Camera> &cameras_,
		const std::string path_/*,
		const int &w_, const int &h_*/){
			//w = w_; h = h_;
			if (cameras_.size() != 0){
				w = cameras_[0].W();
				h = cameras_[0].H();
			}
			path = path_;
			points.clear();
			normals.clear();
			facets.clear();
			cameras.clear();
			//std::copy(points_.begin(), points_.end(), std::back_inserter(points));
			//std::copy(normals_.begin(), normals_.end(), std::back_inserter(normals));
			points.resize(points_.size());
			normals.resize(normals_.size());
			for (int i = 0; i < points_.size(); ++i){
				points[i] = points_[i].cast<float>();
			}
			for (int i = 0; i < normals_.size(); ++i){
				normals[i] = normals_[i].cast<float>();
			}
			std::copy(facets_.begin(), facets_.end(), std::back_inserter(facets));
			std::copy(cameras_.begin(), cameras_.end(), std::back_inserter(cameras));
	}
	static void InitGL();
	static void Reshape(int w, int h);
	static void RenderSence();
	static void Draw();
	static bool UpdateOnce();
	static void RenderDepth();
	static void IdleFunc();
	static void Run(int argv, char *argc[]);
protected:
	static void GetClippingPlane(double proj[], double &znear, double &zfar);
private:
	static std::vector<Eigen::Vector3f> points;
	static std::vector<Eigen::Vector3f> normals;
	static std::vector<int> facets;
	static std::vector<Camera> cameras;

	static std::string path;
	
	static float znear;
	static float zfar;
	static int frmNo;
	static int w, h;
	static Depth2Model* p_d2m;
	static bool isExit;
};

#endif