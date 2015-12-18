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
		const std::vector<std::vector<Eigen::Vector3d>> &points_,
		const std::vector<std::vector<Eigen::Vector3d>> &normals_,
		const std::vector<std::vector<int>> &facets_,
		const std::vector<std::vector<Camera>> &cameras_,
		const std::vector<std::string> path_){
			if (cameras_.size() != 0){
				w = cameras_[0][0].W();
				h = cameras_[0][0].H();
			}
			path = path_;
			points.resize(points_.size());
			normals.resize(normals_.size());
			facets.resize(facets_.size());
			cameras.resize(cameras_.size());
			for (int k = 0; k < points_.size(); ++k){
				points[k].resize(points_[k].size());
				for (int i = 0; i < points_[k].size(); ++i){
					points[k][i] = points_[k][i].cast<float>();
				}
				normals[k].resize(normals_[k].size());
				for (int i = 0; i < normals_[k].size(); ++i){
					normals[k][i] = normals_[k][i].cast<float>();
				}
				facets[k].clear();
				std::copy(facets_[k].begin(), facets_[k].end(), std::back_inserter(facets[k]));
				cameras[k].clear();
				std::copy(cameras_[k].begin(), cameras_[k].end(), std::back_inserter(cameras[k]));
			}
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
	static std::vector<std::vector<Eigen::Vector3f>> points;
	static std::vector<std::vector<Eigen::Vector3f>> normals;
	static std::vector<std::vector<int>> facets;
	static std::vector<std::vector<Camera>> cameras;

	static std::vector<std::string> path;
	
	static float znear;
	static float zfar;
	static int frmNo;
	static int seqNo;
	static int w, h;
	static Depth2Model* p_d2m;
	static bool isExit;
};

#endif