#ifndef SRTSOLVER_H
#define SRTSOLVER_H
#include <time.h>
#include <vector>
#include <Eigen/Dense>
#include "Camera.h"

class SRTSolver{
public:
	SRTSolver(){ iter_num = 100; isPrint = true; }
	SRTSolver(int iter_num_){ iter_num = iter_num_; isPrint = true; }
	//void SetInput(
	//	const std::vector<std::pair<Vec3d, Vec3d>> &matches_,
	//	const Camera &cam1,
	//	const Camera &cam2){
	//	matches.resize(matches_.size());
	//	for (int i = 0; i < matches_.size(); ++i){
	//		matches[i] = std::make_pair(Eigen::Vector3d(matches_[i].first[0], matches_[i].first[1], matches_[i].first[2]),
	//			Eigen::Vector3d(matches_[i].second[0], matches_[i].second[1], matches_[i].second[2]));
	//	}
	//	//std::copy(matches_.begin(), matches_.end(), std::back_inserter(matches));
	//	this->cam1 = cam1;
	//	this->cam2 = cam2;
	//}
	void SetInput(
		const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &matches_,
		const Camera &cam1,
		const Camera &cam2){
		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>().swap(matches);
		std::copy(matches_.begin(), matches_.end(), std::back_inserter(matches));
		this->cam1 = cam1;
		this->cam2 = cam2;
	}
	void SetIterationNum(int iter){ iter_num = iter; }
	void SetPrintFlag(bool isPrint_){ isPrint = isPrint_; }
	void EstimateTransform(double &scale, double R[3][3], double t[3]);
	void EstimateTransform(double &scale, Eigen::Matrix3d &R, Eigen::Vector3d &t);
	double ResidualError(const double scale, Eigen::Matrix3d &R, Eigen::Vector3d &t);
private:
	double EstimateScale();
	double EstimateScaleRansac(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &matches_);
	void EstimateRT(const double scale, Eigen::Matrix3d &R, Eigen::Vector3d &t);
	void EstimateRTRansac(const double scale, Eigen::Matrix3d &R, Eigen::Vector3d &t);
	void EstimateSRTRansac(double &scale, Eigen::Matrix3d &R, Eigen::Vector3d &t);
private:
	bool isPrint;
	int iter_num;
	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> matches;
	Camera cam1;
	Camera cam2;
};

#endif