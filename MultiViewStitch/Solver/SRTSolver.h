#ifndef SRTSOLVER_H
#define SRTSOLVER_H
#include <time.h>
#include <vector>
#include <Eigen/Dense>
#include "../Camera/Camera.h"

class SRTSolver{
public:
	SRTSolver(){ iter_num = 100; isPrint = true; }
	SRTSolver(int iter_num_){ iter_num = iter_num_; isPrint = true; }
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
	void EstimateTransformRansac(double &scale, Eigen::Matrix3d &R, Eigen::Vector3d &t);
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