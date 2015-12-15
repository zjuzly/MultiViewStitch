#ifndef POINTSET_UTILS
#define POINTSET_UTILS
#include <Eigen/Dense>
#include <vector>
#include <iterator>
#include <iostream>

class PointSetUtils{
public:
	void SetInput(const std::vector<Eigen::Vector3d> &points_);
	void CalcPivots(Eigen::MatrixXd &pivots, int pivot_num);

	void CalcBarycenter();
	void CalcBoundingBox();

	Eigen::Vector3d GetBarycenter(){ return baryCenter; }
	Eigen::Vector3d GetBoundingBoxCenter(){ return (leftTop + rightBot) * 0.5; }

private:
	std::vector<Eigen::Vector3d> points;
	Eigen::Vector3d baryCenter;
	Eigen::Vector3d leftTop, rightBot;
};

#endif