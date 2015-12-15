#ifndef ALIGNMENT_H
#define ALIGNMENT_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>

class Alignment{
public:
	Eigen::Vector3d RemoveGround(
		std::vector<Eigen::Vector3d> &points, 
		std::vector<Eigen::Vector3d> &normals = std::vector<Eigen::Vector3d>(), 
		std::vector<int> &facets = std::vector<int>());
private:
	void InitAlignment(
		const std::vector<Eigen::Vector3d> &src,
		const std::vector<Eigen::Vector3d> &s_normals,
		const std::vector<Eigen::Vector3d> &tgt,
		const std::vector<Eigen::Vector3d> &t_normals,
		const Eigen::Vector3d &groundRay,
		const Eigen::Vector3d &viewRay,
		Eigen::Matrix4d &transform,
		double &scale);
	void RetainConnectRegion(
		std::vector<Eigen::Vector3d> &points, 
		std::vector<Eigen::Vector3d> &normals = std::vector<Eigen::Vector3d>(), 
		std::vector<int> &facets = std::vector<int>());
};

#endif