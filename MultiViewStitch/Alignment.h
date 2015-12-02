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
	void RetainConnectRegion(
		std::vector<Eigen::Vector3d> &points, 
		std::vector<Eigen::Vector3d> &normals = std::vector<Eigen::Vector3d>(), 
		std::vector<int> &facets = std::vector<int>());
};

#endif