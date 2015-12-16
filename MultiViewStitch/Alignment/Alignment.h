#ifndef ALIGNMENT_H
#define ALIGNMENT_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>

class Alignment{
public:
#pragma region
	struct Node{
		double d;
		int index;
		Node(double d_, int idx) : d(d_), index(idx){}
		bool operator < (const Node &other) const {
			return d < other.d;
		}
	};
#pragma endregion
public:
	void Alignment::Align(
		std::vector<Eigen::Vector3d> &src,
		std::vector<Eigen::Vector3d> &s_normals,
		std::vector<int> &s_facets,
		std::vector<Eigen::Vector3d> &tgt,
		std::vector<Eigen::Vector3d> &t_normals,
		std::vector<int> &t_facets,
		const Eigen::Vector3d &viewRay);
	Eigen::Vector3d RemoveGround(
		std::vector<Eigen::Vector3d> &points, 
		std::vector<Eigen::Vector3d> &normals = std::vector<Eigen::Vector3d>(), 
		std::vector<int> &facets = std::vector<int>());
	void RetainConnectRegion(
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
	void LocalAlignment(
		std::vector<Eigen::Vector3d> &src,
		std::vector<Eigen::Vector3d> &s_normals,
		const std::vector<int> &s_labels,
		const std::vector<Eigen::Vector3d> &tgt,
		const std::vector<Eigen::Vector3d> &t_normals,
		const std::vector<int> &t_labels);
	void LocalAlignmentCore(
		std::vector<Eigen::Vector3d> &src_,
		const std::vector<int> &s_labels,
		const std::vector<Eigen::Vector3d> &tgt_,
		const std::vector<int> &t_labels,
		const int slabel,
		const int tlabel,
		Eigen::Matrix4d &transform,
		double &scale);
	void AlignByShoulder(
		std::vector<Eigen::Vector3d> &src,
		const std::vector<Eigen::Vector3d> &s_normals,
		const std::vector<int> &slabels,
		const std::vector<Eigen::Vector3d> &tgt,
		const std::vector<int> &tlabels);
	void ModelScaling(
		std::vector<Eigen::Vector3d> &points,
		const std::vector<int> &labels,
		const double scale);
};

#endif