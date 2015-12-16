#ifndef PROCESSOR_H
#define PROCESSOR_H
#include "../Image3D/Image3D.h"
#include <Eigen/Dense>
#include <vector>
#include <string>

class Processor{
public:
	void AlignmentSeq();
	void CheckConsistency(const std::vector<std::vector<Camera>> &cameras);
private:
	/**
	*/
	void CheckConsistencyCore(
		const Camera curcam,
		const std::vector<Camera> refcams,
		std::vector<double> &depth_out,
		const std::vector<std::vector<double>> &refdepth);
	void RemoveDupPoints(
		const std::vector<Image3D> &im,
		const std::vector<Image3D> &jm,
		std::vector<std::vector<std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>>>> &matches);
	void RemoveOutliers(
		const Image3D &im,
		const Image3D &jm,
		double &inlier_ratio_out,
		double &err_out,
		std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>> &matches);
	/**
	* This method is in the process of being tested
	*/
	double RemoveOutliersParsac(
		const Image3D &im,
		const Image3D &jm,
		std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>> &matches);
	/**
	* This method is  deprecated
	*/
	void CalcSimilarityTransformation(
		double &scale,
		double R[3][3],
		double t[3]);
	void CalcSimilarityTransformationSeq(
		const std::vector<std::vector<Camera>> &cameras,
		std::vector<double> &scales,
		std::vector<Eigen::Matrix3d> &Rs,
		std::vector<Eigen::Vector3d> &ts,
		std::vector<std::pair<int, int>> &selectFrames);
private:
	std::vector<std::vector<Image3D>> models;
};

#endif