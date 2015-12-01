#ifndef PROCESSOR_H
#define PROCESSOR_H
#include "Image3D.h"
#include <Eigen/Dense>
#include <vector>
#include <string>

class Processor{
public:
	void SetParamFromFile(const std::string filename);
	void AlignmentSeq();
private:
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
		std::vector<double> &scales,
		std::vector<Eigen::Matrix3d> &Rs,
		std::vector<Eigen::Vector3d> &ts);
private:
	int view_count; //1
	int min_match_count; //10
	int iter_num; //100
	int sample_interval; //40
	int ssd_win; //7

	double pixel_err;
	double threshold; //0.5
	double distmax; //0.5
	double ratiomax; //0.5
	double h_margin_ratio; //0.15
	double v_margin_ratio; //0.15

	std::vector<std::string> imgdirs;
};

#endif