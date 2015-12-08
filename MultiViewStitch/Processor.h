#ifndef PROCESSOR_H
#define PROCESSOR_H
#include "Image3D.h"
#include <Eigen/Dense>
#include <vector>
#include <string>

class Processor{
public:
	//void SetParamFromFile(const std::string filename);
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
		const std::vector<std::vector<Camera>> &cameras,
		std::vector<double> &scales,
		std::vector<Eigen::Matrix3d> &Rs,
		std::vector<Eigen::Vector3d> &ts);
private:
	//bool writeMesh;

	//int view_count; //1
	//int min_match_count; //10
	//int iter_num; //100
	//int sample_interval; //40
	//int ssd_win; //7
	//int axis; //x: 0, y: 1, z: 2
	//
	//double rot_angle;
	//double ssd_err;
	//double pixel_err;
	//double distmax; //0.5
	//double ratiomax; //0.5
	//double hl_margin_ratio; //0.15
	//double hr_margin_ratio; //0.15
	//double vl_margin_ratio; //0.15
	//double vr_margin_ratio; //0.15
	//double m_fMinDsp;
	//double m_fMaxDsp;

	//std::vector<std::string> imgdirs;
};

#endif