#include <opencv2\opencv.hpp>
#include <SiftGPU.h>
#include <string>
#include <vector>
#include <Eigen/Dense>

class FeatureProc{
public:
	static void DetectFeatureSingleView(
		const std::string imgpath,
		const double h_margin_ratio,
		const double v_margin_ratio,
		std::vector<SiftGPU::SiftKeypoint> &keys, 
		std::vector<float> &descs);

	static void MatchFeatureSingleView(
		const std::vector<SiftGPU::SiftKeypoint> &keys1, 
		const std::vector<SiftGPU::SiftKeypoint> &keys2,
		const std::vector<float> &descs1,
		const std::vector<float> &descs2,
		const double distmax,
		const double ratiomax,
		std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>> &matches);

	static void DetectFeature(
		const std::vector<std::string> imgpaths,
		const double h_margin_ratio,
		const double v_margin_ratio,
		std::vector<std::vector<SiftGPU::SiftKeypoint>> &keys,
		std::vector<std::vector<float>> &descs);

	static void MatchFeature(
		const std::vector<std::vector<SiftGPU::SiftKeypoint>> &keys1,
		const std::vector<std::vector<SiftGPU::SiftKeypoint>> &keys2,
		const std::vector<std::vector<float>> &descs1,
		const std::vector<std::vector<float>> &descs2,
		const double distmax,
		const double ratiomax,
		std::vector<std::vector<std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>>>> &matches);

	static double SSD(
		const cv::Mat &img1, const int u1, const int v1,
		const cv::Mat &img2, const int u2, const int v2,
		const int N = 3);
};