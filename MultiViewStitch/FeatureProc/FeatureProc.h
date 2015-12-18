#include <opencv2\opencv.hpp>
#include <SiftGPU.h>
#include <string>
#include <vector>
#include <Eigen/Dense>

class FeatureProc{
public:
	static void DetectFeatureSingleView(
		const std::string imgpath,
		std::vector<SiftGPU::SiftKeypoint> &keys, 
		std::vector<float> &descs);

	static void MatchFeatureSingleView(
		const std::vector<SiftGPU::SiftKeypoint> &keys1, 
		const std::vector<SiftGPU::SiftKeypoint> &keys2,
		const std::vector<float> &descs1,
		const std::vector<float> &descs2,
		std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>> &matches);

	static void DetectFeature(
		const std::vector<std::string> imgpaths,
		std::vector<std::vector<SiftGPU::SiftKeypoint>> &keys,
		std::vector<std::vector<float>> &descs);

	static void MatchFeature(
		const std::vector<std::vector<SiftGPU::SiftKeypoint>> &keys1,
		const std::vector<std::vector<SiftGPU::SiftKeypoint>> &keys2,
		const std::vector<std::vector<float>> &descs1,
		const std::vector<std::vector<float>> &descs2,
		std::vector<std::vector<std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>>>> &matches);
};