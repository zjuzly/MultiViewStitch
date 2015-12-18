#ifndef DEPTH_OPTIMIZER_H
#define DEPTH_OPTIMIZER_H

#include <iostream>
#include <vector>
#include <opencv2\opencv.hpp>
#include "../Camera/Camera.h"
#include "../Common/Utils.h"
#include "../Parameter/ParamParser.h"

class DepthOptimizer{
public:
	DepthOptimizer(){}
	void RefineAllDepthMaps();

private:
	void RefineDepthMap(
		const int seqIdx,
		const int frmIdx,
		std::vector<double> &dsp);
	void DepthRefineCore(
		const Camera &curcam,
		const std::vector<Camera> &refcams,
		const cv::Mat &curimg,
		const std::vector<cv::Mat> &refimgs,
		const int u,
		const int v,
		double dspEst);
	void LoadDataEx();
private:
	std::vector<std::vector<Camera>> cameras;
	std::vector<std::vector<cv::Mat>> images;
};

#endif