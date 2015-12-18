#include "DepthOptimizer.h"

void DepthOptimizer::LoadDataEx(){
	cameras.resize(ParamParser::imgdirs.size());
	images.resize(ParamParser::imgdirs.size());
	for (int k = 0; k < cameras.size(); ++k){
		cameras[k].clear();
		std::vector<std::string> actsfiles = ScanNSortDirectory(ParamParser::imgdirs[k].c_str(), "act");
		cameras[k] = LoadCalibrationFromActs(actsfiles[0]);
		images[k].resize(cameras[k].size());
		char imgpath[128];
		for (int i = 0; i < images[k].size(); ++i){
			sprintf_s(imgpath, "%s%05d.jpg", ParamParser::imgdirs[k].c_str(), i);
			images[k][i] = cv::imread(imgpath);
		}
	}
}

void DepthOptimizer::RefineAllDepthMaps(){
	LoadDataEx();
	for (int k = 0; k < ParamParser::imgdirs.size(); ++k){
		char fn[128];
		CreateDir(ParamParser::imgdirs[k] + "DATA/Refine/");

		std::vector<double> dsp;
		int n = cameras[k].size();
		for (int i = 0; i < n; ++i){
			sprintf_s(fn, "%sDATA/Render/_depth%d.raw", ParamParser::imgdirs[k].c_str(), i);
			LoadDepth(fn, dsp, cameras[k][i].W(), cameras[k][i].H());
			
			RefineDepthMap(k, i, dsp);

			sprintf_s(fn, "%sDATA/Refine/_depth%d.jpg", ParamParser::imgdirs[k].c_str(), i);
			RenderDepthMap(fn, dsp, cameras[k][i].W(), cameras[k][i].H());
			sprintf_s(fn, "%sDATA/Refine/_depth%d.raw", ParamParser::imgdirs[k].c_str(), i);
			SaveDepth(fn, dsp);
		}
	}
}

void DepthOptimizer::RefineDepthMap(
	const int seqIdx,
	const int frmIdx,
	std::vector<double> &depth){
	const static int refFrmCount = 5;
	std::cout << "Seq#" << seqIdx <<" | Processing Frame";
	for (int i = 0; i < refFrmCount; ++i){
		int idx = frmIdx - refFrmCount / 2 + i;
		if (idx >= 0 && idx < cameras[seqIdx].size()){
			std::cout << " " << idx;
		}
	}
	std::cout << "..." << std::endl;

	std::vector<Camera> refcams;
	std::vector<cv::Mat> refimgs;
	for (int i = 0; i < refFrmCount; ++i){
		int idx = frmIdx - refFrmCount / 2 + i;
		if (idx >= 0 && idx < cameras[seqIdx].size() && idx != frmIdx){
			refcams.push_back(cameras[seqIdx][idx]);
			refimgs.push_back(images[seqIdx][idx].clone());
		}
	}
	Camera curcam = cameras[seqIdx][frmIdx];
	cv::Mat curimg = images[seqIdx][frmIdx];
}