#include "FeatureProc.h"
#include "Utils.h"
#include "ParamParser.h"
#include <GL\glew.h>

#if _DEBUG
#pragma comment(lib, "E:/3rdPart/SiftGPU/lib/SIFTGPU_d.lib")
#else
#pragma comment(lib, "E:/3rdPart/SiftGPU/lib/SIFTGPU.lib")
#endif
#pragma comment(lib, "E:/3rdPart/SiftGPU/lib/glut64.lib")
#pragma comment(lib, "E:/3rdPart/SiftGPU/lib/glew64.lib")

void FeatureProc::DetectFeatureSingleView(
	const std::string imgpath,
	std::vector<SiftGPU::SiftKeypoint> &keys, 
	std::vector<float> &descs){
	SiftGPU *sift = new SiftGPU;
	int num = 0;
	char * argv[] = { "-fo", "-1", "-v", "0" };//
	int argc = sizeof(argv) / sizeof(char*);
	sift->ParseParam(argc, argv);

	if (sift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED) return ;

	cv::Mat img = cv::imread(imgpath);
	cv::Mat img1 = img.clone();
	int rowLeftInterval = img.rows * ParamParser::vl_margin_ratio;
	int colLeftInterval = img.cols * ParamParser::hl_margin_ratio;
	int rowRightInterval = img.rows * ParamParser::vr_margin_ratio;
	int colRightInterval = img.cols * ParamParser::hr_margin_ratio;
	for (int i = 0; i < rowLeftInterval; ++i){
		img.row(i).setTo(cv::Scalar(0, 0, 0));
	}
	for (int i = 0; i < colLeftInterval; ++i){
		img.col(i).setTo(cv::Scalar(0, 0, 0));
	}
	for (int i = 0; i < rowRightInterval; ++i){
		img.row(i + img.rows - rowRightInterval).setTo(cv::Scalar(0, 0, 0));
	}
	for (int i = 0; i < colRightInterval; ++i){
		img.col(i + img.cols - colRightInterval).setTo(cv::Scalar(0, 0, 0));
	}
	uchar* data = (uchar*)img.data;

	if (sift->RunSIFT(img.cols, img.rows, (void*)data, GL_BGR, GL_UNSIGNED_BYTE)){
		num = sift->GetFeatureNum();
		keys.resize(num);    descs.resize(128 * num);
		sift->GetFeatureVector(&keys[0], &descs[0]);      
	}
	delete sift;
#if 1
	int left = colLeftInterval, right = img.cols - colRightInterval;
	int top = rowLeftInterval, bottom = img.rows - rowRightInterval;
	int newNum = 0;
	for (int k = 0; k < keys.size(); ++k){
		if (keys[k].x < left || keys[k].x > right || keys[k].y < top || keys[k].y > bottom) continue;
		keys[newNum] = keys[k];
		for (int i = 0; i < 128; ++i){
			descs[newNum * 128 + i] = descs[k * 128 + i];
		}
		++newNum;
	}
	keys.resize(newNum);
	descs.resize(newNum * 128);
#endif
	for (const auto & kp : keys){
		cv::circle(img1, cv::Point2f(kp.x, kp.y), 1, cv::Scalar(0, 255, 0), -1);
	}
	int pos = imgpath.find_last_of('/');
	std::string siftpath = imgpath.substr(0, pos + 1) + "Sift/" + imgpath.substr(pos + 1, imgpath.size() - pos - 1);
	cv::imshow("Sift", img1);
	cv::imwrite(siftpath, img1);
	cv::waitKey(1);
}

void FeatureProc::MatchFeatureSingleView(
	const std::vector<SiftGPU::SiftKeypoint> &keys1,
	const std::vector<SiftGPU::SiftKeypoint> &keys2,
	const std::vector<float> &descs1,
	const std::vector<float> &descs2,
	std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>> &matches){
	SiftMatchGPU *matcher = new SiftMatchGPU(4096);
	matcher->VerifyContextGL();

	matcher->SetDescriptors(0, keys1.size(), &descs1[0]);
	matcher->SetDescriptors(1, keys2.size(), &descs2[0]);

	int(*match_buf)[2] = new int[keys1.size()][2];
	int num_match = matcher->GetSiftMatch(keys1.size(), match_buf, ParamParser::distmax, ParamParser::ratiomax);
	//std::cout << num_match << " sift matches were found;" << std::endl;

	matches.resize(num_match);
	for (int i = 0; i < num_match; ++i){
		const SiftGPU::SiftKeypoint & key1 = keys1[match_buf[i][0]];
		const SiftGPU::SiftKeypoint & key2 = keys2[match_buf[i][1]];
		matches[i] = std::make_pair(Eigen::Vector2i(int(key1.x + 0.5), int(key1.y + 0.5)), Eigen::Vector2i(int(key2.x + 0.5), int(key2.y + 0.5)));
	}

	delete[]match_buf;
	delete matcher;
}

void FeatureProc::DetectFeature(
	const std::vector<std::string> imgpaths,
	std::vector<std::vector<SiftGPU::SiftKeypoint>> &keys,
	std::vector<std::vector<float>> &descs){
	keys.resize(imgpaths.size());
	descs.resize(imgpaths.size());
	for (int i = 0; i < (int)imgpaths.size(); ++i){
		DetectFeatureSingleView(imgpaths[i], keys[i], descs[i]);
	}
}

void FeatureProc::MatchFeature(
	const std::vector<std::vector<SiftGPU::SiftKeypoint>> &keys1,
	const std::vector<std::vector<SiftGPU::SiftKeypoint>> &keys2,
	const std::vector<std::vector<float>> &descs1,
	const std::vector<std::vector<float>> &descs2,
	std::vector<std::vector<std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>>>> &matches){
	int m1 = keys1.size();
	int m2 = keys2.size();
	matches.resize(m1, std::vector<std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>>>(m2));
	for (int i = 0; i < m1; ++i){
		for (int j = 0; j < m2; ++j){
			//std::cout << "View# " << i << ", " << j << "..." << std::endl;
			std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>> matches_;
			MatchFeatureSingleView(keys1[i], keys2[j], descs1[i], descs2[j], matches_);
			matches[i][j] = matches_;
		}
	}
}

double FeatureProc::SSD(
	const cv::Mat &img1, const int u1, const int v1,
	const cv::Mat &img2, const int u2, const int v2,
	const int N){
	int type = CV_32FC1;
	if (img1.channels() == 3){ type = CV_32FC3; }
	else if (img1.channels() == 1){ type = CV_32FC1; }

	cv::Rect roi1 = cv::Rect(u1 - N, v1 - N, 2 * N + 1, 2 * N + 1);
	cv::Mat roiMat1(img1, roi1);
	cv::Rect roi2 = cv::Rect(u2 - N, v2 - N, 2 * N + 1, 2 * N + 1);
	cv::Mat roiMat2(img2, roi2);

	double sum = 0.0;
	int length = 2 * N + 1;
	if (type == CV_32FC1){
		for (int i = 0; i < length; ++i){
			for (int j = 0; j < length; ++j){
				sum += double(roiMat1.at<uchar>(i, j) - roiMat2.at<uchar>(i, j)) * double(roiMat1.at<uchar>(i, j) - roiMat2.at<uchar>(i, j));
			}
		}
		return sqrt(sum / (length * length));
	}
	else{
		for (int i = 0; i < length; ++i){
			for (int j = 0; j < length; ++j){
				cv::Vec3b bgr1 = roiMat1.at<cv::Vec3b>(i, j);
				cv::Vec3b bgr2 = roiMat2.at<cv::Vec3b>(i, j);
				sum += double(bgr1[0] - bgr2[0]) * double(bgr1[0] - bgr2[0]);
				sum += double(bgr1[1] - bgr2[1]) * double(bgr1[1] - bgr2[1]);
				sum += double(bgr1[2] - bgr2[2]) * double(bgr1[2] - bgr2[2]);
			}
		}
		return sqrt(sum / (length * length * 3.0));
	}
}