#ifndef UTILS_H
#define UTILS_H
#include <io.h>
#include <direct.h>
#include <Windows.h>
#include <tchar.h>
#include <time.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <opencv2\opencv.hpp>


inline bool CheckRange(const int u, const int v, const int w, const int h){
	return (u >= 0 && u < w && v >= 0 && v < h);
}

//从0 - n-1中随机选出m个元素
static void Shuffle(int *k, int n, int m){
	int i, j, j0, r;
	for (i = 0; i < m; i++) {
		r = (rand()) % (n - i);
		for (j = 0; j < i && r >= k[j]; j++) r++;
		j0 = j;
		for (j = i; j > j0; j--) k[j] = k[j - 1];
		k[j0] = r;
	}
}

#pragma region//String Operation
static std::vector<std::string> Split(const std::string &line, const char del = ' '){
	std::vector<std::string> ret;
	int pos = 0;
	while (pos < line.length()){
		int newPos = (int)line.find_first_of(del, pos);
		if (newPos == -1){
			ret.push_back(line.substr(pos, line.size() - pos));
			break;
		}
		ret.push_back(line.substr(pos, newPos - pos));
		pos = newPos + 1;
	}
	return ret;
}
static std::vector<std::string> ScanNSortDirectory(const char* path, const char* ext){
	WIN32_FIND_DATA wfd;
	HANDLE hHandle;
	std::string searchPath, searchFile;
	std::vector<std::string> vFilenames;
	int nbFiles = 0;

	searchPath = std::string(path) + "*." + std::string(ext);
	//std::cout << searchPath << std::endl;
	hHandle = FindFirstFile(_T(searchPath.c_str()), &wfd);

	if (INVALID_HANDLE_VALUE == hHandle){
		fprintf(stderr, "ERROR(%s, %d): Cannot find (*.%s)files in directory %s\n",
			__FILE__, __LINE__, ext, path);
		exit(0);
	}
	do{
		if (wfd.cFileName[0] == '.')
			continue;
		if (wfd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
			continue;
		else{
			searchFile = std::string(path)/* + "/"*/ + std::string((char *)wfd.cFileName);
			vFilenames.push_back(searchFile);
			nbFiles++;
		}
	} while (FindNextFile(hHandle, &wfd));

	FindClose(hHandle);

	//qsort((void *)&(vFilenames[0]), (size_t)nbFiles, sizeof(string), str_compare);
	return vFilenames;
}
static int CreateDir(std::string dir){
	int iRet;
	int len;
	if (dir.length() == 0)	return 0;
	len = (int)dir.length();
	for (int i = 0; i < len; ++i){
		if (dir[i] == '\\' || dir[i] == '/'){
			dir[i] = '\0';
			iRet = _access(dir.c_str(), 0);
			if (iRet != 0){
				iRet = _mkdir(dir.c_str());
				if (iRet != 0){
					return -1;
				}
			}
			dir[i] = '/';
		}
	}
	//iRet = _mkdir(dir.c_str());
	if (iRet != 0){
		std::cerr << "Can not create directory " << dir << std::endl;
		exit(-1);
	}
	return iRet;
}
#pragma endregion

#pragma region//Axis Angle
static double DotProduct(const Eigen::Vector3d &a, const Eigen::Vector3d &b){
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static Eigen::Vector3d CrossProduct(const Eigen::Vector3d &a, const Eigen::Vector3d &b){
	Eigen::Vector3d c;
	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];
	return c;
}

static void RotationMatrix(const double angle, const Eigen::Vector3d &u, Eigen::Matrix3d &R){
	double cosine = cos(angle);
	double sine = sin(angle);
	R(0, 0) = cosine + u[0] * u[0] * (1 - cosine);
	R(0, 1) = u[0] * u[1] * (1 - cosine) - u[2] * sine;
	R(0, 2) = u[1] * sine + u[0] * u[2] * (1 - cosine);

	R(1, 0) = u[2] * sine + u[0] * u[1] * (1 - cosine);
	R(1, 1) = cosine + u[1] * u[1] * (1 - cosine);
	R(1, 2) = -u[0] * sine + u[1] * u[2] * (1 - cosine);

	R(2, 0) = -u[1] * sine + u[0] * u[2] * (1 - cosine);
	R(2, 1) = u[0] * sine + u[1] * u[2] * (1 - cosine);
	R(2, 2) = cosine + u[2] * u[2] * (1 - cosine);
}

static void CalcRotation(const Eigen::Vector3d &vecBefore, const Eigen::Vector3d &vecAfter, Eigen::Matrix3d &R){
	double rotationAngle;
	Eigen::Vector3d vecBefore_n = vecBefore.normalized();
	Eigen::Vector3d vecAfter_n = vecAfter.normalized();
	double value = vecBefore_n.dot(vecAfter_n);
	rotationAngle = acos(value); //acos(__min(1.0, value));
	Eigen::Vector3d u = vecBefore_n.cross(vecAfter_n);
	u.normalize();
	RotationMatrix(rotationAngle, u, R);
}

static void AxisAngleTransform(const Eigen::Vector3d axis, const double angle, const Eigen::Vector3d pb, Eigen::Vector3d &pa){
	if (angle <= 1e-9 && angle >= -1e-9){ pa = pb; return; }
	double radian = (angle / 180.0) * M_PI;
	double cosine = cos(radian);
	double sine = sin(radian);
	double len = axis.norm();
	Eigen::Vector3d n;
	n[0] = axis[0] / len; n[1] = axis[1] / len; n[2] = axis[2] / len;
	pa = pb * cosine + n * DotProduct(n, pb) * (1 - cosine)
		+ CrossProduct(pb, n) * sine;
}

#pragma endregion

#pragma region//Serialization
static void LoadDepth(const std::string filename, std::vector<double> &depth, int w, int h){
	//std::vector<double>().swap(depth);
	depth.clear();
	depth.resize(w * h);
	float *raw = new float[w * h];
	std::ifstream ifs(filename.c_str(), std::ifstream::in | std::ifstream::binary);
	ifs.read((char*)raw, w * h * sizeof(float));
	for (int i = 0; i < w * h; ++i){ depth[i] = raw[i]; }
	ifs.close();
	delete[]raw;
}
static void SaveDepth(const std::string filename, std::vector<double> &depth){
	std::ofstream ofs(filename.c_str(), std::ofstream::out | std::ofstream::binary);
	int size = depth.size();
	float *pData = new float[size];
	for (int i = 0; i < size; ++i){ pData[i] = depth[i]; }
	ofs.write((char*)pData, sizeof(float)* size);
	ofs.close();
	delete[]pData;
}
#pragma endregion

#pragma region //Render View
static void RenderDepthMap(
	const std::string filename,
	const std::vector<double> &depth,
	int w, int h
	){
	if (depth.size() <= 0)	return;

	double d_min = HUGE_VAL;
	double d_max = 1 - HUGE_VAL;
	for (int i = 0; i < depth.size(); ++i){
		if (depth[i] >= 0){
			d_min = __min(d_min, depth[i]);
			d_max = __max(d_max, depth[i]);
		}
	}
	cv::Mat gray(h, w, CV_8UC1);
	for (int j = 0; j < h; ++j){
		for (int i = 0; i < w; ++i){
			if (depth[j * w + i] >= 0){
				uchar val = uchar(255 * (depth[j * w + i] - d_min) / (d_max - d_min));
				gray.at<uchar>(j, i) = val;
			}
			else{
				gray.at<uchar>(j, i) = 255;
			}
		}
	}
	cv::imwrite(filename, gray);
}
#pragma endregion

#pragma region//Image Operation
static double SSD(
	const cv::Mat &img1, const int u1, const int v1,
	const cv::Mat &img2, const int u2, const int v2,
	const int N = 3){
	cv::Rect roi1 = cv::Rect(u1 - N, v1 - N, 2 * N + 1, 2 * N + 1);
	cv::Mat roiMat1(img1, roi1);
	cv::Mat roiMatGray1(roi1.height, roi1.width, CV_8UC1);
	cv::cvtColor(roiMat1, roiMatGray1, cv::COLOR_RGB2GRAY);
	cv::Rect roi2 = cv::Rect(u2 - N, v2 - N, 2 * N + 1, 2 * N + 1);
	cv::Mat roiMat2(img2, roi2);
	cv::Mat roiMatGray2(roi2.height, roi2.width, CV_8UC1);
	cv::cvtColor(roiMat2, roiMatGray2, cv::COLOR_RGB2GRAY);

	double sum = 0.0;
	int length = 2 * N + 1;
	for (int i = 0; i < length; ++i){
		for (int j = 0; j < length; ++j){
			sum += double(roiMatGray1.at<uchar>(i, j) - roiMatGray2.at<uchar>(i, j)) * double(roiMatGray1.at<uchar>(i, j) - roiMatGray2.at<uchar>(i, j));
		}
	}
	return sqrt(sum / (length * length));
	//if (type == CV_32FC1){
	//	for (int i = 0; i < length; ++i){
	//		for (int j = 0; j < length; ++j){
	//			sum += double(roiMat1.at<uchar>(i, j) - roiMat2.at<uchar>(i, j)) * double(roiMat1.at<uchar>(i, j) - roiMat2.at<uchar>(i, j));
	//		}
	//	}
	//	return sqrt(sum / (length * length));
	//}
	//else{
	//	for (int i = 0; i < length; ++i){
	//		for (int j = 0; j < length; ++j){
	//			cv::Vec3b bgr1 = roiMat1.at<cv::Vec3b>(i, j);
	//			cv::Vec3b bgr2 = roiMat2.at<cv::Vec3b>(i, j);
	//			sum += double(bgr1[0] - bgr2[0]) * double(bgr1[0] - bgr2[0]);
	//			sum += double(bgr1[1] - bgr2[1]) * double(bgr1[1] - bgr2[1]);
	//			sum += double(bgr1[2] - bgr2[2]) * double(bgr1[2] - bgr2[2]);
	//		}
	//	}
	//	return sqrt(sum / (length * length * 3.0));
	//}
}

//debugging
static double NCC(
	const cv::Mat &img1, const int u1, const int v1,
	const cv::Mat &img2, const int u2, const int v2,
	const int &N){
	//cv::Rect roi1 = cv::Rect(u1 - N, v1 - N, 2 * N + 1, 2 * N + 1);
	//cv::Mat roiMat1(img1, roi1);
	//cv::Mat roiMatGray1(roi1.height, roi1.width, CV_8UC1);
	//cv::cvtColor(roiMat1, roiMatGray1, cv::COLOR_RGB2GRAY);
	//cv::Mat f1 = cv::Mat(roi1.height, roi1.width, CV_32FC1);
	//roiMatGray1.convertTo(f1, CV_32FC1);
	//float s1(0.0f);
	//for (int i = 0; i < f1.rows; ++i){
	//	for (int j = 0; j < f1.cols; ++j){
	//		s1 += f1.at<float>(i, j);
	//	}
	//}
	//s1 /= (f1.rows * f1.cols);
	//float sum1(0.0f);
	//for (int i = 0; i < f1.rows; ++i){
	//	for (int j = 0; j < f1.cols; ++j){
	//		float &p = f1.at<float>(i, j);
	//		p -= s1;
	//		sum1 += p * p;
	//	}
	//}
	//sum1 = sqrt(sum1);
	//for (int i = 0; i < f1.rows; ++i){
	//	for (int j = 0; j < f1.cols; ++j){
	//		float &p = f1.at<float>(i, j);
	//		p /= sum1;
	//	}
	//}

	//cv::Rect roi2 = cv::Rect(u2 - N, v2 - N, 2 * N + 1, 2 * N + 1);
	//cv::Mat roiMat2(img2, roi2);
	//cv::Mat roiMatGray2(roi2.height, roi2.width, CV_8UC1);
	//cv::cvtColor(roiMat2, roiMatGray2, cv::COLOR_RGB2GRAY);
	//cv::Mat f2 = cv::Mat(roi2.height, roi2.width, CV_32FC1);
	//roiMatGray2.convertTo(f2, CV_32FC1);
	//float s2(0.0f);
	//for (int i = 0; i < f2.rows; ++i){
	//	for (int j = 0; j < f2.cols; ++j){
	//		s2 += f2.at<float>(i, j);
	//	}
	//}
	//s2 /= (f2.rows * f2.cols);

	//float sum2(0.0f);
	//for (int i = 0; i < f2.rows; ++i){
	//	for (int j = 0; j < f2.cols; ++j){
	//		float &p = f2.at<float>(i, j);
	//		p -= s2;
	//		sum2 += p * p;
	//	}
	//}
	//sum2 = sqrt(sum2);
	//for (int i = 0; i < f2.rows; ++i){
	//	for (int j = 0; j < f2.cols; ++j){
	//		float &p = f2.at<float>(i, j);
	//		p /= sum2;
	//	}
	//}
	//double corr = 0.0, corrsq = 0.0;
	//for (int i = 0; i < f2.rows; ++i){
	//	for (int j = 0; j < f2.cols; ++j){
	//		float g1 = f1.at<float>(i, j);
	//		float g2 = f2.at<float>(i, j);
	//		corr = corr + fabs(g1 * g2);
	//		corrsq = corrsq + (g1 * g1 * g2 * g2);
	//	}
	//}
	//return (corr / sqrt(corrsq)) / (roi1.height * roi1.width);
}

#pragma endregion
#endif