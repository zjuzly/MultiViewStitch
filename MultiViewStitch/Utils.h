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
#include <Eigen/Dense>


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

//String Operation
#pragma region
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
	std::cout << searchPath << std::endl;
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

//Axis Angle
#pragma region
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

#endif