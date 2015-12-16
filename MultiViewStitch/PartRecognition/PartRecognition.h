#ifndef PART_RECOGNITION_H
#define PART_RECOGNITION_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <iterator>

#ifndef ENUM_PART
#define ENUM_PART
enum PART{
	HEAD,				//0
	NECK,				//1
	LEFT_UPPER_ARM,		//2
	LEFT_LOWER_ARM,		//3
	LEFT_HAND,			//4
	RIGHT_UPPER_ARM,	//5
	RIGHT_LOWER_ARM,	//6
	RIGHT_HAND,			//7
	LEFT_THIGH,			//8
	LEFT_SHANK,			//9
	LEFT_FOOT,			//10
	RIGHT_THIGH,		//11
	RIGHT_SHANK,		//12
	RIGHT_FOOT,			//13
	TRUNCUS,			//14
	HIP					//15
};
#endif

class PartRecognition{
public:
	void SetInput(const std::vector<Eigen::Vector3d> &points_){
		std::vector<Eigen::Vector3d>().swap(points);
		std::copy(points_.begin(), points_.end(), std::back_inserter(points));
	}
	void LoadParts(const std::string filename);
	
	std::vector<Eigen::Vector3d>& GetPoints(){ return points; }
	std::vector<int>& GetLabels(){ return parts; }

	void PartRecog(const std::vector<Eigen::Vector3d> &points_, std::vector<int> &outParts);

protected:
	std::vector<Eigen::Vector3d> points;
	std::vector<int> parts;
};

void LoadShoulderJoints(
	const std::string filename, 
	std::vector<std::vector<int>> &index);
void Visualization(
	const std::string filename,  
	const std::vector<Eigen::Vector3d> &points, 
	const std::vector<int> &parts);
void Test();

#endif