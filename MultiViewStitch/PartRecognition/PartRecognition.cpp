#include "PartRecognition.h"
#include "../SetUtils/PointSetUtils.h"
#include "../common/Utils.h"
#include "../PlyObj/PlyObj.h"
#include <map>

void PartRecognition::LoadParts(const std::string filename){
	using namespace std;
	ifstream ifs;
	ifs.open(filename.c_str(), ifstream::in);
	if(!ifs.is_open()){
		cerr << "ERROR: Open FILE " << filename << " Failed" << endl;
		exit(-1);
	}
#pragma region
	map<string, int> mp;
	mp["Head"]			= HEAD;
	mp["Neck"]			= NECK;
	mp["LeftUpperArm"]	= LEFT_UPPER_ARM;
	mp["LeftLowerArm"]	= LEFT_LOWER_ARM;
	mp["LeftHand"]		= LEFT_HAND;
	mp["RightUpperArm"]	= RIGHT_UPPER_ARM;
	mp["RightLowerArm"]	= RIGHT_LOWER_ARM;
	mp["RightHand"]		= RIGHT_HAND;
	mp["LeftThigh"]		= LEFT_THIGH;
	mp["LeftShank"]		= LEFT_SHANK;
	mp["LeftFoot"]		= LEFT_FOOT;
	mp["RightThigh"]	= RIGHT_THIGH;
	mp["RightShank"]	= RIGHT_SHANK;
	mp["RightFoot"]		= RIGHT_FOOT;
	mp["Truncus"]		= TRUNCUS;
	mp["Hip"]			= HIP;
#pragma endregion
	string line;
	parts.resize(points.size());
	while(ifs.peek() != EOF){
		getline(ifs, line);
		int pos = (int)line.find('=', 0);
		string header = line.substr(0, pos);
		string tmp = line.substr(pos + 1, line.length() - pos - 1);
		vector<string> ret = Split(tmp, ';');
		for(size_t i = 0; i < ret.size(); ++i){
			int vtx = atoi(ret[i].c_str());
			parts[vtx] = mp[header];
		}
	}
	ifs.close();
}

void PartRecognition::PartRecog(const std::vector<Eigen::Vector3d> &points_, std::vector<int> &outParts){
	using namespace std;
	//Create KdTree
	cv::flann::KDTreeIndexParams indexParams;
	cv::Mat cvNodes = cv::Mat::zeros((int)points.size(), 3, CV_32FC1);
	int i, j;
	for(i = 0; i < (int)points.size(); ++i){
		for(j = 0; j < 3; ++j){
			cvNodes.at<float>(i, j) = points[i][j];
		}
	}
	cv::flann::Index kdtree(cvNodes, indexParams);
	//--------------

	outParts.resize(points_.size());
	for(i = 0; i < (int)points_.size(); ++i){
		const Eigen::Vector3f p = points_[i].cast<float>();
		vector<float> query;
		query.push_back(p[0]);
		query.push_back(p[1]);
		query.push_back(p[2]);
		
		vector<int> indices;
		vector<float> dists;
		kdtree.knnSearch(query, indices, dists, 1, cv::flann::SearchParams(200));
		outParts[i] = parts[indices[0]];
	}
}

void Visualization(
	const std::string filename, 
	const std::vector<Eigen::Vector3d> &points,
	const std::vector<int> &parts){
#pragma region
	std::map<int, Eigen::Vector3i> mp;
	mp[HEAD]			= Eigen::Vector3i(0, 0, 255);
	mp[NECK]			= Eigen::Vector3i(128, 0, 0);
	mp[LEFT_UPPER_ARM]	= Eigen::Vector3i(255, 0, 0);
	mp[LEFT_LOWER_ARM]	= Eigen::Vector3i(0, 255, 255);
	mp[LEFT_HAND]		= Eigen::Vector3i(255, 0, 255);
	mp[RIGHT_UPPER_ARM]	= Eigen::Vector3i(255, 0, 0);
	mp[RIGHT_LOWER_ARM]	= Eigen::Vector3i(0, 255, 255);
	mp[RIGHT_HAND]		= Eigen::Vector3i(255, 0, 255);
	mp[LEFT_THIGH]		= Eigen::Vector3i(128, 0, 0);
	mp[LEFT_SHANK]		= Eigen::Vector3i(0, 128, 128);
	mp[LEFT_FOOT]		= Eigen::Vector3i(128, 0, 128);
	mp[RIGHT_THIGH]		= Eigen::Vector3i(128, 0, 0);
	mp[RIGHT_SHANK]		= Eigen::Vector3i(0, 128, 128);
	mp[RIGHT_FOOT]		= Eigen::Vector3i(128, 0, 128);
	mp[TRUNCUS]			= Eigen::Vector3i(128, 255, 0);
	mp[HIP]				= Eigen::Vector3i(255, 255, 0);
#pragma endregion
	std::vector<Eigen::Vector3i> colors(points.size());
	for(int i = 0; i < (int)points.size(); ++i){ 
		colors[i] = mp[parts[i]];
	}
	WriteObj(filename.c_str(), points, colors);
}


void LoadShoulderJoints(
	const std::string filename, 
	std::vector<std::vector<int>> &index){
	using namespace std;
	ifstream ifs;
	ifs.open(filename.c_str(), ifstream::in);
	if(!ifs.is_open()){
		cerr << "ERROR: Open FILE " << filename << " Failed" << endl;
		exit(-1);
	}
	string line;
	index.resize(2);
	while(ifs.peek() != EOF){
		getline(ifs, line);
		int pos = (int)line.find('=', 0);
		string header = line.substr(0, pos);
		string tmp = line.substr(pos + 1, line.length() - pos - 1);
		vector<string> ret = Split(tmp, ';');
		for(size_t i = 0; i < ret.size(); ++i){
			int vtx = atoi(ret[i].c_str());
			if(header == "Left"){
				index[0].push_back(vtx);
			}else if(header == "Right"){
				index[1].push_back(vtx);
			}
		}
	}
	ifs.close();
}

void Test(){
	using namespace std;
	using namespace Eigen;

	vector<Vector3d> points_;
	ReadObj("./data/zly/aligned.obj", points_);
	
	PartRecognition pr;
	pr.SetInput(points_);
	pr.LoadParts("./Template/part/parts");
	Visualization("./parts.ply", points_, pr.GetLabels());
}