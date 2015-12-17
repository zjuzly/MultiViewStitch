#ifndef DEPTH2MODEL_H
#define DEPTH2MODEL_H
#include <iostream>
#include <vector>
#include <string>
#include "../Camera/Camera.h"

class Depth2Model{
public:
	Depth2Model() : m_fMinDepth(0.0), m_fMaxDepth(0.0), m_fSmoothThreshold(0.0){}
	Depth2Model(double fMinDepth, double fMaxDepth, double fSmoothThreshold);
	void SaveModel(
		const std::vector<double> &depth,
		const Camera &cam,
		const std::string desPath = "",
		const bool writeMesh = false);
public:
	std::vector<Eigen::Vector3d> point3d;
	std::vector<int> facets;
	std::vector<int> texIndex;
private:
	double m_fMinDepth;
	double m_fMaxDepth;
	double m_fSmoothThreshold;

};

#endif