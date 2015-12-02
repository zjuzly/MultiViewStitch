#include "Alignment.h"
#include "UnionSetUtils.h"
#include <unordered_map>

Eigen::Vector3d Alignment::RemoveGround(
	std::vector<Eigen::Vector3d> &points,
	std::vector<Eigen::Vector3d> &normals,
	std::vector<int> &facets){
	RetainConnectRegion(points, normals, facets);
	return Eigen::Vector3d::Identity();
}

void Alignment::RetainConnectRegion(
	std::vector<Eigen::Vector3d> &points,
	std::vector<Eigen::Vector3d> &normals,
	std::vector<int> &facets){
	UnionSet us(points.size());
	for (int i = 0; i < (int)facets.size(); i += 3){
		us.Merge(facets[i], facets[i + 1]);
		us.Merge(facets[i], facets[i + 2]);
	}
	int r = us.ProminentRepresent();

	std::unordered_map<int, int> mp;
	int m_nPoints = 0;
	for (int i = 0; i < (int)points.size(); ++i){
		if (r == us.Find(i)){
			points[m_nPoints] = points[i];
			if (normals.size() == points.size()){
				normals[m_nPoints] = normals[i];
			}
			mp[i] = m_nPoints;
			m_nPoints++;
		}
	}
	points.resize(m_nPoints);
	if (normals.size() == points.size()){
		normals.resize(m_nPoints);
	}

	std::vector<int> facets_;
	for (int i = 0; i < (int)facets.size(); i += 3){
		if (r != us.Find(facets[i]))	continue;
		facets_.push_back(mp[facets[i]]);
		facets_.push_back(mp[facets[i + 1]]);
		facets_.push_back(mp[facets[i + 2]]);
	}
	swap(facets_, facets);
}