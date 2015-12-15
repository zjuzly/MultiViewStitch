#include "Alignment.h"
#include "../SetUtils/PointSetUtils.h"
#include "../SetUtils/UnionSetUtils.h"
#include "../Parameter/ParamParser.h"
#include "../Common/Utils.h"
#include "../Common/PlyObj.h"
#include <unordered_map>

Eigen::Vector3d Alignment::RemoveGround(
	std::vector<Eigen::Vector3d> &points,
	std::vector<Eigen::Vector3d> &normals,
	std::vector<int> &facets){

	this->RetainConnectRegion(points, normals, facets);
	return Eigen::Vector3d(1.0, 0.0, 0.0);

	Eigen::Vector3d groundRay;
	PointSetUtils pst;
	pst.SetInput(points);

	const static int pivot_num = 3;
	Eigen::MatrixXd pivots;
	pst.CalcPivots(pivots, pivot_num);

	Eigen::Vector3d baryCenter = pst.GetBarycenter();

	std::vector<Eigen::Vector3d> candidates1, candidates2;
	std::vector<int> idx1, idx2;
	std::vector<std::pair<double, int>> dist_index1, dist_index2;
	double tMax1 = std::numeric_limits<double>::min();
	double tMax2 = std::numeric_limits<double>::min();
	Eigen::Vector3d pivot = pivots.col(0);
	for (int i = 0; i < (int)points.size(); ++i){
		double t = pivot.dot(points[i] - baryCenter) / (pivot.norm() * pivot.norm());
		if (t < 0){
			dist_index1.push_back(std::make_pair(-t, i));
			tMax1 = std::max(-t, tMax1);
		}
		else{
			dist_index2.push_back(std::make_pair(t, i));
			tMax2 = std::max(t, tMax2);
		}
	}

	for (int i = 0; i < (int)dist_index1.size(); ++i){
		if (dist_index1[i].first > tMax1 * ParamParser::dist_thres){
			candidates1.push_back(points[dist_index1[i].second]);
			idx1.push_back(dist_index1[i].second);
		}
	}
	for (int i = 0; i < (int)dist_index2.size(); ++i){
		if (dist_index2[i].first > tMax2 * ParamParser::dist_thres){
			candidates2.push_back(points[dist_index2[i].second]);
			idx2.push_back(dist_index2[i].second);
		}
	}
	std::vector<Eigen::Vector3d> candidates;
	std::vector<int> idx;
	if (candidates1.size() > candidates2.size()){
		candidates = candidates1;
		idx = idx1;
		groundRay = -pivot;
	}
	else{
		candidates = candidates2;
		idx = idx2;
		groundRay = pivot;
	}
	std::string filename;
	filename = "./Result/candidate1.obj";
	WriteObj(filename, candidates1);
	filename = "./Result/candidate2.obj";
	WriteObj(filename, candidates2);
	filename = "./Result/candidate.obj";
	WriteObj(filename, candidates);

	//fit plane
	Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
	Eigen::Vector3d b(0.0f, 0.0f, 0.0f);
	for (int i = 0; i < (int)candidates.size(); ++i){
		A += candidates[i] * candidates[i].transpose();
		b += candidates[i];
	}
	Eigen::Vector3d ans = -A.inverse() * b;
	//plane: [ans, d]
	double d = 1.0 / ans.norm();
	ans.normalize();
	if (ans.dot(pivot) < 0){
		ans = -ans;
		d = -d;
	}

	std::cout << "+-------------------------------------------------------+" << std::endl;
	std::cout << "plane parameters: [" << ans.transpose() << " " << d << "]" << std::endl;
	std::cout << "+-------------------------------------------------------+" << std::endl;

#if 1
	std::vector<Eigen::Vector3d> plane;
	plane.push_back(Eigen::Vector3d(0, 0, -d / ans[2]));
	plane.push_back(Eigen::Vector3d(100, 0, (-100 * ans[0] - d) / ans[2]));
	plane.push_back(Eigen::Vector3d(0, 100, (-100 * ans[1] - d) / ans[2]));
	plane.push_back(Eigen::Vector3d(100, 100, (-100 * ans[1] - 100 * ans[0] - d) / ans[2]));
	std::vector<int> face;
	face.push_back(0); face.push_back(1); face.push_back(2);
	face.push_back(1); face.push_back(3); face.push_back(2);
	filename = "./Result/plane.obj";
	WriteObj(filename, plane, std::vector<Eigen::Vector3d>(), face);
#endif

	std::vector<std::pair<double, int>> dists;
	double maxDist = std::numeric_limits<double>::min();
	for (int i = 0; i < (int)candidates.size(); ++i){
		double dist = fabs(ans.dot(candidates[i]) + d);
		maxDist = std::max(maxDist, dist);
		dists.push_back(std::make_pair(dist, idx[i]));
	}
	double threshold = maxDist * 0.28;

	int remCnt = 0;
	std::vector<bool> remove(points.size(), false);
	for (int i = 0; i < dists.size(); ++i){
		if (dists[i].first < threshold){ remove[dists[i].second] = true; remCnt++; }
	}
	std::cout << "Remove: " << remCnt << std::endl;

	std::unordered_map<int, int> mp;
	int m_nPoints = 0;
	for (int i = 0; i < (int)points.size(); ++i){
		if (!remove[i]){
			points[m_nPoints] = points[i];
			normals[m_nPoints] = normals[i];
			mp[i] = m_nPoints;
			m_nPoints++;
		}
	}
	points.resize(m_nPoints);
	normals.resize(m_nPoints);

	std::vector<int> facets_;
	int m_nFacets = 0;
	for (int i = 0; i < (int)facets.size(); i += 3){
		if (!remove[facets[i]] && !remove[facets[i + 1]] && !remove[facets[i + 2]]){
			facets_.push_back(mp[facets[i]]);
			facets_.push_back(mp[facets[i + 1]]);
			facets_.push_back(mp[facets[i + 2]]);
		}
	}
	swap(facets_, facets);
	std::cout << "points: " << m_nPoints << std::endl;
#if 1
	filename = "./Result/ModelTrim0.obj";
	WriteObj(filename, points, normals, facets);
#endif

	//Retain maximum connected region
	UnionSet us((int)points.size() + 1);
	for (int i = 0; i < (int)facets.size(); i += 3){
		us.Merge(facets[i], facets[i + 1]);
		us.Merge(facets[i], facets[i + 2]);
	}
	int r = us.ProminentRepresent();
	m_nPoints = 0;
	mp.clear();
	for (int i = 0; i < (int)points.size(); ++i){
		if (r == us.Find(i)){
			points[m_nPoints] = points[i];
			normals[m_nPoints] = normals[i];
			mp[i] = m_nPoints;
			m_nPoints++;
		}
	}
	points.resize(m_nPoints);
	normals.resize(m_nPoints);

	facets_.clear();
	m_nFacets = 0;
	for (int i = 0; i < (int)facets.size(); i += 3){
		if (r != us.Find(facets[i]))	continue;
		facets_.push_back(mp[facets[i]]);
		facets_.push_back(mp[facets[i + 1]]);
		facets_.push_back(mp[facets[i + 2]]);
	}
	swap(facets_, facets);
#if 1
	filename = "./Result/ModelTrim.obj";
	WriteObj(filename, points, normals, facets);
#endif
	return groundRay;
}

void Alignment::InitAlignment(
	const std::vector<Eigen::Vector3d> &src,
	const std::vector<Eigen::Vector3d> &s_normals,
	const std::vector<Eigen::Vector3d> &tgt,
	const std::vector<Eigen::Vector3d> &t_normals,
	const Eigen::Vector3d &groundRay,
	const Eigen::Vector3d &viewRay,
	Eigen::Matrix4d &transform,
	double &scale){

	PointSetUtils pst1;
	pst1.SetInput(src);

	PointSetUtils pst2;
	pst2.SetInput(tgt);

	const static int pivot_num = 3;
	Eigen::MatrixXd src_pivots, tgt_pivots;
	pst1.CalcPivots(src_pivots, pivot_num);
	pst2.CalcPivots(tgt_pivots, pivot_num);
	if (groundRay.dot(tgt_pivots.col(0)) < 0) tgt_pivots.col(0) = -tgt_pivots.col(0);
	if (viewRay.dot(tgt_pivots.col(2)) < 0) tgt_pivots.col(2) = -tgt_pivots.col(2);

	Eigen::Vector3d baryCenter1 = pst1.GetBarycenter();
	//for (int i = 0; i < (int)src.size(); ++i){ src[i] -= baryCenter1; }
	Eigen::Vector3d baryCenter2 = pst2.GetBarycenter();

#if 1
	std::string filename;
	std::vector<Eigen::Vector3d> pivots1;
	pivots1.push_back(baryCenter1);
	pivots1.push_back(src_pivots.col(0) * 10 + baryCenter1);
	pivots1.push_back(src_pivots.col(1) * 10 + baryCenter1);
	pivots1.push_back(src_pivots.col(2) * 10 + baryCenter1);
	filename = "./Result/pivot_src.obj";
	WriteObj(filename, pivots1);

	std::vector<Eigen::Vector3d> pivots2;
	pivots2.push_back(baryCenter2);
	pivots2.push_back(tgt_pivots.col(0) * 300 + baryCenter2);
	pivots2.push_back(tgt_pivots.col(1) * 300 + baryCenter2);
	pivots2.push_back(tgt_pivots.col(2) * 300 + baryCenter2);
	filename = "./Result/pivot_tgt.obj";
	WriteObj(filename, pivots2);
#endif

	//Estimate the scale
	Eigen::Vector2d range1(std::numeric_limits<double>::max(), std::numeric_limits<double>::min());
	Eigen::Vector2d range2(std::numeric_limits<double>::max(), std::numeric_limits<double>::min());
	Eigen::Vector3d pivot;
	pivot = src_pivots.col(0);
	for (int i = 0; i < (int)src.size(); ++i){
		double t = pivot.dot(src[i] - baryCenter1) / (pivot.norm() * pivot.norm());
		range1[0] = std::min(range1[0], t);
		range1[1] = std::max(range1[1], t);
	}
	pivot = tgt_pivots.col(0);
	for (int i = 0; i < (int)tgt.size(); ++i){
		double t = pivot.dot(tgt[i] - baryCenter2) / (pivot.norm() * pivot.norm());
		range2[0] = std::min(range2[0], t);
		range2[1] = std::max(range2[1], t);
	}

	scale = (range2[1] - range2[0]) / (range1[1] - range1[0]);

	Eigen::Matrix3d R = tgt_pivots * src_pivots.inverse();
	Eigen::Vector3d translate = tgt_pivots.col(0) * (range2[1] - range1[1] * scale) + baryCenter2 - scale * R * baryCenter1;

	//Eigen::Vector3d translate = baryCenter2 - scale * R * baryCenter1;

	transform = Eigen::Matrix4d::Identity();
	transform.block(0, 0, 3, 3) = R;
	transform.block(0, 3, 3, 1) = translate;

#if 0
	std::cout << "Rotation Matrix: " << std::endl << R << std::endl;
	std::cout << "translation: " << translate.transpose() << std::endl;
	std::cout << "Det: " << R.determinant() << std::endl;
	std::cout << "Scale: " << scale << std::endl;
#endif
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