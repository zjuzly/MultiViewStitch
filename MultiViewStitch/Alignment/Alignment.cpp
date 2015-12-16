#include "Alignment.h"
#include "../SetUtils/PointSetUtils.h"
#include "../SetUtils/UnionSetUtils.h"
#include "../Parameter/ParamParser.h"
#include "../Common/Utils.h"
#include "../PlyObj/PlyObj.h"
#include "../PartRecognition/PartRecognition.h"
#include <unordered_map>
#include <queue>

void Alignment::Align(
	std::vector<Eigen::Vector3d> &src,
	std::vector<Eigen::Vector3d> &s_normals,
	std::vector<int> &s_facets,
	std::vector<Eigen::Vector3d> &tgt,
	std::vector<Eigen::Vector3d> &t_normals,
	std::vector<int> &t_facets,
	const Eigen::Vector3d &viewRay){

	std::cout << "Remove Ground" << std::endl;
	Eigen::Vector3d groundRay = RemoveGround(tgt, t_normals, t_facets);

	Eigen::Matrix4d transform;
	double scale = 1.0;

	std::cout << "Initial Alignment" << std::endl;
	InitAlignment(src, s_normals, tgt, t_normals, groundRay, viewRay, transform, scale);

	Eigen::Matrix3d R = transform.block(0, 0, 3, 3);
	Eigen::Vector3d translate = transform.block(0, 3, 3, 1);
	for (int i = 0; i < (int)src.size(); ++i){
		src[i] = R * src[i] * scale + translate;
		s_normals[i] = R * s_normals[i];
	}
	std::string filename = "./Result/initAlign0.obj";
	WriteObj(filename, src, s_normals, s_facets);

	PartRecognition pr1;
	pr1.SetInput(src);
	pr1.LoadParts("./Template/part/parts");
	std::vector<int> s_labels = pr1.GetLabels();
	Visualization("./parts.obj", src, s_labels);

	std::vector<int> t_labels;
	std::cout << "Part Recognition" << std::endl;
	clock_t start_clock, end_clock;
	start_clock = clock();

	pr1.PartRecog(tgt, t_labels);

	end_clock = clock();
	std::cout << "Cost Time: " << (end_clock - start_clock) << "ms" << std::endl;
	Visualization("./parts_tgt.obj", tgt, t_labels);

#if 1
	Eigen::Vector3d bc1(0.0f, 0.0f, 0.0f);
	Eigen::Vector3d bc2(0.0f, 0.0f, 0.0f);
	int count1 = 0, count2 = 0;
	for (int i = 0; i < (int)src.size(); ++i){ if (NECK == s_labels[i]){ count1++; bc1 += src[i]; } }
	for (int i = 0; i < (int)tgt.size(); ++i){ if (NECK == t_labels[i]){ count2++; bc2 += tgt[i]; } }
	bc1 /= (double)count1;
	bc2 /= (double)count2;
	Eigen::Vector3d offset = bc2 - bc1;
	for (int i = 0; i < (int)src.size(); ++i){ src[i] = src[i] + offset; }

	filename = "./Result/initAlign1.obj";
	WriteObj(filename, src, s_normals, s_facets);
#endif

	std::cout << "Local Alignment" << std::endl;
	LocalAlignment(src, s_normals, s_labels, tgt, t_normals, t_labels);
	//std::cout << "Align By Shoulder" << std::endl;
	//AlignByShoulder(src, s_normals, s_labels, tgt, t_labels);
	filename = "./Result/initAlign.obj";
	WriteObj(filename, src, s_normals, s_facets);
}


Eigen::Vector3d Alignment::RemoveGround(
	std::vector<Eigen::Vector3d> &points,
	std::vector<Eigen::Vector3d> &normals,
	std::vector<int> &facets){

	//this->RetainConnectRegion(points, normals, facets);
	//return Eigen::Vector3d(1.0, 0.0, 0.0);

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
		//if (!remove[facets[i]] && !remove[facets[i + 1]] && !remove[facets[i + 2]]){
		if (mp.find(facets[i]) != mp.end() && mp.find(facets[i + 1]) != mp.end() && mp.find(facets[i + 2]) != mp.end()){
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
	RetainConnectRegion(points, normals, facets);
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

void Alignment::LocalAlignment(
	std::vector<Eigen::Vector3d> &src,
	std::vector<Eigen::Vector3d> &s_normals,
	const std::vector<int> &s_labels,
	const std::vector<Eigen::Vector3d> &tgt,
	const std::vector<Eigen::Vector3d> &t_normals,
	const std::vector<int> &t_labels){
	using namespace std;
	using namespace Eigen;

	vector<Vector3d> leftArmPts1, rightArmPts1;
	vector<Vector3d> leftLegPts1, rightLegPts1;
	vector<int> leftArmIdx1, rightArmIdx1;
	vector<int> leftLegIdx1, rightLegIdx1;
	for (int i = 0; i < (int)src.size(); ++i){
		if (LEFT_UPPER_ARM == s_labels[i] || LEFT_LOWER_ARM == s_labels[i] || LEFT_HAND == s_labels[i])		{
			leftArmPts1.push_back(src[i]);
			leftArmIdx1.push_back(s_labels[i]);
		}
		if (RIGHT_UPPER_ARM == s_labels[i] || RIGHT_LOWER_ARM == s_labels[i] || RIGHT_HAND == s_labels[i])	{
			rightArmPts1.push_back(src[i]);
			rightArmIdx1.push_back(s_labels[i]);
		}
		if (LEFT_THIGH == s_labels[i] || LEFT_SHANK == s_labels[i]/* || LEFT_FOOT == s_labels[i]*/)				{
			leftLegPts1.push_back(src[i]);
			leftLegIdx1.push_back(s_labels[i]);
		}
		if (RIGHT_THIGH == s_labels[i] || RIGHT_SHANK == s_labels[i]/* || RIGHT_FOOT == s_labels[i]*/)			{
			rightLegPts1.push_back(src[i]);
			rightLegIdx1.push_back(s_labels[i]);
		}
	}
	vector<Vector3d> leftArmPts2, rightArmPts2;
	vector<Vector3d> leftLegPts2, rightLegPts2;
	vector<int> leftArmIdx2, rightArmIdx2;
	vector<int> leftLegIdx2, rightLegIdx2;
	for (int i = 0; i < (int)tgt.size(); ++i){
		if (LEFT_UPPER_ARM == t_labels[i] || LEFT_LOWER_ARM == t_labels[i] || LEFT_HAND == t_labels[i])		{
			leftArmPts2.push_back(tgt[i]);
			leftArmIdx2.push_back(t_labels[i]);
		}
		if (RIGHT_UPPER_ARM == t_labels[i] || RIGHT_LOWER_ARM == t_labels[i] || RIGHT_HAND == t_labels[i])	{
			rightArmPts2.push_back(tgt[i]);
			rightArmIdx2.push_back(t_labels[i]);
		}
		if (LEFT_THIGH == t_labels[i] || LEFT_SHANK == t_labels[i]/* || LEFT_FOOT == t_labels[i]*/)				{
			leftLegPts2.push_back(tgt[i]);
			leftLegIdx2.push_back(t_labels[i]);
		}
		if (RIGHT_THIGH == t_labels[i] || RIGHT_SHANK == t_labels[i]/* || RIGHT_FOOT == t_labels[i]*/)			{
			rightLegPts2.push_back(tgt[i]);
			rightLegIdx2.push_back(t_labels[i]);
		}
	}

	Matrix4d transform;
	Matrix3d R;
	Vector3d translate;
	double scale = 1.0;
	//Arm
#pragma region
	//Left Arm
	LocalAlignmentCore(leftArmPts1, leftArmIdx1, leftArmPts2, leftArmIdx2, LEFT_HAND, LEFT_HAND, transform, scale);
	R = transform.block(0, 0, 3, 3);
	translate = transform.block(0, 3, 3, 1);
	for (int i = 0; i < (int)src.size(); ++i){
		if (LEFT_UPPER_ARM == s_labels[i] || LEFT_LOWER_ARM == s_labels[i] || LEFT_HAND == s_labels[i]){
			src[i] = scale * R * src[i] + translate;
			s_normals[i] = R * s_normals[i];
		}
	}
	//Right Arm
	LocalAlignmentCore(rightArmPts1, rightArmIdx1, rightArmPts2, rightArmIdx2, RIGHT_HAND, RIGHT_HAND, transform, scale);
	R = transform.block(0, 0, 3, 3);
	translate = transform.block(0, 3, 3, 1);
	for (int i = 0; i < (int)src.size(); ++i){
		if (RIGHT_UPPER_ARM == s_labels[i] || RIGHT_LOWER_ARM == s_labels[i] || RIGHT_HAND == s_labels[i]){
			src[i] = scale * R * src[i] + translate;
			s_normals[i] = R * s_normals[i];
		}
	}
#pragma endregion
	//Leg
#pragma region
	//Left Leg
	LocalAlignmentCore(leftLegPts1, leftLegIdx1, leftLegPts2, leftLegIdx2, LEFT_SHANK/*LEFT_FOOT*/, LEFT_SHANK/*LEFT_FOOT*/, transform, scale);
	R = transform.block(0, 0, 3, 3);
	translate = transform.block(0, 3, 3, 1);
	for (int i = 0; i < (int)src.size(); ++i){
		if (LEFT_THIGH == s_labels[i] || LEFT_SHANK == s_labels[i] || LEFT_FOOT == s_labels[i]){
			src[i] = scale * R * src[i] + translate;
			s_normals[i] = R * s_normals[i];
		}
	}
	//Right Leg
	LocalAlignmentCore(rightLegPts1, rightLegIdx1, rightLegPts2, rightLegIdx2, RIGHT_SHANK/*RIGHT_FOOT*/, RIGHT_SHANK/*RIGHT_FOOT*/, transform, scale);
	R = transform.block(0, 0, 3, 3);
	translate = transform.block(0, 3, 3, 1);
	for (int i = 0; i < (int)src.size(); ++i){
		if (RIGHT_THIGH == s_labels[i] || RIGHT_SHANK == s_labels[i] || RIGHT_FOOT == s_labels[i]){
			src[i] = scale * R * src[i] + translate;
			s_normals[i] = R * s_normals[i];
		}
	}
#pragma endregion
}

void Alignment::LocalAlignmentCore(
	std::vector<Eigen::Vector3d> &src_,
	const std::vector<int> &s_labels,
	const std::vector<Eigen::Vector3d> &tgt_,
	const std::vector<int> &t_labels,
	const int slabel,
	const int tlabel,
	Eigen::Matrix4d &transform,
	double &scale){
	PointSetUtils pst1;
	pst1.SetInput(src_);

	PointSetUtils pst2;
	pst2.SetInput(tgt_);

	const static int pivot_num = 3;
	Eigen::MatrixXd src_pivots, tgt_pivots;
	pst1.CalcPivots(src_pivots, pivot_num);
	pst2.CalcPivots(tgt_pivots, pivot_num);
	
	//important
	if (src_pivots.col(0).dot(tgt_pivots.col(0)) < 0){
		tgt_pivots.col(0) = -tgt_pivots.col(0);
	}

	Eigen::Vector3d baryCenter1 = pst1.GetBarycenter();
	for (int i = 0; i < (int)src_.size(); ++i){ src_[i] -= baryCenter1; }
	Eigen::Vector3d baryCenter2 = pst2.GetBarycenter();

#if 1
	static int no = 0;
	char fn[128];
	std::vector<Eigen::Vector3d> pivots1;
	pivots1.push_back(baryCenter1);
	pivots1.push_back(src_pivots.col(0) * 100 + baryCenter1);
	pivots1.push_back(src_pivots.col(1) * 100 + baryCenter1);
	pivots1.push_back(src_pivots.col(2) * 100 + baryCenter1);
	sprintf_s(fn, "./Result/pivot_src%d.obj", ++no);
	//WritePly(fn, pivots1);
	WriteObj(fn, pivots1);

	std::vector<Eigen::Vector3d> pivots2;
	pivots2.push_back(baryCenter2);
	pivots2.push_back(tgt_pivots.col(0) * 100 + baryCenter2);
	pivots2.push_back(tgt_pivots.col(1) * 100 + baryCenter2);
	pivots2.push_back(tgt_pivots.col(2) * 100 + baryCenter2);
	sprintf_s(fn, "./Result/pivot_tgt%d.obj", no);
	//WritePly(fn, pivots2);
	WriteObj(fn, pivots2);

#endif

	std::unordered_map<int, int> scnt, tcnt;
	for (int i = 0; i < (int)s_labels.size(); ++i){ scnt[s_labels[i]]++; }
	for (int i = 0; i < (int)t_labels.size(); ++i){ tcnt[t_labels[i]]++; }
	int label = slabel;
	if (scnt.size() < tcnt.size()){
		label = slabel;
		std::unordered_map<int, int>::iterator it = tcnt.begin();
		for (; it != tcnt.end(); ++it){
			if (scnt.find(it->first) == scnt.end()){
				tcnt.erase(it->first);
				break;
			}
		}
	}
	else if (scnt.size() > tcnt.size()){
		label = tlabel;
		std::unordered_map<int, int>::iterator it = scnt.begin();
		for (; it != scnt.end(); ++it){
			if (tcnt.find(it->first) == tcnt.end()){
				scnt.erase(it->first);
				break;
			}
		}
	}

	//Estimate the scale
	Eigen::Vector2d range1(std::numeric_limits<double>::max(), std::numeric_limits<double>::min());
	Eigen::Vector2d range2(std::numeric_limits<double>::max(), std::numeric_limits<double>::min());
	int fidx1, nidx1, fidx2, nidx2;
	Eigen::Vector3d pivot;
	pivot = src_pivots.col(0);
	for (int i = 0; i < (int)src_.size(); ++i){
		if (scnt.find(s_labels[i]) == scnt.end()) continue;
		double t = pivot.dot(src_[i]) / (pivot.norm() * pivot.norm());
		if (range1[0] > t){ range1[0] = t; fidx1 = i; }
		if (range1[1] < t){ range1[1] = t; nidx1 = i; }
	}
	int sign = 1;
	if (s_labels[nidx1] != label){
		sign = -1;
		std::swap(range1[0], range1[1]);
		std::swap(fidx1, nidx1);
	}
	pivot = tgt_pivots.col(0);
	for (int i = 0; i < (int)tgt_.size(); ++i){
		if (tcnt.find(t_labels[i]) == tcnt.end()) continue;
		double t = pivot.dot(tgt_[i] - baryCenter2) / (pivot.norm() * pivot.norm());
		if (range2[0] > t){ range2[0] = t; fidx2 = i; }
		if (range2[1] < t){ range2[1] = t; nidx2 = i; }
	}
	if (t_labels[nidx2] != label){
		std::swap(range2[0], range2[1]);
		std::swap(fidx2, nidx2);
	}
	scale = (range2[1] - range2[0]) / (range1[1] - range1[0]);

	Eigen::Matrix3d R;
	CalcRotation(src_pivots.col(0), tgt_pivots.col(0), R);
	//Eigen::Vector3d t = sign * tgt_pivots.col(0) * (((range2[1] - range2[0]) - (range1[1] - range1[0]))) * 0.8f;
	//Eigen::Vector3d translate = t + src_[fidx1] + baryCenter1 - R * (src_[fidx1] + baryCenter1);
	Eigen::Vector3d translate = src_[fidx1] + baryCenter1 - scale * R * (src_[fidx1] + baryCenter1);

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

void Alignment::AlignByShoulder(
	std::vector<Eigen::Vector3d> &src,
	const std::vector<Eigen::Vector3d> &s_normals,
	const std::vector<int> &slabels,
	const std::vector<Eigen::Vector3d> &tgt,
	const std::vector<int> &tlabels){
	std::vector<std::vector<int>> index;
	LoadShoulderJoints("./Template/ShoulderJoint", index);
	std::vector<Eigen::Vector3d> s_joints(2, Eigen::Vector3d(0.0, 0.0, 0.0));
	std::vector<Eigen::Vector3d> s_norm(2, Eigen::Vector3d(0.0, 0.0, 0.0));
	for (int i = 0; i < (int)index.size(); ++i){
		for (int j = 0; j < (int)index[i].size(); ++j){
			s_joints[i] += src[index[i][j]];
			s_norm[i] += s_normals[index[i][j]];
		}
	}
	s_joints[0] /= (double)index[0].size();
	s_joints[1] /= (double)index[1].size();
	s_norm[0] = (s_norm[0] / (double)index[0].size()).normalized();
	s_norm[1] = (s_norm[1] / (double)index[1].size()).normalized();

	const int k = 50;
	std::priority_queue<Node> Q1, Q2;
	int lcnt = 0, rcnt = 0;
	for (int i = 0; i < (int)tgt.size(); ++i){
		if (tlabels[i] == NECK || tlabels[i] == LEFT_UPPER_ARM || tlabels[i] == TRUNCUS){
			double d = (double)(s_joints[0] - tgt[i]).norm();
			Q1.push(Node(d, i));
			if (Q1.size() > k) Q1.pop();
		}
		if (tlabels[i] == NECK || tlabels[i] == RIGHT_UPPER_ARM || tlabels[i] == TRUNCUS){
			double d = (double)(s_joints[1] - tgt[i]).norm();
			Q2.push(Node(d, i));
			if (Q2.size() > k) Q2.pop();
		}
	}
	std::vector<Eigen::Vector3d> t_joints(2, Eigen::Vector3d(0.0, 0.0, 0.0));
	double dist1 = 0.0, dist2 = 0.0;
	while (!Q1.empty()){
		Node p = Q1.top(); Q1.pop();
		dist1 += p.d;
		t_joints[0] = t_joints[0] + tgt[p.index];
		++lcnt;
	}
	while (!Q2.empty()){
		Node p = Q2.top(); Q2.pop();
		dist2 += p.d;
		t_joints[1] = t_joints[1] + tgt[p.index];
		++rcnt;
	}
	dist1 /= (double)lcnt;
	dist2 /= (double)rcnt;
	s_norm[0] *= dist1;
	s_norm[1] *= dist2;
	t_joints[0] /= (double)lcnt;
	t_joints[1] /= (double)rcnt;

	if (s_norm[0].dot(t_joints[0] - s_joints[0]) < 0.0f){ s_norm[0] = -s_norm[0]; }
	if (s_norm[1].dot(t_joints[1] - s_joints[1]) < 0.0f){ s_norm[1] = -s_norm[1]; }

	for (int i = 0; i < (int)src.size(); ++i){
		if (slabels[i] == LEFT_UPPER_ARM || slabels[i] == LEFT_LOWER_ARM || slabels[i] == LEFT_HAND){
			src[i] = src[i] + s_norm[0];
		}
		if (slabels[i] == RIGHT_UPPER_ARM || slabels[i] == RIGHT_LOWER_ARM || slabels[i] == RIGHT_HAND){
			src[i] = src[i] + s_norm[1];
		}
	}
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