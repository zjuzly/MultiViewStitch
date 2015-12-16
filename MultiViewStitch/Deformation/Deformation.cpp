#pragma warning(disable: 4819)
#pragma warning(disable: 4996)

#include "Deformation.h"
#include "../common/Utils.h"
#include <opencv2\opencv.hpp>
#include <queue>

Deformation::Deformation(const std::string filename){
	typedef CgalPolyhedron::HalfedgeDS HDS;
	try{
		std::vector<Eigen::Vector3d> points_, normals_;
		std::vector<int> facets_;
		ReadObj(filename, points_, normals_, facets_);
		this->normals = normals_;
			
		Build_triangle<HDS> BT(points_, facets_);
		poly.delegate(BT);
		if(!poly.is_valid()){
			throw CGAL::Assertion_exception("", "", "", 0, "");
		}
	}catch(const CGAL::Assertion_exception&){
		std::string _msg = "Error: Deformation@params-[filename]";
		std::cerr << _msg << std::endl;
		exit(-1);
	}
}

Deformation::Deformation(
	const std::vector<Eigen::Vector3d> &points_,
	const std::vector<Eigen::Vector3d> &normals_,
	const std::vector<int> &facets_){
	typedef CgalPolyhedron::HalfedgeDS HDS;
	this->normals = normals_;
	try{	
		Build_triangle<HDS> BT(points_, facets_);
		poly.delegate(BT);
		if(!poly.is_valid()){
			throw CGAL::Assertion_exception("", "", "", 0, "");
		}
	}catch(const CGAL::Assertion_exception&){
		std::string _msg = "Error: Deformation@params-[points, facets]";
		std::cerr << _msg << std::endl;
		exit(-1);
	}
}

void Deformation::Simplification(){
	int npoints = (int)poly.size_of_vertices();
	SMS::Count_stop_predicate<CgalPolyhedron> stop((int)(npoints * 1.0));
	int r = SMS::edge_collapse
		(poly,
		stop,
		CGAL::vertex_index_map(get(CGAL::vertex_external_index, poly))
		.halfedge_index_map(get(CGAL::halfedge_external_index, poly))
		.get_cost(SMS::Edge_length_cost<CgalPolyhedron>())
		.get_placement(SMS::Midpoint_placement<CgalPolyhedron>())
		);
	std::cout << "\nFinished...\n" << r << " edges remove.\n"
		<< (poly.size_of_halfedges() / 2) << " final edges.\n";
}

void Deformation::UniformSampling(){
	vertex_iterator vb, ve;
	boost::tie(vb, ve) = vertices(poly);

	int npoints = (int)poly.size_of_vertices();
	cv::flann::KDTreeIndexParams indexParams;
	cv::Mat cvNodes = cv::Mat::zeros(npoints, 3, CV_32FC1);
	
	int i = 0, j = 0;
	CgalPolyhedron::Point_iterator it;
	for(it = poly.points_begin(); it != poly.points_end(); ++it){
		cvNodes.at<float>(i, 0) = (float)it->x();
		cvNodes.at<float>(i, 1) = (float)it->y();
		cvNodes.at<float>(i, 2) = (float)it->z();
		i++;
	}
	cv::flann::Index kdtree(cvNodes, indexParams);

	std::vector<bool> remove(npoints, false);
	sampIdx.clear();
	std::vector<Eigen::Vector3d> points;
	for(i = 0; i < npoints; ++i){
		if(!remove[i]){
			sampIdx.push_back(i);
			float x = cvNodes.at<float>(i, 0);
			float y = cvNodes.at<float>(i, 1);
			float z = cvNodes.at<float>(i, 2);
			points.push_back(Eigen::Vector3d(x, y, z));
			std::vector<float> query;
			query.push_back(x);
			query.push_back(y);
			query.push_back(z);
			std::vector<int> indices;
			std::vector<float> dists;
			kdtree.knnSearch(query, indices, dists, 16, cv::flann::SearchParams(64));
			for(j = 0; j < (int)indices.size(); ++j){
				if(i != indices[j]){
					remove[indices[j]] = true;
				}
			}
		}
	}
	WriteObj("./Result/sample.obj", points);
}

std::vector<std::vector<std::pair<int, double>>> Deformation::KNearestNeighbor(int K){
	int npoints = (int)sampIdx.size();
	cv::flann::KDTreeIndexParams indexParams;
	cv::Mat cvNodes = cv::Mat::zeros(npoints, 3, CV_32FC1);

	vertex_iterator vb, ve;
	boost::tie(vb, ve) = vertices(poly);
	for(int i = 0; i < (int)sampIdx.size(); ++i){
		int idx = sampIdx[i];
		vertex_descriptor p = *(CGAL::cpp11::next(vb, idx));

		cvNodes.at<float>(i, 0) = (float)p->point()[0];
		cvNodes.at<float>(i, 1) = (float)p->point()[1];
		cvNodes.at<float>(i, 2) = (float)p->point()[2];
	}
	cv::flann::Index kdtree(cvNodes, indexParams);
	
	//const int K = 8; //CommandParser::kNearest;
	std::vector<std::vector<std::pair<int, double>>> res(sampIdx.size());
	for(int i = 0; i < (int)sampIdx.size(); ++i){
		std::vector<float> query;
		query.push_back(cvNodes.at<float>(i, 0));
		query.push_back(cvNodes.at<float>(i, 1));
		query.push_back(cvNodes.at<float>(i, 2));
		std::vector<int> indices(K + 1, -1);
		std::vector<float> dists;
		kdtree.knnSearch(query, indices, dists, K + 1, cv::flann::SearchParams(64));
		
		float sum = 0.0f;
		for (int j = 0; j < (int)dists.size(); ++j){
			sum += dists[j];
			//if(indices[j] != i){ sum += dists[j]; }
		}

		for(int j = 0; j < (int)indices.size(); ++j){
			res[i].push_back(std::make_pair(indices[j], 1.0 / (K + 1)));
			//if (indices[j] != i){
			//	res[i].push_back(std::make_pair(indices[j], 0.6 * (1.0 - dists[j] / sum)));
			//}
			//else{
			//	res[i].push_back(std::make_pair(indices[j], 0.4));
			//}
		}
	}
	return res;
}

void Deformation::Deform(
	const std::vector<Eigen::Vector3d> &tpts,
	const std::vector<Eigen::Vector3d> &tnormals,
	const double threshold){
	int npoints = (int)tpts.size();
	cv::flann::KDTreeIndexParams indexParams;
	cv::Mat cvNodes = cv::Mat::zeros(npoints, 6, CV_32FC1);
	float scale = 0.0f;
	for(int i = 0; i < npoints; ++i){
		cvNodes.at<float>(i, 0) = tpts[i][0];
		cvNodes.at<float>(i, 1) = tpts[i][1];
		cvNodes.at<float>(i, 2) = tpts[i][2];
		cvNodes.at<float>(i, 3) = tnormals[i][0] * scale;
		cvNodes.at<float>(i, 4) = tnormals[i][1] * scale;
		cvNodes.at<float>(i, 5) = tnormals[i][2] * scale;
	}
	cv::flann::Index kdtree(cvNodes, indexParams);
	
	set_halfedgeds_items_id(poly);
	
	CgalPolyhedron::Vertex_iterator it = poly.vertices_begin();
	it->point();

	Surface_mesh_deformation deform_mesh(poly);

	vertex_iterator vb, ve;
	boost::tie(vb, ve) = vertices(poly);
	deform_mesh.insert_roi_vertices(vb, ve);

	for(int i = 0; i < (int)sampIdx.size(); ++i){
		int idx = sampIdx[i];
		vertex_descriptor control_i = *(CGAL::cpp11::next(vb, idx));
		CgalPolyhedron::Point_3 p = control_i->point();
		std::vector<float> query;
		query.push_back((float)p[0]);
		query.push_back((float)p[1]);
		query.push_back((float)p[2]);
		query.push_back((float)normals[idx][0] * scale);
		query.push_back((float)normals[idx][1] * scale);
		query.push_back((float)normals[idx][2] * scale);
		std::vector<int> indices;
		std::vector<float> dists;
		kdtree.knnSearch(query, indices, dists, 1, cv::flann::SearchParams(32));
		if(dists[0] > threshold) continue;
		
		CgalPolyhedron::Point_3 p1(tpts[indices[0]][0], tpts[indices[0]][1], tpts[indices[0]][2]);

		Eigen::Vector3d dir((p1[0] - p[0]), (p1[1] - p[1]), (p1[2] - p[2]));
		if(dir.dot(normals[idx]) / (dir.norm() * normals[idx].norm()) < 0.5)	continue;

		Surface_mesh_deformation::Point constrained_pos_i(p1);

		deform_mesh.insert_control_vertex(control_i);
		deform_mesh.set_target_position(control_i, constrained_pos_i);
	}
#if 0
	vertex_descriptor control_1 = *(CGAL::cpp11::next(vb, 181));
	vertex_descriptor control_2 = *(CGAL::cpp11::next(vb, 6702));
	deform_mesh.insert_control_vertex(control_1);
	deform_mesh.insert_control_vertex(control_2);
	
	CgalPolyhedron::Point_3 p = control_1->point();
	CgalPolyhedron::Point_3 p1(p[0], p[1], p[2] + 20);
	CgalPolyhedron::Point_3 p2(p[0], p[1], p[2] - 20);
	Surface_mesh_deformation::Point constrained_pos_1(p1);
	Surface_mesh_deformation::Point constrained_pos_2(p2);
	deform_mesh.set_target_position(control_1, constrained_pos_1);
	//deform_mesh.set_target_position(control_2, constrained_pos_2);
#endif
	bool is_matrix_factorization_OK = deform_mesh.preprocess();
	if(!is_matrix_factorization_OK){
		std::cerr << "Error in preprocessing, check documentation of preprocess()" << std::endl;
		exit(-1);
	}
	deform_mesh.deform();
}

void Deformation::Deform(
	const std::vector<Eigen::Vector3d> &tpts,
	const std::vector<Eigen::Vector3d> &tnormals,
	const double projLenErr,
	const double projDistErr){
	int npoints = (int)tpts.size();
	cv::Mat cvNodes = cv::Mat::zeros(npoints, 3, CV_32FC1);
	for(int i = 0; i < npoints; ++i){
		cvNodes.at<float>(i, 0) = tpts[i][0];
		cvNodes.at<float>(i, 1) = tpts[i][1];
		cvNodes.at<float>(i, 2) = tpts[i][2];
	}
	const int maxResult = 10000;
	cv::flann::KDTreeIndexParams indexParams;
	cv::flann::Index kdtree(cvNodes, indexParams);

	if (sampIdx.size() == 0){
		UniformSampling();
	}

	int counter = 1;
	while(counter--){
		set_halfedgeds_items_id(poly);

		Surface_mesh_deformation deform_mesh(poly);
	
		vertex_iterator vb, ve;
		boost::tie(vb, ve) = vertices(poly);
		deform_mesh.insert_roi_vertices(vb, ve);

		std::vector<Eigen::Vector3d> controls(sampIdx.size());
		std::vector<Eigen::Vector3d> orig(sampIdx.size());
		std::vector<bool> isValid(sampIdx.size(), false);

		for(int i = 0; i < (int)sampIdx.size(); ++i){
			int idx = sampIdx[i];
			vertex_descriptor control_i = *(CGAL::cpp11::next(vb, idx));
		
			CgalPolyhedron::Point_3 p = control_i->point();
			controls[i] = Eigen::Vector3d(p[0], p[1], p[2]);
			orig[i] = controls[i];
		
			std::vector<float> query;
			query.push_back((float)p[0]);
			query.push_back((float)p[1]);
			query.push_back((float)p[2]);

			int newSize = 0;
#if 1
			std::vector<int> indices;
			std::vector<float> dists;
			kdtree.knnSearch(query, indices, dists, 1, cv::flann::SearchParams(64));
			float minDist = dists[0];

			indices.resize(maxResult, -1);
			dists.resize(maxResult);
			kdtree.radiusSearch(query, indices, dists, minDist * 2.0f, maxResult, cv::flann::SearchParams(64));
			
			for(int k = 0; k < indices.size(); ++k){
				if(indices[k] == -1){
					newSize = k;
					break;
				}
			}
			indices.resize(newSize);
			dists.resize(newSize);
#else
			std::vector<int> indices;
			std::vector<float> dists;
			kdtree.knnSearch(query, indices, dists, 64, cv::flann::SearchParams(64));
#endif
			//Eigen::Vector3d ori(p[0], p[1], p[2]);
			Eigen::Vector3d norm = normals[idx];
			newSize = 0;
			for (int k = 0; k < indices.size(); ++k){
				if (norm.dot(tnormals[indices[k]]) > 0){
					indices[newSize] = indices[k];
					dists[newSize] = dists[k];
					newSize++;
				}
			}
			indices.resize(newSize);
			dists.resize(newSize);
			if (newSize == 0)	continue;
#if 1
#pragma region
			struct Node{
				double projLen;
				double projDist;
				int idx;
				Node(double pl, double pj, int i) : projLen(pl), projDist(pj), idx(i){}
				bool operator < (const Node& other) const {
					if(fabs(projDist - other.projDist) <= 1e-6)	return fabs(projLen) > fabs(other.projLen);
					else	return projDist > other.projDist;
				}
			};
#pragma endregion
			std::priority_queue<Node> Q;
			for(int j = 0; j < (int)indices.size(); ++j){
				Eigen::Vector3d dir = tpts[indices[j]] - orig[i];//ori;
				double projLen = dir.dot(norm) / norm.norm();
				//if(projLen < 0)	projLen = -projLen;
				double projDist = sqrt(dir.squaredNorm() - projLen * projLen);
				Q.push(Node(projLen, projDist, indices[j]));
			}
			std::cout << Q.size() << " ";
			const int neighborNum = __min(8, (int)Q.size());
			double m_projLen = 0, m_projDist = 0;
			Eigen::Vector3d m_pts(0, 0, 0);
			for(int j = 0; j < neighborNum; ++j){
				Node cur = Q.top(); Q.pop();
				m_projLen += cur.projLen;
				m_projDist += cur.projDist;
				m_pts = m_pts + tpts[cur.idx];
			}
			m_projLen	/= neighborNum;
			m_projDist	/= neighborNum;
			m_pts		/= neighborNum;
			if(m_projLen >= projLenErr || m_projDist >= projDistErr)	continue;

			Eigen::Vector3d dir((m_pts[0] - p[0]), (m_pts[1] - p[1]), (m_pts[2] - p[2]));
			if(fabs(dir.dot(norm) / (dir.norm() * norm.norm())) < 0.1)	continue;
#endif
			isValid[i] = true;
			controls[i] = m_pts;
		}
#if 1
		std::vector<std::vector<std::pair<int, double>>> neighbor = KNearestNeighbor(8);
		std::vector<Eigen::Vector3d> controlTmp(controls.size());
		std::cout << std::endl << "Deformation..." << std::endl;
		for(int iter = 0; iter < 2; ++iter){
			std::cout << "Iteration# " << iter << std::endl;
			for(int i = 0; i < (int)controls.size(); ++i){
				std::vector<std::pair<int, double>>& W = neighbor[i];
				//double length = 0.0;
				Eigen::Vector3d tmp(0, 0, 0);
				for(int j = 0; j < W.size(); ++j){
					int idx = W[j].first;
					double w = W[j].second;
					tmp = tmp + w * (controls[idx] - orig[idx]);
					//double normLen = (controls[idx] - orig[idx]).stableNorm();
					//length = length + w * normLen;
				}
				//Eigen::Vector3d norm = normals[sampIdx[i]];
				//norm.normalize();
				//controlTmp[i] = orig[i] + norm * length;
				controlTmp[i] = orig[i] + tmp;
			}
			std::swap(controls, controlTmp);
		}
#endif
		for(int i = 0; i < (int)controls.size(); ++i){
			//if(!isValid[i])	continue;
			CgalPolyhedron::Point_3 p1(controls[i][0], controls[i][1], controls[i][2]);
			Surface_mesh_deformation::Point constrained_pos_i(p1);
		
			int idx = sampIdx[i];
			vertex_descriptor control_i = *(CGAL::cpp11::next(vb, idx));
			deform_mesh.insert_control_vertex(control_i);
			deform_mesh.set_target_position(control_i, constrained_pos_i);
		}
		bool is_matrix_factorization_OK = deform_mesh.preprocess();
		if(!is_matrix_factorization_OK){
			std::cerr << "Error in preprocessing, check documentation of preprocess()" << std::endl;
			exit(-1);
		}
		deform_mesh.deform(5, 1e-4);
		//deform_mesh.deform(10, 1e-6);
		deform_mesh.overwrite_initial_geometry();
	}
}