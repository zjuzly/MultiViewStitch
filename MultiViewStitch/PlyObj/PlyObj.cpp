#include "PlyObj.h"

void Mesh::ReadObj(
	const std::string filename,
	std::vector<Eigen::Vector3d> &points,
	std::vector<Eigen::Vector3d> &normals,
	std::vector<int> &facets){
	ReadObjCore(filename);
	points = Vertices;
	facets = Facets;
	if (VertexNormals.size() == 0){
		std::cout << "Compute Vertex Normals..." << std::endl;
		CalculateVertexNormals();
	}
	normals = VertexNormals;
}

void Mesh::WriteObj(
	const std::string filename,
	const std::vector<Eigen::Vector3d> &points,
	const std::vector<Eigen::Vector3d> &normals,
	const std::vector<int> &facets){
	Vertices = points;
	VertexNormals = normals;
	Facets = facets;
	WriteObjCore(filename);
}

void Mesh::ReadObjCore(const std::string filename){
	Vertices.clear();
	VertexNormals.clear();
	Facets.clear();
	std::ifstream ifs;
	ifs.open(filename.c_str(), std::ifstream::in);
	if (!ifs.is_open()){
		std::cerr << "ERROR: Open File " << filename << " Failed! Please check if it exists" << std::endl;
		exit(-1);
	}
	char line[512];
	while (ifs.peek() != EOF){
		ifs.getline(line, 512);
		if (line[0] == '#')	continue;
		if (line[0] == 'v'){
			float x, y, z;
			if (line[1] == 'n'){
				sscanf_s(line, "vn %f %f %f", &x, &y, &z);
				//normal3f.push_back(Eigen::Vector3f(x, y, z));
				//normal3f.back().normalize();
				VertexNormals.push_back(Eigen::Vector3f(x, y, z).cast<double>());
				VertexNormals.back().normalize();
			}
			else{
				sscanf_s(line, "v %f %f %f", &x, &y, &z);
				//point3f.push_back(Eigen::Vector3f(x, y, z));
				Vertices.push_back(Eigen::Vector3f(x, y, z).cast<double>());
			}
		}
		if (line[0] == 'f'){
			int ptIdx[3], nIdx[3];
			if (VertexNormals.size() > 0 && std::string(line).find('/') != std::string::npos){
				sscanf_s(line, "f %d//%d %d//%d %d//%d", &ptIdx[0], &nIdx[0], &ptIdx[1], &nIdx[1], &ptIdx[2], &nIdx[2]);
				Facets.push_back(ptIdx[0] - 1);
				Facets.push_back(ptIdx[1] - 1);
				Facets.push_back(ptIdx[2] - 1);
			}
			else{
				sscanf_s(line, "f %d %d %d", &ptIdx[0], &ptIdx[1], &ptIdx[2]);
				Facets.push_back(ptIdx[0] - 1);
				Facets.push_back(ptIdx[1] - 1);
				Facets.push_back(ptIdx[2] - 1);
			}
		}
	}
	ifs.close();
}

void Mesh::WriteObjCore(const std::string filename){
	std::cout << filename << std::endl;
	using namespace std;
	ofstream ofs;
	ofs.open(filename.c_str(), ofstream::out);

	ofs << "####" << endl;
	ofs << "#" << endl;
	ofs << "# OBJ File Generated by MultiviewStitch Program" << endl;
	ofs << "#" << endl;
	ofs << "####" << endl;
	ofs << "# Object " << filename << endl;
	ofs << "#" << endl;
	ofs << "# Vertices: " << Vertices.size() << endl;
	ofs << "# Faces: " << Facets.size() / 3 << endl;
	ofs << "#" << endl;
	ofs << "####" << endl;

	if (VertexNormals.size() == Vertices.size()){
		for (int i = 0; i < (int)Vertices.size(); ++i){
			ofs << "vn "
				<< (float)VertexNormals[i][0] << " "
				<< (float)VertexNormals[i][1] << " "
				<< (float)VertexNormals[i][2] << endl;
			ofs << "v "
				<< (float)Vertices[i][0] << " "
				<< (float)Vertices[i][1] << " "
				<< (float)Vertices[i][2] << endl;
		}
	}
	else{
		for (int i = 0; i < (int)Vertices.size(); ++i){
			ofs << "v "
				<< (float)Vertices[i][0] << " "
				<< (float)Vertices[i][1] << " "
				<< (float)Vertices[i][2] << endl;
		}
	}

	ofs << "# " << Vertices.size() << " vertices, " << VertexNormals.size() << " vertices normals" << endl;
	ofs << endl;

	int tri_num = (int)(Facets.size() / 3);
	if (VertexNormals.size() == Vertices.size()){
		for (int i = 0; i < tri_num; ++i){
			ofs << "f "
				<< Facets[3 * i] + 1 << "//" << Facets[3 * i] + 1 << " "
				<< Facets[3 * i + 1] + 1 << "//" << Facets[3 * i + 1] + 1 << " "
				<< Facets[3 * i + 2] + 1 << "//" << Facets[3 * i + 2] + 1 << endl;
		}
	}
	else{
		for (int i = 0; i < tri_num; ++i){
			ofs << "f "
				<< Facets[3 * i] + 1 << " "
				<< Facets[3 * i + 1] + 1 << " "
				<< Facets[3 * i + 2] + 1 << endl;
		}
	}
	ofs.close();
}

void Mesh::CalculateVertexNormals(){
	if (FacetNormals.size() == 0){
		CalculateFacetNormals();
	}
	if (adjacentFacetsPerVertex.size() == 0){
		CalculateAdjacentFacetsPerVertex();
	}
	VertexNormals.reserve(Vertices.size());
	for (int i = 0; i < Vertices.size(); ++i){
		Eigen::Vector3d meanNormal(0.0, 0.0, 0.0);
		std::vector<int>& tlist = *(adjacentFacetsPerVertex[i]);
		int size = tlist.size();
		for (int j = 0; j < size; ++j){
			meanNormal = meanNormal + FacetNormals[tlist[j]];
		}
		VertexNormals.push_back(meanNormal / size);
		VertexNormals.back().normalize();
	}
}

void Mesh::CalculateFacetNormals(){
	FacetNormals.reserve(Facets.size() / 3);
	for (int i = 0; i < Facets.size(); i += 3){
		Eigen::Vector3d& p0 = Vertices[Facets[i]];
		Eigen::Vector3d& p1 = Vertices[Facets[i + 1]];
		Eigen::Vector3d& p2 = Vertices[Facets[i + 2]];
		Eigen::Vector3d normal;
		CalculateTriangleNormal(p0, p1, p2, normal);
		FacetNormals.push_back(normal);
		//FacetNormals.back().normalize();
	}
}

void Mesh::CalculateTriangleNormal(
	const Eigen::Vector3d &p0,
	const Eigen::Vector3d &p1,
	const Eigen::Vector3d &p2,
	Eigen::Vector3d &out){
	Eigen::Vector3d v1 = p1 - p0;
	Eigen::Vector3d v2 = p2 - p1;
	if (v1.norm() <= 1e-6) v1 = p1 * 1e+9 - p0 * 1e+9;
	if (v2.norm() <= 1e-6) v2 = p2 * 1e+9 - p1 * 1e+9;
	out[0] = v1[1] * v2[2] - v1[2] * v2[1];
	out[1] = v1[2] * v2[0] - v1[0] * v2[2];
	out[2] = v1[0] * v2[1] - v1[1] * v2[0];
	out.normalize();
}

void Mesh::CalculateAdjacentVerticesPerVertex(){
	adjacentVerticesPerVertex.reserve(Vertices.size());
	for (int i = 0; i < Vertices.size(); ++i){
		std::vector<int>* list = new std::vector<int>();
		list->reserve(4);
		adjacentVerticesPerVertex.push_back(list);
	}
	for (int i = 0; i < Facets.size(); i += 3){
		int index0 = Facets[i];
		int index1 = Facets[i + 1];
		int index2 = Facets[i + 2];
		std::vector<int> *p0list = adjacentVerticesPerVertex[index0];
		std::vector<int> *p1list = adjacentVerticesPerVertex[index1];
		std::vector<int> *p2list = adjacentVerticesPerVertex[index2];
		if (std::find(p0list->begin(), p0list->end(), index1) == p0list->end()){
			p0list->push_back(index1);
		}
		if (std::find(p0list->begin(), p0list->end(), index2) == p0list->end()){
			p0list->push_back(index2);
		}

		if (std::find(p1list->begin(), p1list->end(), index0) == p1list->end()){
			p1list->push_back(index0);
		}
		if (std::find(p1list->begin(), p1list->end(), index2) == p1list->end()){
			p1list->push_back(index2);
		}

		if (std::find(p2list->begin(), p2list->end(), index0) == p2list->end()){
			p2list->push_back(index0);
		}
		if (std::find(p2list->begin(), p2list->end(), index1) == p2list->end()){
			p2list->push_back(index1);
		}
	}
}

void Mesh::CalculateAdjacentFacetsPerVertex(){
	adjacentFacetsPerVertex.reserve(Vertices.size());
	for (int i = 0; i < Vertices.size(); ++i){
		std::vector<int>* list = new std::vector<int>();
		list->reserve(4);
		adjacentFacetsPerVertex.push_back(list);
	}
	for (int i = 0; i < Facets.size(); i += 3){
		int index0 = Facets[i];
		int index1 = Facets[i + 1];
		int index2 = Facets[i + 2];
		std::vector<int> *t0list = adjacentFacetsPerVertex[index0];
		std::vector<int> *t1list = adjacentFacetsPerVertex[index1];
		std::vector<int> *t2list = adjacentFacetsPerVertex[index2];
		int t = i / 3;
		t0list->push_back(t);
		t1list->push_back(t);
		t2list->push_back(t);
	}
}