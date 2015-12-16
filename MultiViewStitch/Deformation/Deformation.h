#ifndef DEFORMATION_H
#define DEFORMATION_H

#define CGAL_EIGEN3_ENABLED

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Polyhedron_traits_with_normals_3.h>
#include <CGAL/HalfedgeDS_list.h>
#include <CGAL/HalfedgeDS_default.h>
#include <CGAL/IO/print_wavefront.h>
#include <CGAL/exceptions.h>

#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_length_cost.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Midpoint_placement.h>

#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/boost/graph/properties_Polyhedron_3.h>

#include <CGAL/Surface_mesh_deformation.h>

#include <algorithm>
#include <vector>

#include "PropertyMap.h"
#include "../PlyObj/PlyObj.h"

#pragma comment(lib, "D:/Program Files/CGAL-4.6/auxiliary/gmp/lib/libgmp-10.lib")
#pragma comment(lib, "D:/Program Files/CGAL-4.6/auxiliary/gmp/lib/libmpfr-4.lib")

#if _DEBUG
#pragma comment(lib, "D:/CGAL-MSVC12/lib/CGAL-vc120-mt-gd-4.6.lib")
#else
#pragma comment(lib, "D:/CGAL-MSVC12/lib/CGAL-vc120-mt-4.6.lib")
#endif

namespace SMS = CGAL::Surface_mesh_simplification;

typedef CGAL::Simple_cartesian<double>															Kernel;
//typedef CGAL::Polyhedron_traits_with_normals_3<Kernel>											Traits;
//typedef CGAL::Polyhedron_3<Traits>																CgalPolyhedron;
typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3>							CgalPolyhedron;
typedef boost::graph_traits<CgalPolyhedron>::vertex_descriptor									vertex_descriptor;
typedef boost::graph_traits<CgalPolyhedron>::vertex_iterator									vertex_iterator;
typedef CGAL::Surface_mesh_deformation<CgalPolyhedron>											Surface_mesh_deformation;

template <class HDS>
class Build_triangle : public CGAL::Modifier_base<HDS>{
public:
	Build_triangle(const std::vector<Eigen::Vector3d> &points_, const std::vector<int> &facets_){
		std::copy(points_.begin(), points_.end(), std::back_inserter(points));
		std::copy(facets_.begin(), facets_.end(), std::back_inserter(facets));
	}
	void operator()(HDS& hds){
		typedef typename HDS::Vertex Vertex;
		typedef typename Vertex::Point Point;
		
		int m_iVertices = (int)points.size();
		int m_iFacets = (int)(facets.size() / 3);

		CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);

		B.begin_surface(m_iVertices, m_iFacets, (m_iVertices + m_iFacets - 2) * 2);
		for(int i = 0; i < m_iVertices; ++i){
			B.add_vertex(Point(points[i][0], points[i][1], points[i][2]));
		}
	
		for(int i = 0; i < m_iFacets; ++i){
			B.begin_facet();
			B.add_vertex_to_facet(facets[3 * i]);
			B.add_vertex_to_facet(facets[3 * i + 1]);
			B.add_vertex_to_facet(facets[3 * i + 2]);
			B.end_facet();
		}
		B.end_surface();
	}
private:
	std::vector<Eigen::Vector3d> points;
	std::vector<int> facets;
};

template<class TPolyhedron, class TFacetNormals>
struct VertexNormal{
public:
	typedef typename TPolyhedron::Vertex									Vertex;
	typedef typename TPolyhedron::Facet										Facet;
	typedef typename TPolyhedron::Traits::Vector_3							Vector3;
	typedef typename TPolyhedron::Halfedge_around_vertex_const_circulator	HalfEdgeConstCirculator;
	typedef Vector3 result_type;
public:
	VertexNormal(const TFacetNormals& facetNormals_) : facetNormals(facetNormals_){}
	inline Vector3 operator()(const Vertex& v) const{
		Vector3					n = CGAL::NULL_VECTOR;
		HalfEdgeConstCirculator s = v.vertex_begin();
		HalfEdgeConstCirculator e = s;
		CGAL_For_all(s, e){
			if(!s->is_border()){
				n = n + facetNormals.value(&*(s->facet()));
			}
		}
		return n / std::sqrt(n * n);
	}
public:
	const TFacetNormals& facetNormals;
};

template<class TPolyhedron>
struct FacetNormal{
public:
	typedef typename TPolyhedron::Facet					Facet;
	typedef typename TPolyhedron::Halfedge_const_handle HalfedgeConstHandle;
	typedef typename TPolyhedron::Traits::Point_3		Point3;
	typedef typename TPolyhedron::Traits::Vector_3		Vector3;
	typedef Vector3 result_type;
public:
	inline Vector3 operator()(const Facet& f)const{
		HalfedgeConstHandle h  = f.halfedge();
		Point3				p1 = h->vertex()->point();
		Point3				p2 = h->next()->vertex()->point();
		Point3				p3 = h->next()->next()->vertex()->point();
		Vector3				n  = CGAL::cross_product(p2 - p1, p3 - p1);
		return n / std::sqrt(n * n);
	}
};

template<class TPolyhedron, class TVertexNormals, class TFacetNormals>
void ComputeVertexNormals(const TPolyhedron& polyhedron, TVertexNormals* vertexNormals, const TFacetNormals& facetNormals){
	if(0 == vertexNormals)	return;
	typename TPolyhedron::Vertex_const_iterator _begin = polyhedron.vertices_begin();
	typename TPolyhedron::Vertex_const_iterator _end	  = polyhedron.vertices_end();
	VertexNormal<TPolyhedron, TFacetNormals> _vertexNormals(facetNormals);
	CGAL_For_all(_begin, _end){
		vertexNormals->setValue(&*_begin, _vertexNormals(*_begin));
	}
}

template<class TPolyhedron, class TFacetNormals>
void ComputeFacetNormals(const TPolyhedron& polyhedron, TFacetNormals* facetNormals){
	if(0 == facetNormals)	return;
	typename TPolyhedron::Facet_const_iterator _begin = polyhedron.facets_begin();
	typename TPolyhedron::Facet_const_iterator _end	  = polyhedron.facets_end();
	FacetNormal<TPolyhedron> _facetNormal;
	CGAL_For_all(_begin, _end){
		facetNormals->setValue(&*_begin, _facetNormal(*_begin));
	}
}

template<class TPoly>
void importOBJ(const std::string filename, TPoly* polyhedron){
	if(polyhedron){
		try{
			std::vector<Eigen::Vector3d> points, normals;
			std::vector<int> facets;
			ReadObj(filename, points, normals, facets);
			
			typedef typename TPoly::HalfedgeDS HDS;
			Build_triangle<HDS> BT(points, facets);
			polyhedron->delegate(BT);
			if(!polyhedron->is_valid()){
				throw CGAL::Assertion_exception("", "", "", 0, "");
			}
		}catch(const CGAL::Assertion_exception&){
			std::string _msg = "importOBJ: Error loading " + filename;
			std::cerr << _msg << std::endl;
			exit(-1);
		}
	}
}

template<class TPoly>
void exportOBJ(const std::string filename, TPoly* polyhedron){
	if(polyhedron){
		std::ofstream ofs(filename.c_str());
		if(!ofs.is_open()){
			std::string _msg = "exportOBJ: Error exporting " + filename;
			std::cerr << _msg << std::endl;
			exit(-1);
		}
		//CGAL::print_polyhedron_wavefront(ofs, *polyhedron);

		typedef PropertyMap<const CgalPolyhedron::Facet*, CgalPolyhedron::Traits::Vector_3> FacetNormalPM;
		FacetNormalPM _facetNormals;
		ComputeFacetNormals(*polyhedron, &_facetNormals);

		typedef PropertyMap<const CgalPolyhedron::Vertex*, CgalPolyhedron::Traits::Vector_3> VertexNormalPM;
		VertexNormalPM _vertexNormals;
		ComputeVertexNormals(*polyhedron, &_vertexNormals, _facetNormals);

		int i = 0;
		std::vector<Eigen::Vector3d> points(polyhedron->size_of_vertices());
		std::vector<Eigen::Vector3d> normals(polyhedron->size_of_vertices());
		//vertex_iterator vb, ve;
		//boost::tie(vb, ve) = vertices(*polyhedron);
		CgalPolyhedron::Vertex_iterator p_it = polyhedron->vertices_begin();
		for(i = 0; p_it != polyhedron->vertices_end(); ++p_it, ++i){
			CgalPolyhedron::Point_3 p3 = p_it->point();
			points[i] = Eigen::Vector3d(p3[0], p3[1], p3[2]);
			CgalPolyhedron::Traits::Vector_3 n3 = _vertexNormals.value(&*p_it);
			normals[i] = Eigen::Vector3d(n3[0], n3[1], n3[2]);
		}

		std::vector<int> facets(polyhedron->size_of_facets() * 3);
		CgalPolyhedron::Facet_const_iterator f_it = polyhedron->facets_begin();
		for(i = 0; f_it != polyhedron->facets_end(); ++f_it, i += 3){
			CgalPolyhedron::Halfedge_around_facet_const_circulator c_it = f_it->facet_begin();
			std::vector<int> idx;
			do{
				idx.push_back((int)c_it->vertex()->id());
			}while(++c_it != f_it->facet_begin());
			facets[i]	  = idx[0];
			facets[i + 1] = idx[1];
			facets[i + 2] = idx[2];
		}

		WriteObj(filename, points, normals, facets);
	}
}


class Deformation{
public:
	Deformation(){}
	Deformation(const std::string filename);
	Deformation(
		const std::vector<Eigen::Vector3d> &points_,
		const std::vector<Eigen::Vector3d> &normals_,
		const std::vector<int> &facets_);

	CgalPolyhedron Polyhedron(){ return poly; }

	void Simplification();
	void UniformSampling();
	void Deform(
		const std::vector<Eigen::Vector3d> &tpts,
		const std::vector<Eigen::Vector3d> &tnormals,
		const double threshold);
	void Deform(
		const std::vector<Eigen::Vector3d> &tpts,
		const std::vector<Eigen::Vector3d> &tnormals,
		const double projLenErr,
		const double projDistErr);
private:
	std::vector<std::vector<std::pair<int, double>>> KNearestNeighbor(int K);
private:
	CgalPolyhedron poly;
	std::vector<Eigen::Vector3d> normals;
	std::vector<int> sampIdx;
};

//--Debug
inline void cgalTest(){
	std::vector<Eigen::Vector3d> t_points, s_points;
	std::vector<Eigen::Vector3d> t_normals, s_normals;
	std::vector<int> t_facets, s_facets;
	ReadObj("./data/zly/aligned1.obj", s_points, s_normals, s_facets);
	ReadObj("./data/zly/ModelTrim.obj", t_points, t_normals, t_facets);

	Deformation deform(s_points, s_normals, s_facets);
	//deform.Simplification();
	//exportOBJ("./data/zly/cgal_aligned1.obj", &deform.Polyhedron());
	
	deform.UniformSampling();

	deform.Deform(t_points, t_normals, 100.0, 100.0);
	
	exportOBJ("./data/zly/deform.obj", &deform.Polyhedron());
}
//-------

#endif