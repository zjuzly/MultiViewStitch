#include "Depth2Model.h"
#include "Vector.h"
#include <fstream>

Depth2Model::Depth2Model(double fMinDepth, double fMaxDepth, double fSmoothThreshold) :
	m_fMinDepth(fMinDepth), m_fMaxDepth(fMaxDepth), m_fSmoothThreshold(fSmoothThreshold){}

void Depth2Model::SaveModel(
	const std::vector<double> &depth,
	const Camera &cam,
	const std::string desPath){

	std::cout << "Save to Model" << std::endl;

	int width = cam.W();
	int height = cam.H();

#if WRITE_MESH
	std::ofstream ofs;
	ofs.open(desPath.c_str(), std::ofstream::out | std::ofstream::trunc);
	if(!ofs.is_open()){
		std::cerr << "ERROR: Can not open the object file: " << desPath << std::endl;
		exit(-1);
	}
#endif
	std::vector<std::vector<int>> tab(height, std::vector<int>(width, 0));
	int tabNum = 0;
	Eigen::Vector3d p3d;
	//Vec3d p3d;
	for(int y = 0; y < height; ++y){
		for(int x = 0; x < width; ++x){
			if(depth[y * width + x] > 0){
				if(depth[y * width + x] > m_fMaxDepth || depth[y * width + x] < m_fMinDepth)	continue;
				tab[y][x] = ++tabNum;
				cam.GetWorldCoordFromImg(x, y, depth[y * width + x], p3d);
				point3d.push_back(p3d);
				texIndex.push_back(y * width + x);
#if WRITE_MESH
				ofs << "v " << p3d[0] << " " << p3d[1] << " " << p3d[2] << std::endl;
#endif
			}
		}
	}
	std::cout << "Total of " << tabNum << " vertices" << std::endl; 
	float threshold = m_fSmoothThreshold * (m_fMaxDepth - m_fMinDepth) / 100;
	for(int y = 0; y < height - 1; ++y){
		for(int x = 0; x < width - 1; ++x){
			if(tab[y][x] != 0 && tab[y + 1][x + 1] != 0 && fabs(depth[y * width + x] - depth[(y + 1) * width + x + 1]) <= threshold){
				if(tab[y + 1][x] != 0 && 
					fabs(depth[y * width + x] - depth[(y + 1) * width + x]) <= threshold &&
					fabs(depth[(y + 1) * width + x + 1] - depth[(y + 1) * width + x]) <= threshold){
					//facets.push_back(Vec3i(tab[y][x] - 1, tab[y + 1][x] - 1, tab[y + 1][x + 1] - 1));
					facets.push_back(Eigen::Vector3i(tab[y][x] - 1, tab[y + 1][x + 1] - 1, tab[y + 1][x] - 1));
#if WRITE_MESH
					ofs << "f " << tab[y][x] << " " << tab[y + 1][x] << " " << tab[y + 1][x + 1] << std::endl;
#endif
				}
				if(tab[y][x + 1] != 0 &&
					fabs(depth[y * width + x] - depth[y * width + x + 1]) <= threshold &&
					fabs(depth[(y + 1) * width + x + 1] - depth[y * width + x + 1]) <= threshold){
					//facets.push_back(Vec3i(tab[y][x] - 1, tab[y + 1][x + 1] - 1, tab[y][x + 1] - 1));
					facets.push_back(Eigen::Vector3i(tab[y][x] - 1, tab[y][x + 1] - 1, tab[y + 1][x + 1] - 1));
#if WRITE_MESH
					ofs << "f " << tab[y][x] << " " << tab[y + 1][x + 1] << " " << tab[y][x + 1] << std::endl;
#endif
				}
			}
		}
	}
#if WRITE_MESH
	ofs.close();
	std::cout << "Save Done!" << std::endl;
#endif
}