#ifndef GEOMETRYREC_H
#define GEOMETRYREC_H

#include <GeoRec.h>
#include <string>

class GeometryRec{
public:
	GeometryRec(){ gr = new ZJU::GeoRec; }
	GeometryRec(const GeometryRec &other){
		if (gr != NULL){
			delete gr;
			gr = NULL;
		}
		gr = new ZJU::GeoRec(*other.gr);
	}
	GeometryRec& operator = (const GeometryRec &other){
		GeometryRec tmp(other);
		std::swap(*this, tmp);
		return *this;
	}
	~GeometryRec(){ 
		if (gr != NULL){
			delete gr;
			gr = NULL;
		}
	}
	void Init(
		const std::string actfile,
		const double dspMax,
		const double dspMin,
		const int ptSampRds,
		const double dspErr,
		const double minConf,
		const int nbrFrmNum,
		const int nbrFrmStep,
		const int startFrmIdx,
		const int endFrmIdx,
		const double edgeSzThres,
		const int maxPsDep,
		const int minPsDep
		);
	bool RunPointSample();
	bool RunPoisson(const std::string filename);
private:
	ZJU::GeoRec *gr;
};

#endif