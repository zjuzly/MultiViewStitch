#include "GeometryRec.h"
#include "ParamParser.h"
#include "Utils.h"

#ifdef _DEBUG
#pragma comment(lib, "E:/3rdPart/VideoMat/zly/lib/GeoRec_d.lib")
#else
#pragma comment(lib, "E:/3rdPart/VideoMat/zly/lib/GeoRec.lib")
#endif

void GeometryRec::Init(
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
	){
	gr->SetActFile((char*)actfile.c_str());
	gr->SetDspMax((float)dspMax);
	gr->SetDspMin((float)dspMin);
	gr->SetPtSampRds(ptSampRds);
	gr->SetMaxDspErr((float)dspErr);
	gr->SetMinConf((float)minConf);
	gr->SetNbrFrmNum(nbrFrmNum);
	gr->SetNbrFrmStep(nbrFrmStep);
	gr->SetStartFrmIdx(startFrmIdx);
	gr->SetEndFrmIdx(endFrmIdx);
	gr->SetEdgeSzThres((float)edgeSzThres);
	gr->SetMaxPsDep(maxPsDep);
	gr->SetMinPsDep(minPsDep);

	gr->Init();
}

bool GeometryRec::RunPointSample(){
	return gr->RunPointSample();
}

bool GeometryRec::RunPoisson(const std::string filename){
	return gr->RunPoisson(filename.c_str());
}