#include "Processor\Processor.h"
#include "Parameter\ParamParser.h"
#include "PlyObj\PlyObj.h"
#include "Common\Utils.h"
#include "Alignment\Alignment.h"

int main(int argc, char *argv[]){

	ParamParser::setParamFromFile(argv[1]);
#if 0
	Processor pro;
	pro.AlignmentSeq();
#else
	std::vector<std::vector<Camera>> cameras(ParamParser::imgdirs.size());
	for (int k = 0; k < cameras.size(); ++k){
		std::vector<std::string> actsfiles = ScanNSortDirectory(ParamParser::imgdirs[k].c_str(), "act");
		cameras[k] = LoadCalibrationFromActs(actsfiles[0]);
	}

	std::vector<Eigen::Vector3d> tgt, tgt_normals;
	std::vector<int> tgt_facets;
	ReadObj("./Result/Model.obj", tgt, tgt_normals, tgt_facets);

	std::vector<Eigen::Vector3d> src, src_normals;
	std::vector<int> src_facets;
	ReadObj("./Template/meanbody.obj", src, src_normals, src_facets);

	Eigen::Matrix3d R;
	Eigen::Vector3d t;
	cameras[0][0].GetRT(R, t);
	Alignment align;
	align.Align(src, src_normals, src_facets, tgt, tgt_normals, tgt_facets, R.transpose().col(2));
	//align.RemoveGround(point3d, normal3d, facets);
#endif
	return 0;
}