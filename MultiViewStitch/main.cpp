#include "Processor\Processor.h"
#include "Parameter\ParamParser.h"
#include "Common\PlyObj.h"
#include "Alignment\Alignment.h"

int main(int argc, char *argv[]){

	//std::vector<Eigen::Vector3d> point3d, normal3d;
	//std::vector<int> facets;
	//ReadObj("./Result/Model.obj", point3d, normal3d, facets);

	//Alignment align;
	//align.RemoveGround(point3d, normal3d, facets);
	//WriteObj("./Result/Model1.obj", point3d, normal3d, facets);
	//return 0;

	ParamParser::setParamFromFile(argv[1]);
	Processor pro;
	pro.AlignmentSeq();
	return 0;
}