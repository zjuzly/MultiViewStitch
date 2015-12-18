#include "Processor\Processor.h"
#include "Parameter\ParamParser.h"

#include "Common\Utils.h"

void Helper(){
	std::cout << "MultiViewStitch.exe " << "-a <Integer>" << std::endl;
}

int main(int argc, char *argv[]){

	if (argc != 4){
		Helper();
		return 0;
	}

	ParamParser::setParamFromFile(argv[1]);
	Processor pro;

	if(std::string(argv[2]) == "-a"){
		if(std::atoi(argv[3]) == 1){
			pro.AlignmentSeq();
		}
		else{
			pro.Render(argc, argv);
		}
	}
//#if 0
//	//pro.Deform();
//	pro.AlignmentSeq();
//#else
//	pro.Render(argc, argv);
//#endif
	return 0;
}