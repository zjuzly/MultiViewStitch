#include "Processor.h"
#include "ParamParser.h"

int main(int argc, char *argv[]){
	std::cout << !!0 << std::endl;
	std::cout << !!1 << std::endl;
	ParamParser::setParamFromFile(argv[1]);
	Processor pro;
	//pro.SetParamFromFile(argv[1]);
	pro.AlignmentSeq();
	return 0;
}