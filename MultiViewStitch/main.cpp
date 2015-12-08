#include "Processor.h"
#include "ParamParser.h"

int main(int argc, char *argv[]){
	ParamParser::setParamFromFile(argv[1]);
	Processor pro;
	//pro.SetParamFromFile(argv[1]);
	pro.AlignmentSeq();
	return 0;
}