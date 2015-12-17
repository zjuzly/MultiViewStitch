#include "Processor\Processor.h"
#include "Parameter\ParamParser.h"

#include "Common\Utils.h"

int main(int argc, char *argv[]){
	ParamParser::setParamFromFile(argv[1]);
	Processor pro;
#if 0
	//pro.Deform();
	pro.AlignmentSeq();
#else
	pro.Render(argc, argv);
#endif
	return 0;
}