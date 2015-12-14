#include <windows.h>
#include "Processor.h"
#include "ParamParser.h"

int main(int argc, char *argv[]){
	//MoveFile("E:\\kinect\\zhouly\\MultiViewStitch\\data\\1449053713426\\front\\DATA\\_depth7.raw",  \
		"E:\\kinect\\zhouly\\MultiViewStitch\\data\\1449053713426\\front\\DATA\\TMP\\_depth7.raw");
	ParamParser::setParamFromFile(argv[1]);
	Processor pro;
	//pro.SetParamFromFile(argv[1]);
	pro.AlignmentSeq();
	return 0;
}