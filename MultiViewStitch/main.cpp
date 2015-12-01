#include "Processor.h"

int main(int argc, char *argv[]){
	Processor pro;
	pro.SetParamFromFile(argv[1]);
	pro.AlignmentSeq();
	return 0;
}