#include "ParamParser.h"
#include <iostream>
#include <fstream>

bool ParamParser::writeMesh = false;

int ParamParser::view_count = 1;
int ParamParser::min_match_count = 5;
int ParamParser::iter_num = 100;
int ParamParser::sample_interval = 24;
int ParamParser::ssd_win = 7;
int ParamParser::axis = 0;

double ParamParser::rot_angle = 10.0;
double ParamParser::ssd_err = 16.0;
double ParamParser::pixel_err = 55.0;
double ParamParser::distmax = 0.7;
double ParamParser::ratiomax = 0.8;
double ParamParser::hl_margin_ratio = 0.1;
double ParamParser::hr_margin_ratio = 0.25;
double ParamParser::vl_margin_ratio = 0.33;
double ParamParser::vr_margin_ratio = 0.25;
double ParamParser::m_fMinDsp = 0.0001;
double ParamParser::m_fMaxDsp = 0.5;

std::vector<std::string> ParamParser::imgdirs = std::vector<std::string>();

void ParamParser::setParamFromFile(const std::string filename){
	std::ifstream ifs;
	ifs.open(filename.c_str(), std::ifstream::in);
	if (!ifs.is_open()){
		std::cerr << "ERROR: Open File " << filename << " Failed in File " << __FILE__ << ", Line " << __LINE__ << std::endl;
		exit(-1);
	}
	std::string tip;
	std::string imgPathListFile;
	while (ifs.peek() != EOF){
		ifs >> tip;
		if (tip.size() == 0 || (tip.size() > 0 && tip[0] == '#')) continue;
		if ("WriteMesh" == tip){ ifs >> writeMesh; }
		else if ("ViewCount" == tip){ ifs >> view_count; }
		else if ("MinMatchCount" == tip){ ifs >> min_match_count; }
		else if ("IterNum" == tip){ ifs >> iter_num; }
		else if ("SampleIterval" == tip){ ifs >> sample_interval; }
		else if ("SSDWin" == tip){ ifs >> ssd_win; }
		else if ("Axis" == tip){ ifs >> axis; }
		else if ("RotAngle" == tip){ ifs >> rot_angle; }
		else if ("PixelError" == tip){ ifs >> pixel_err; }
		else if ("SSDError" == tip){ ifs >> ssd_err; }
		else if ("DistMax" == tip){ ifs >> distmax; }
		else if ("RatioMax" == tip){ ifs >> ratiomax; }
		else if ("HLMarginRatio" == tip){ ifs >> hl_margin_ratio; }
		else if ("VLMarginRatio" == tip){ ifs >> vl_margin_ratio; }
		else if ("HRMarginRatio" == tip){ ifs >> hr_margin_ratio; }
		else if ("VRMarginRatio" == tip){ ifs >> vr_margin_ratio; }
		else if ("MinDsp" == tip){ ifs >> m_fMinDsp; }
		else if ("MaxDsp" == tip){ ifs >> m_fMaxDsp; }
		else if ("ImgPathList" == tip){ ifs >> imgPathListFile; }
	}
	ifs.close();

	ifs.open(imgPathListFile.c_str(), std::ifstream::in);
	if (!ifs.is_open()){
		std::cerr << "ERROR: Open File " << imgPathListFile << " Failed in File " << __FILE__ << ", Line " << __LINE__ << std::endl;
		exit(-1);
	}
	std::vector<std::string>().swap(imgdirs);
	while (ifs.peek() != EOF){
		std::string imgdir;
		ifs >> imgdir;
		if (imgdir.size() == 0) continue;
		else if (imgdir[0] == '#') continue;
		imgdirs.push_back(imgdir);
	}
	ifs.close();
}