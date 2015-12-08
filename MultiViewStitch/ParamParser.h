#ifndef PARAM_PARSER_H
#define PARAM_PARSER_H

#include <string>
#include <vector>

class ParamParser{
public:
	static void setParamFromFile(const std::string filename);
public:
	static bool writeMesh;

	static int view_count; //1
	static int min_match_count; //10
	static int iter_num; //100
	static int sample_interval; //40
	static int ssd_win; //7
	static int axis; //x: 0, y: 1, z: 2

	static double rot_angle;
	static double ssd_err;
	static double pixel_err;
	static double distmax; //0.5
	static double ratiomax; //0.5
	static double hl_margin_ratio; //0.15
	static double hr_margin_ratio; //0.15
	static double vl_margin_ratio; //0.15
	static double vr_margin_ratio; //0.15
	static double m_fMinDsp;
	static double m_fMaxDsp;

	static std::vector<std::string> imgdirs;
};

#endif