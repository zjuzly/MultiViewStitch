#ifndef PARAM_PARSER_H
#define PARAM_PARSER_H

#include <string>
#include <vector>

class ParamParser{
public:
	static void setParamFromFile(const std::string filename);
public:
	//parameters for alignment
	static bool writeMesh;
	static bool isSegment;

	static int view_count; //1
	static int min_match_count; //10
	static int iter_num; //100
	static int sample_interval; //40
	static int ssd_win; //7
	static int reproj_err; //4
	static int axis; //x: 0, y: 1, z: 2

	static double rot_angle;
	static double ssd_err;
	static double pixel_err;
	static double adapt_pixel_err_ratio;
	static double distmax; //0.5
	static double ratiomax; //0.5
	static double hl_margin_ratio; //0.15
	static double hr_margin_ratio; //0.15
	static double vl_margin_ratio; //0.15
	static double vr_margin_ratio; //0.15
	static double m_fMinDsp;
	static double m_fMaxDsp;

	static std::vector<std::string> imgdirs;

	//parameters for reconstruction
	static int sample_radius;
	static int nbr_frm_num;
	static int nbr_frm_step;
	static int psn_dpt_max;
	static int psn_dpt_min;

	//static double dsp_min;
	//static double dsp_max;
	static double dsp_err;
	static double conf_min;
	static double edge_sz_thres;

	static double dist_thres;
	static double smooth_thres;
};

#endif