#include "../Common/Utils.h"
#include "../PlyObj/PlyObj.h"
#include "../Vector/Vector.h"
#include "../Camera/Camera.h"
#include "../FeatureProc/FeatureProc.h"
#include "../Solver/SRTSolver.h"
#include "../Processor/Processor.h"
#include "../Depth2Model/Depth2Model.h"
#include "../Alignment/Alignment.h"
#include "../Parameter/ParamParser.h"
#include "../Reconstruction/GeometryRec.h"
#include "../Deformation/Deformation.h"
#include "../Model2Depth/Model2Depth.h"
#include <algorithm>
#include <set>
#include <unordered_map>

#define PRINT_INFO

void Processor::LoadCameras(){
	cameras.resize(ParamParser::imgdirs.size());
	for (int k = 0; k < cameras.size(); ++k){
		std::vector<std::string> actsfiles = ScanNSortDirectory(ParamParser::imgdirs[k].c_str(), "act");
		cameras[k] = LoadCalibrationFromActs(actsfiles[0]);
	}
}

void Processor::CheckConsistency(const std::vector<std::vector<Camera>> &cameras){
	char fn[128];
	for (int k = 0; k < cameras.size(); ++k){
		int n = cameras[k].size();
		std::vector<std::vector<double>> depths(n);
		std::cout << "Load depth map: ";
		for (int i = 0; i < n; ++i){
			std::cout << i << " ";
			sprintf_s(fn, "%sDATA/_depth%d.raw", ParamParser::imgdirs[k].c_str(), i);
			LoadDepth(fn, depths[i], cameras[k][i].W(), cameras[k][i].H());
		}
		std::cout << std::endl;
		CreateDir(ParamParser::imgdirs[k] + "DATA/CHECK/");
		for (int i = 0; i < n; ++i){
			std::vector<double> depth_to_check = depths[i];
			Camera curcam = cameras[k][i];

			std::vector<std::vector<double>> refdepths;
			std::vector<Camera> refcams;
			std::cout << std::endl << "Image#" << i << " Depth Consistency check..." << std::endl;
			for (int j = 0; j < 3/*2 * ParamParser::nbr_frm_num + 1*/; ++j){
				int idx = i - 1/*ParamParser::nbr_frm_num*/ + j;
				if (idx >= 0 && idx < n && idx != i){
					refdepths.push_back(depths[idx]);
					refcams.push_back(cameras[k][idx]);
				}
			}
			CheckConsistencyCore(curcam, refcams, depth_to_check, refdepths);

			sprintf_s(fn, "%sDATA/CHECK/_depth%d.raw", ParamParser::imgdirs[k].c_str(), i);
			SaveDepth(fn, depth_to_check);
			sprintf_s(fn, "%sDATA/CHECK/_depth%d.jpg", ParamParser::imgdirs[k].c_str(), i);
			::RenderDepthMap(fn, depth_to_check, curcam.W(), curcam.H());
		}
	}
}

void Processor::CheckConsistencyCore(
	const Camera curcam,
	const std::vector<Camera> refcams,
	std::vector<double> &depth_out,
	const std::vector<std::vector<double>> &refdepth){
	int w = curcam.W();
	int h = curcam.H();
	int n = refdepth.size();
	for (int j = 0; j < h; ++j){
		if (j % 10 == 0) std::cout << j << " ";
		for (int i = 0; i < w; ++i){
			double &dp = depth_out[j * w + i];
			if (dp >= ParamParser::m_fMinDsp && dp <= ParamParser::m_fMaxDsp){
				Eigen::Vector3d p3d;
				curcam.GetWorldCoordFromImg(i, j, 1.0 / dp, p3d);
				int u, v;
				for (int k = 0; k < n; ++k){
					refcams[k].GetImgCoordFromWorld(p3d, u, v);
					if (CheckRange(u, v, refcams[k].W(), refcams[k].H())){
						if (refdepth[k][v * w + u] >= ParamParser::m_fMinDsp && refdepth[k][v * w + u] <= ParamParser::m_fMaxDsp){
							Eigen::Vector3d p3d_;
							refcams[k].GetWorldCoordFromImg(u, v, 1.0 / refdepth[k][v * w + u], p3d_);
							curcam.GetImgCoordFromWorld(p3d_, u, v);
							if (!CheckRange(u, v, w, h)){
								dp = 0.0f;
								break;
							}
							double reproj_err = sqrt(double((i - u) * (i - u) + (j - v) * (j - v)));
							if (reproj_err > ParamParser::reproj_err){
								dp = 0.0f;
								break;
							}
						}
						else{
							dp = 0.0f;
							break;
						}
					}
					else{
						dp = 0.0f;
						break;
					}
				}
			}
			else{
				dp = 0.0f;
			}
		}
	}
}

void Processor::RemoveDupPoints(
	const std::vector<Image3D> &im,
	const std::vector<Image3D> &jm,
	std::vector<std::vector<std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>>>> &matches){
	
	std::vector<std::vector<std::set<std::pair<Vec2i, Vec2i>>>> uniqueMatches(im.size(),
		std::vector<std::set<std::pair<Vec2i, Vec2i>>>(jm.size()));
	int w = im[0].GetWidth();
	int h = im[0].GetHeight();
	for (int i = 0; i < matches.size(); ++i){
		//int frmNo1 = i / view_count;
		int frmNo1 = i / ParamParser::view_count;
		for (int j = 0; j < matches[i].size(); ++j){
			//int frmNo2 = j / view_count;
			int frmNo2 = j / ParamParser::view_count;
			for (int k = 0; k < matches[i][j].size(); ++k){
				std::pair<Eigen::Vector2i, Eigen::Vector2i> &pr = matches[i][j][k];
				int idx1 = im[frmNo1].GetTexIndex(i % ParamParser::view_count, pr.first[0], pr.first[1]);
				int idx2 = jm[frmNo2].GetTexIndex(j % ParamParser::view_count, pr.second[0], pr.second[1]);
				if (idx1 != -1 && idx2 != -1 && im[frmNo1].IsValid(pr.first[0], pr.first[1]) && jm[frmNo2].IsValid(pr.second[0], pr.second[1])){
					uniqueMatches[frmNo1][frmNo2].insert(std::make_pair(Vec2i(idx1 % w, idx1 / w), Vec2i(idx2 % w, idx2 / w)));
				}
			}
		}
	}
	std::vector<std::vector<std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>>>>().swap(matches);
	matches.resize(im.size(), std::vector<std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>>>(jm.size()));

	//double pixelErr = sample_interval * sample_interval;
	double pixelErr = ParamParser::sample_interval * ParamParser::sample_interval;
	for (int i = 0; i < matches.size(); ++i){
		for (int j = 0; j < matches[i].size(); ++j){
			std::set<std::pair<Vec2i, Vec2i>>::iterator it = uniqueMatches[i][j].begin();
			for (; it != uniqueMatches[i][j].end(); ++it){
				matches[i][j].push_back(std::make_pair(Eigen::Vector2i(it->first[0], it->first[1]), Eigen::Vector2i(it->second[0], it->second[1])));
			}

			int k0 = 0, j0 = 0;
			for (int k = 0; k < matches[i][j].size(); ++k){
				bool flag = false;
				for (int k0 = 0; k0 < j0; ++k0){
					std::pair<Eigen::Vector2i, Eigen::Vector2i> &pr1 = matches[i][j][k0];
					std::pair<Eigen::Vector2i, Eigen::Vector2i> &pr2 = matches[i][j][k];
					Eigen::Vector2i x = pr1.first - pr2.first;
					Eigen::Vector2i y = pr1.second - pr2.second;
					if ((x[0] * x[0] + x[1] * x[1]) <= pixelErr || (y[0] * y[0] + y[1] * y[1]) <= pixelErr){
						flag = true;
						break;
					}
				}
				if (!flag) matches[i][j][j0++] = matches[i][j][k];
			}
			matches[i][j].resize(j0);
//#ifdef PRINT_INFO
//			std::cout << "Frame# " << i << ", " << j << " | " << matches[i][j].size() << " sift matches found" << std::endl;
//#endif
		}
	}
}

void Processor::RemoveOutliers(
	const Image3D &im,
	const Image3D &jm,
	double &inlier_ratio_out,
	double &err_out,
	std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>> &matches){

	int size = matches.size();
	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> matchPoints(size);

	for (int i = 0; i < size; ++i){
		const std::pair<Eigen::Vector2i, Eigen::Vector2i> &pr = matches[i];
		Eigen::Vector3d p1 = im.GetPoint(pr.first[0], pr.first[1]);
		Eigen::Vector3d p2 = jm.GetPoint(pr.second[0], pr.second[1]);
		matchPoints[i] = std::make_pair(p1, p2);
	}
	double scale = 1.0, err = HUGE_VAL, inlier_ratio = 0.0;
	Eigen::Matrix3d R;
	Eigen::Vector3d t;
#if 1
	double ratio = 1.0;
	for (int k = 0; k < 3; ++k){
		double scale_;
		Eigen::Matrix3d R_;
		Eigen::Vector3d t_;
		SRTSolver solver(200);
		solver.SetPrintFlag(false);
		solver.SetInput(matchPoints, im.GetCamera(), jm.GetCamera());
		solver.EstimateTransformRansac(scale_, R_, t_);

		std::vector<double> dists(size);
		double err_inlier_ = 0.0, err_all_ = 0.0;
		int newSize = 0;
		for (int i = 0; i < size; ++i){
			const Eigen::Vector3d &p1 = matchPoints[i].first;
			const Eigen::Vector3d &p2 = matchPoints[i].second;

			Eigen::Vector3d tp = scale_ * R_ * p1 + t_;
			int u1, v1, u2, v2;
			jm.GetCamera().GetImgCoordFromWorld(tp, u1, v1);
			jm.GetCamera().GetImgCoordFromWorld(p2, u2, v2);

			Eigen::Vector3d tp_ = 1.0 / scale_ * R_.transpose() * (p2 - t_);
			int u1_, v1_, u2_, v2_;
			im.GetCamera().GetImgCoordFromWorld(tp_, u2_, v2_);
			im.GetCamera().GetImgCoordFromWorld(p1, u1_, v1_);

			//double pixel_err_ = (sqrt((u1 - u2) * (u1 - u2) + (v1 - v2) * (v1 - v2)) + \
				sqrt((u1_ - u2_) * (u1_ - u2_) + (v1_ - v2_) * (v1_ - v2_))) * 0.5;
			//err_all_ += pixel_err_;
			double pixel_err1 = sqrt((u1 - u2) * (u1 - u2) + (v1 - v2) * (v1 - v2));
			double pixel_err2 = sqrt((u1_ - u2_) * (u1_ - u2_) + (v1_ - v2_) * (v1_ - v2_));
			err_all_ += (pixel_err1 + pixel_err2) * 0.5;

			//if (pixel_err_ <= ParamParser::pixel_err)
			if (pixel_err1 <= ParamParser::pixel_err * ratio && pixel_err2 <= ParamParser::pixel_err * ratio)
			{
				matchPoints[newSize] = matchPoints[i];
				matches[newSize++] = matches[i];
				//err_inlier_ += pixel_err_;
				err_inlier_ += (pixel_err1 + pixel_err2) * 0.5;
			}
		}
		ratio *= ParamParser::adapt_pixel_err_ratio;
		matchPoints.resize(newSize);
		matches.resize(newSize);
		//if(newSize > 0) err = err_all_ / newSize;
		err = err_all_ / size;

		//err_all_ /= size;
		//if (err > err_all_) err = err_all_;
		//if (newSize > 0){
		//	err_inlier_ /= newSize;
		//	if (inlier_ratio < (err_inlier_ / err_all_)){
		//		inlier_ratio = err_inlier_ / err_all_;
		//		scale = scale_;
		//		R = R_;
		//		t = t_;
		//	}
		//}
		size = newSize;
		if (newSize < 3 || fabs(inlier_ratio - 1.0) <= 1e-9) break;
	}
#else
	SRTSolver solver(200);
	solver.SetPrintFlag(false);
	solver.SetInput(matchPoints, im.GetCamera(), jm.GetCamera());
	solver.EstimateTransform(scale, R, t);
	err = solver.ResidualError(scale, R, t);
#endif
	inlier_ratio_out = inlier_ratio;
	err_out = err;
}

double Processor::RemoveOutliersParsac(
	const Image3D &im,
	const Image3D &jm,
	std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>> &matches){

	int size = matches.size();
	if (size < 8) return HUGE_VAL;

	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> matchPoints(size);
	for (int i = 0; i < size; ++i){
		const std::pair<Eigen::Vector2i, Eigen::Vector2i> &pr = matches[i];
		Eigen::Vector3d p1 = im.GetPointCam(pr.first[0], pr.first[1]);
		Eigen::Vector3d p2 = jm.GetPointCam(pr.second[0], pr.second[1]);
		matchPoints[i] = std::make_pair(Eigen::Vector3d(p1[0] / p1[2], p1[1] / p1[2], 1.0), Eigen::Vector3d(p2[0] / p2[2], p2[1] / p2[2], 1.0));
	}

	const double pixelErr = 0.3;
	double areaSize1 = HUGE_VAL, areaSize2 = HUGE_VAL;
	Eigen::Matrix3d Ebest;

	//Find a hypothesis with ransac framework
	for (int iter = 0; iter < 50; ++iter){
		int idx[8];
		Shuffle(idx, size, 8);
		Eigen::MatrixXd Y(8, 9);
		Eigen::Vector3d y1, y2;
		for (int i = 0; i < 8; ++i){
			y1 = matchPoints[idx[i]].first;
			y2 = matchPoints[idx[i]].second;
			Y(i, 0) = y2(0) * y1(0);
			Y(i, 1) = y2(0) * y1(1);
			Y(i, 2) = y2(0);
			Y(i, 3) = y2(1) * y1(0);
			Y(i, 4) = y2(1) * y1(1);
			Y(i, 5) = y2(1);
			Y(i, 6) = y1(0);
			Y(i, 7) = y1(1);
			Y(i, 8) = 1.0;
		}
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::MatrixXd U = svd.matrixU();
		Eigen::MatrixXd V = svd.matrixV();
		Eigen::Matrix3d E_;
		for (int i = 0; i < 3; ++i){
			E_(i, 0) = V.col(8)(3 * i);
			E_(i, 1) = V.col(8)(3 * i + 1);
			E_(i, 2) = V.col(8)(3 * i + 2);
		}
		Eigen::JacobiSVD<Eigen::Matrix3d> svd_(E_, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U_ = svd_.matrixU();
		Eigen::Matrix3d V_ = svd_.matrixV();
		Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
		S(2, 2) = 0.0;
		Eigen::Matrix3d E = U_ * S * V_.transpose();

		std::vector<Eigen::Vector2d> pixelCoords1, pixelCoords2;
		Eigen::Vector2d baryCenter1(0.0, 0.0), baryCenter2(0.0, 0.0);
		for (int i = 0; i < size; ++i){
			double err_ = fabs(matchPoints[i].second.adjoint() * E * matchPoints[i].first);
			if (err_ <= pixelErr){ //inlier
				const std::pair<Eigen::Vector2i, Eigen::Vector2i> &pr = matches[i];
				pixelCoords1.push_back(Eigen::Vector2d(pr.first[0], pr.first[1]));
				pixelCoords2.push_back(Eigen::Vector2d(pr.second[0], pr.second[1]));
				baryCenter1 = baryCenter1 + pixelCoords1.back();
				baryCenter2 = baryCenter2 + pixelCoords2.back();
			}
		}
		if (pixelCoords1.size() > 1){
			baryCenter1 /= pixelCoords1.size();
			baryCenter2 /= pixelCoords2.size();

			Eigen::Matrix2d C1 = Eigen::Matrix2d::Zero();
			Eigen::Matrix2d C2 = Eigen::Matrix2d::Zero();
			for (int i = 0; i < pixelCoords1.size(); ++i){
				Eigen::Vector2d dif1 = (pixelCoords1[i] - baryCenter1);
				C1 = C1 + dif1 * dif1.transpose();
				Eigen::Vector2d dif2 = (pixelCoords2[i] - baryCenter2);
				C2 = C2 + dif2 * dif2.transpose();
			}
			C1 = C1 / (pixelCoords1.size() - 1);
			C2 = C2 / (pixelCoords2.size() - 1);
			if (std::sqrt(C1.determinant()) <= areaSize1 && std::sqrt(C2.determinant()) <= areaSize2){
				areaSize1 = std::sqrt(C1.determinant());
				areaSize2 = std::sqrt(C2.determinant());
				Ebest = E;
			}
		}
		//std::cout << "Inliers: " << pixelCoords1.size() << " / " << size << std::endl;
	}
	//std::cout << "Hypothesis: " << std::endl << Ebest << std::endl;

	int newSize = 0;
	double err = 0;
	for (int i = 0; i < size; ++i){
		double err_ = fabs(matchPoints[i].second.adjoint() * Ebest * matchPoints[i].first);
		if (err_ <= pixelErr){ //inlier
			matches[newSize++] = matches[i];
		}
		err += err_;
	}
	matches.resize(newSize);
	std::cout << "+-----------------------------------------------+" << std::endl;
	std::cout << "Residual Error£º " << err << std::endl;
	std::cout << "Removed outliers£º " << size - newSize << " | " << size << std::endl;
	std::cout << "+-----------------------------------------------+" << std::endl;
	return err / size;
	//return (areaSize1 + areaSize2) * 0.5;
}

void Processor::CalcSimilarityTransformation(
	double &scale,
	double R[3][3],
	double t[3]){

	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> matchPoints;
#pragma region //Generate Views
	std::string actsfile1 = ParamParser::imgdirs[0] + "seq.act";
	std::vector<Camera> cameras1 = LoadCalibrationFromActs(actsfile1);
	std::vector<Image3D> im(cameras1.size(), Image3D(/*view_count, axis, rot_angle*/));
	CreateDir(ParamParser::imgdirs[0] + "Views/");
	for (int i = 0; i < im.size(); ++i){
		char imgpath[128], rawpath[128];
		sprintf_s(imgpath, "%s%05d.jpg", ParamParser::imgdirs[0].c_str(), i);
		sprintf_s(rawpath, "%sDATA/_depth%d.raw", ParamParser::imgdirs[0].c_str(), i);
		im[i].LoadModel(imgpath, rawpath, cameras1[i]);
	}

	std::string actsfile2 = ParamParser::imgdirs[1] + "seq.act";
	std::vector<Camera> cameras2 = LoadCalibrationFromActs(actsfile2);
	std::vector<Image3D> jm(cameras2.size(), Image3D(/*view_count, axis, rot_angle*/));
	CreateDir(ParamParser::imgdirs[1] + "Views/");
	for (int i = 0; i < (int)jm.size(); ++i){
		char imgpath[128], rawpath[128];
		sprintf_s(imgpath, "%s%05d.jpg", ParamParser::imgdirs[1].c_str(), i);
		sprintf_s(rawpath, "%sDATA/_depth%d.raw", ParamParser::imgdirs[1].c_str(), i);
		jm[i].LoadModel(imgpath, rawpath, cameras2[i]);
	}
#pragma endregion

#pragma region //Feature Detecting
	CreateDir(ParamParser::imgdirs[0] + "Views/Sift/");
	char fn[128];
	std::vector<std::string> proj_view_path1;// = ScanNSortDirectory((imgdir1 + "Views/").c_str(), "jpg");
	for (int i = 0; i < cameras1.size(); ++i){
		for (int j = 0; j < ParamParser::view_count; ++j){
			sprintf_s(fn, "%sViews/proj%d_%d.jpg", ParamParser::imgdirs[0].c_str(), i, j);
			proj_view_path1.push_back(fn);
		}
	}
	std::vector<std::vector<SiftGPU::SiftKeypoint>> keys1;
	std::vector<std::vector<float>> descs1;
	FeatureProc::DetectFeature(proj_view_path1, /*hl_margin_ratio, vl_margin_ratio, hr_margin_ratio, vr_margin_ratio, */keys1, descs1);

	CreateDir(ParamParser::imgdirs[1] + "Views/Sift/");
	std::vector<std::string> proj_view_path2;// = ScanNSortDirectory((imgdir2 + "Views/").c_str(), "jpg");
	for (int i = 0; i < cameras2.size(); ++i){
		for (int j = 0; j < ParamParser::view_count; ++j){
			sprintf_s(fn, "%sViews/proj%d_%d.jpg", ParamParser::imgdirs[1].c_str(), i, j);
			proj_view_path2.push_back(fn);
		}
	}
	std::vector<std::vector<SiftGPU::SiftKeypoint>> keys2;
	std::vector<std::vector<float>> descs2;
	FeatureProc::DetectFeature(proj_view_path2, /*hl_margin_ratio, vl_margin_ratio, hr_margin_ratio, vr_margin_ratio, */keys2, descs2);
#pragma endregion

#pragma region //Feature Matching
	std::vector<std::vector<std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>>>> matches;
	FeatureProc::MatchFeature(keys1, keys2, descs1, descs2, /*distmax, ratiomax, */matches);
	RemoveDupPoints(im, jm, matches);

	double err = std::numeric_limits<double>::max();
	std::cout << "Remove Outliers..." << std::endl;
	int maxMatchCount = 0, frmIdx1, frmIdx2;
	for (int i = 0; i < matches.size(); ++i){
		for (int j = 0; j < matches[i].size(); ++j){
			if (matches[i][j].size() < ParamParser::min_match_count) continue;
			double res_err = HUGE_VAL;
			double inlier_ratio_ = 0.0;
			RemoveOutliers(im[i], jm[j], inlier_ratio_, res_err, matches[i][j]);
			if (res_err < err && matches[i][j].size() >= ParamParser::min_match_count){
				maxMatchCount = matches[i][j].size();
				err = res_err;
				frmIdx1 = i;
				frmIdx2 = j;
			}
		}
	}
	if (maxMatchCount < ParamParser::min_match_count){
		std::cerr << "No Enough Sift Feature Matches! File " << __FILE__ << ", Line " << __LINE__ << std::endl;
		exit(-1);
	}
	std::cout << maxMatchCount << " " << err << " " << frmIdx1 << " " << frmIdx2 << std::endl;
	std::cout << "sift match: " << matches[frmIdx1][frmIdx2].size() << std::endl;
	for (int k = 0; k < matches[frmIdx1][frmIdx2].size(); ++k){
		const std::pair<Eigen::Vector2i, Eigen::Vector2i> &pr = matches[frmIdx1][frmIdx2][k];
		Eigen::Vector3d p1 = im[frmIdx1].GetPoint(pr.first[0], pr.first[1]);
		Eigen::Vector3d p2 = jm[frmIdx2].GetPoint(pr.second[0], pr.second[1]);
		matchPoints.push_back(std::make_pair(p1, p2));
	}
#if 1
	cv::RNG rng;
	CreateDir("./Match/");
	std::vector<cv::Mat> img1(im.size());
	std::vector<cv::Mat> img2(jm.size());
	for (int i = 0; i < img1.size(); ++i){
		char imgpath[128];
		sprintf_s(imgpath, "%s%05d.jpg", ParamParser::imgdirs[0].c_str(), i);
		img1[i] = cv::imread(imgpath);
	}
	for (int i = 0; i < img2.size(); ++i){
		char imgpath[128];
		sprintf_s(imgpath, "%s%05d.jpg", ParamParser::imgdirs[1].c_str(), i);
		img2[i] = cv::imread(imgpath);
	}
	for (int i = 0; i < matches.size(); ++i){
		for (int j = 0; j < matches[i].size(); ++j){
			cv::Mat img(img1[i].rows, img1[i].cols, img1[i].type());
			cv::vconcat(img1[i], img2[j], img);
			for (int k = 0; k < (int)matches[i][j].size(); ++k){
				uchar r = rng.uniform(0.0, 1.0) * 255;
				uchar g = rng.uniform(0.0, 1.0) * 255;
				uchar b = rng.uniform(0.0, 1.0) * 255;
				Eigen::Vector2i xy1 = matches[i][j][k].first;
				Eigen::Vector2i xy2 = matches[i][j][k].second;
				cv::circle(img, cv::Point2f(xy1[0], xy1[1]), 1, cv::Scalar(255, 255, 0));
				cv::circle(img, cv::Point2f(xy2[0], img1[i].rows + xy2[1]), 1, cv::Scalar(255, 255, 0));
				cv::line(img, cv::Point2f(xy1[0], xy1[1]), cv::Point2f(xy2[0], img1[i].rows + xy2[1]), cv::Scalar(b, g, r), 2);
			}
			sprintf_s(fn, "./Match/match%d_%d.jpg", i, j);
			cv::imshow("match", img);
			cv::imwrite(fn, img);
			cv::waitKey(1);
		}
	}
#endif
#pragma endregion

	SRTSolver solver(ParamParser::iter_num);
	solver.SetInput(matchPoints, cameras1[frmIdx1], cameras2[frmIdx2]);
	solver.EstimateTransform(scale, R, t);
}

void Processor::CalcSimilarityTransformationSeq(
	const std::vector<std::vector<Camera>> &cameras,
	std::vector<double> &scales,
	std::vector<Eigen::Matrix3d> &Rs,
	std::vector<Eigen::Vector3d> &ts,
	std::vector<std::pair<int, int>> &selectFrames){

	char fn[128];
	std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> matchPoints;

#pragma region //Generate Views
	//std::vector<std::vector<Image3D>> models(ParamParser::imgdirs.size());
	models.resize(ParamParser::imgdirs.size());
	for (int k = 0; k < models.size(); ++k){
		models[k].resize(cameras[k].size(), Image3D());
		CreateDir(ParamParser::imgdirs[k] + "Views/");
		for (int i = 0; i < models[k].size(); ++i){
			char imgpath[128], rawpath[128];
			sprintf_s(imgpath, "%s%05d.jpg", ParamParser::imgdirs[k].c_str(), i);
			sprintf_s(rawpath, "%sDATA/CHECK/_depth%d.raw", ParamParser::imgdirs[k].c_str(), i);
			models[k][i].LoadModel(imgpath, rawpath, cameras[k][i]);
		}
	}
#pragma endregion

	std::vector<std::vector<cv::Mat>> imgs(models.size());
	for (int k = 0; k < imgs.size(); ++k){
		imgs[k].resize(models[k].size());
		for (int i = 0; i < imgs[k].size(); ++i){
			char imgpath[128];
			sprintf_s(imgpath, "%s%05d.jpg", ParamParser::imgdirs[k].c_str(), i);
			imgs[k][i] = cv::imread(imgpath);
		}
	}

#pragma region //Feature Detecting
	std::vector<std::vector<std::vector<SiftGPU::SiftKeypoint>>> keys(models.size());
	std::vector<std::vector<std::vector<float>>> descs(models.size());
	for (int k = 0; k < models.size(); ++k){
		CreateDir(ParamParser::imgdirs[k] + "Views/Sift/");
		CreateDir(ParamParser::imgdirs[k] + "Views/Sift1/");
		std::vector<std::string> proj_view_path;
		for (int i = 0; i < models[k].size(); ++i){
			for (int j = 0; j < ParamParser::view_count; ++j){
				sprintf_s(fn, "%sViews/proj%d_%d.jpg", ParamParser::imgdirs[k].c_str(), i, j);
				proj_view_path.push_back(fn);
			}
		}
		FeatureProc::DetectFeature(proj_view_path, keys[k], descs[k]);

		/*-------------------------Remove Background Points-------------------------*/
#if 1
		//remove background points
		for (int i = 0; i < keys[k].size(); ++i){ //sequence
			const int curfrmIdx = i / ParamParser::view_count;
			const int view = i % ParamParser::view_count;
			const Image3D &curIm = models[k][curfrmIdx];
			const int w = curIm.GetWidth();
			const int h = curIm.GetHeight();
			int newSize = 0;
			for (int j = 0; j < keys[k][i].size(); ++j){ //frame
				const int idx = curIm.GetTexIndex(view, keys[k][i][j].x, keys[k][i][j].y);
				if (idx == -1 || !curIm.IsValid(idx % w, idx / w) || (ParamParser::isSegment && !curIm.InMask(idx % w, idx / w))) continue;
				bool removed = false;
				const Eigen::Vector3d p3d = curIm.GetPoint(idx % w, idx / w);
				for (int frmIdx = 0; frmIdx < models[k].size(); ++frmIdx){
					if (frmIdx != curfrmIdx){
						const Image3D &im = models[k][frmIdx];
						int u, v;
						im.GetCamera().GetImgCoordFromWorld(p3d, u, v);
						if (!CheckRange(u, v, w, h)){
							removed = true;
							break;
						}
					}
				}
				if (!removed){
					keys[k][i][newSize] = keys[k][i][j];
					for (int j0 = 0; j0 < 128; ++j0){
						descs[k][i][newSize * 128 + j0] = descs[k][i][j * 128 + j0];
					}
					++newSize;
				}
			}
			keys[k][i].resize(newSize);
			descs[k][i].resize(newSize * 128);
		}
#endif
		/*-------------------------Remove Background Points-------------------------*/

		for (int i = 0; i < keys[k].size(); ++i){
			const std::string imgpath = proj_view_path[i];
			cv::Mat img1 = cv::imread(imgpath);
			for (const auto & kp : keys[k][i]){
				cv::circle(img1, cv::Point2f(kp.x, kp.y), 1, cv::Scalar(0, 255, 0), -1);
			}
			int pos = imgpath.find_last_of('/');
			std::string siftpath = imgpath.substr(0, pos + 1) + "Sift1/" + imgpath.substr(pos + 1, imgpath.size() - pos - 1);
			cv::imshow("Sift1", img1);
			cv::imwrite(siftpath, img1);
			cv::waitKey(1);
		}
	}
#pragma endregion

#pragma region //Feature Matching
	CreateDir("./Match/");
	//CreateDir("./Match1/");

	matchPoints.resize(models.size() - 1);
	scales.resize(models.size() - 1);
	Rs.resize(models.size() - 1);
	ts.resize(models.size() - 1);

	//std::vector<std::pair<int, int>> selectFrames;
	for (int k = 0; k < models.size() - 1; ++k){
		std::cout << "|-------------------- Seq " << k << " ----> " << k + 1 << "--------------------|" << std::endl;

		/*-----------------------------Feature Matching-----------------------------*/
		std::vector<std::vector<std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>>>> matches;
		FeatureProc::MatchFeature(keys[k], keys[k + 1], descs[k], descs[k + 1], matches);
		/*-----------------------------Feature Matching-----------------------------*/

		const int w = models[k][0].GetWidth();
		const int h = models[k][0].GetHeight();
		const std::vector<Image3D> &model1 = models[k];
		const std::vector<Image3D> &model2 = models[k + 1];

		//RemoveDupPoints(model1, model2, matches);

#if 1
		std::vector<std::vector<int>> size1(model1.size(), std::vector<int>(model2.size()));
		std::vector<std::vector<int>> size2(model1.size(), std::vector<int>(model2.size()));
		std::vector<std::vector<int>> size3(model1.size(), std::vector<int>(model2.size()));

		/*-------------------------Remove Duplicate Points--------------------------*/
		std::vector<std::vector<std::set<std::pair<Vec2i, Vec2i>>>> uniqueMatches(model1.size(),
			std::vector<std::set<std::pair<Vec2i, Vec2i>>>(model2.size()));
		for (int i = 0; i < matches.size(); ++i){
			int frmNo1 = i / ParamParser::view_count;
			for (int j = 0; j < matches[i].size(); ++j){
				int frmNo2 = j / ParamParser::view_count;
				for (int k = 0; k < matches[i][j].size(); ++k){
					std::pair<Eigen::Vector2i, Eigen::Vector2i> &pr = matches[i][j][k];
					if (CheckRange(pr.first[0], pr.first[1], w, h) && CheckRange(pr.second[0], pr.second[1], w, h)){
						int idx1 = model1[frmNo1].GetTexIndex(i % ParamParser::view_count, pr.first[0], pr.first[1]);
						int idx2 = model2[frmNo2].GetTexIndex(j % ParamParser::view_count, pr.second[0], pr.second[1]);
						if (idx1 != -1 && idx2 != -1 && model1[frmNo1].IsValid(pr.first[0], pr.first[1]) && model2[frmNo2].IsValid(pr.second[0], pr.second[1])){
							uniqueMatches[frmNo1][frmNo2].insert(std::make_pair(Vec2i(idx1 % w, idx1 / w), Vec2i(idx2 % w, idx2 / w)));
						}
					}
				}
			}
		}
		std::vector<std::vector<std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>>>>().swap(matches);
		matches.resize(model1.size(), std::vector<std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>>>(model2.size()));

		for (int i = 0; i < uniqueMatches.size(); ++i){
			for (int j = 0; j < uniqueMatches[i].size(); ++j){
				std::set<std::pair<Vec2i, Vec2i>>::iterator it = uniqueMatches[i][j].begin();
				for (; it != uniqueMatches[i][j].end(); ++it){
					matches[i][j].push_back(std::make_pair(Eigen::Vector2i(it->first[0], it->first[1]), Eigen::Vector2i(it->second[0], it->second[1])));
				}
				size1[i][j] = matches[i][j].size();
			}
		}
		/*-------------------------Remove Duplicate Points--------------------------*/
#endif
		/*----------------------------------SSD Error-------------------------------*/
		int &ssd_win = ParamParser::ssd_win;
		double &ssd_err = ParamParser::ssd_err;
		for (int i = 0; i < matches.size(); ++i){
			for (int j = 0; j < matches[i].size(); ++j){
				int newSize = 0;
				for (int idx = 0; idx < matches[i][j].size(); ++idx){
					const std::pair<Eigen::Vector2i, Eigen::Vector2i> &pr = matches[i][j][idx];
					int u1 = pr.first[0], v1 = pr.first[1];
					int u2 = pr.second[0], v2 = pr.second[1];
					if (u1 >= ssd_win && v1 >= ssd_win && u2 >= ssd_win && v2 >= ssd_win &&
						u1 < w - ssd_win && v1 < h - ssd_win && u2 < w - ssd_win && v2 < h - ssd_win){
						double err = FeatureProc::SSD(imgs[k][i], u1, v1, imgs[k + 1][j], u2, v2, ssd_win);
						if (err <= ssd_err){
							matches[i][j][newSize++] = matches[i][j][idx];
						}
					}
				}
				matches[i][j].resize(newSize);
				size2[i][j] = newSize;
			}
		}
		/*----------------------------------SSD Error-------------------------------*/
#if 1
		/*----------------------------------Gap Error-------------------------------*/
		const double pixelGap = ParamParser::sample_interval * ParamParser::sample_interval;
		for (int i = 0; i < matches.size(); ++i){
			for (int j = 0; j < matches[i].size(); ++j){
				int k0 = 0, newSize = 0;
				for (int k = 0; k < matches[i][j].size(); ++k){
					bool flag = false;
					for (int k0 = 0; k0 < newSize; ++k0){
						std::pair<Eigen::Vector2i, Eigen::Vector2i> &pr1 = matches[i][j][k0];
						std::pair<Eigen::Vector2i, Eigen::Vector2i> &pr2 = matches[i][j][k];
						Eigen::Vector2i x = pr1.first - pr2.first;
						Eigen::Vector2i y = pr1.second - pr2.second;
						if ((x[0] * x[0] + x[1] * x[1]) <= pixelGap || (y[0] * y[0] + y[1] * y[1]) <= pixelGap){
							flag = true;
							break;
						}
					}
					if (!flag) matches[i][j][newSize++] = matches[i][j][k];
				}
				matches[i][j].resize(newSize);
				size3[i][j] = newSize;
			}
		}
		/*----------------------------------Gap Error-------------------------------*/

#ifdef PRINT_INFO
		for (int i = 0; i < model1.size(); ++i){
			for (int j = 0; j < model2.size(); ++j){
				std::cout << "Frame# " << i << ", " << j << "; " << size3[i][j] << " | " << size2[i][j] << " | " << size1[i][j] << " sift matches" << std::endl;
			}
		}
#endif
#endif

		/*-------------------------------Remove Outliers----------------------------*/
		double err = HUGE_VAL;
		double inlier_ratio = 0.0;
		std::cout << "select keyframe..." << std::endl;
		int maxMatchCount = 0, frmIdx1, frmIdx2;
		for (int i = 0; i < matches.size(); ++i){
			for (int j = 0; j < matches[i].size(); ++j){
				if (matches[i][j].size() < ParamParser::min_match_count) continue;
				double inlier_ratio_ = 0.0;
				double res_err = HUGE_VAL;
				RemoveOutliers(model1[i], model2[j], inlier_ratio_, res_err, matches[i][j]);
				if (res_err < err && matches[i][j].size() >= ParamParser::min_match_count){
					maxMatchCount = matches[i][j].size();
					err = res_err;
					inlier_ratio = inlier_ratio_;
					frmIdx1 = i;
					frmIdx2 = j;
				}
			}
		}
		/*-------------------------------Remove Outliers----------------------------*/
#if 1
		cv::RNG rng;
		for (int i = 0; i < matches.size(); ++i){
			for (int j = 0; j < matches[i].size(); ++j){
				cv::Mat img(imgs[k][i].rows, imgs[k][i].cols, imgs[k][i].type());
				cv::Mat img1(imgs[k][i].rows, imgs[k][i].cols, imgs[k][i].type()); 
				imgs[k][i].copyTo(img1);
				cv::Mat img2(imgs[k + 1][j].rows, imgs[k + 1][j].cols, imgs[k + 1][j].type());
				imgs[k + 1][j].copyTo(img2);
				cv::vconcat(img1, img2, img);
				for (int idx = 0; idx < (int)matches[i][j].size(); ++idx){
					uchar r = rng.uniform(0.0, 1.0) * 255;
					uchar g = rng.uniform(0.0, 1.0) * 255;
					uchar b = rng.uniform(0.0, 1.0) * 255;
					Eigen::Vector2i xy1 = matches[i][j][idx].first;
					Eigen::Vector2i xy2 = matches[i][j][idx].second;
					cv::circle(img, cv::Point2f(xy1[0], xy1[1]), 1, cv::Scalar(255, 255, 0));
					cv::circle(img, cv::Point2f(xy2[0], imgs[k][i].rows + xy2[1]), 1, cv::Scalar(255, 255, 0));
					cv::line(img, cv::Point2f(xy1[0], xy1[1]), cv::Point2f(xy2[0], imgs[k][i].rows + xy2[1]), cv::Scalar(b, g, r), 2);
				}
				sprintf_s(fn, "./Match/match%d_%d_%d.jpg", k, i, j);
				cv::imshow("match", img);
				cv::imwrite(fn, img);
				cv::waitKey(1);
			}
		}
#endif
		if (maxMatchCount < ParamParser::min_match_count){
			std::cout << "sift match: " << maxMatchCount << std::endl;
			std::cout << "residual error: " << err << std::endl;
			std::cout << "inlier ratio: " << inlier_ratio << std::endl;
			std::cerr << "No Enough Sift Feature Matches! File " << __FILE__ << ", Line " << __LINE__ << std::endl;
			exit(-1);
		}
		selectFrames.push_back(std::make_pair(frmIdx1, frmIdx2));
		std::cout << "selected keyframe: " << frmIdx1 << " " << frmIdx2 << std::endl;
		std::cout << "sift match: " << matches[frmIdx1][frmIdx2].size() << std::endl;
		std::cout << "residual error: " << err << std::endl;
		std::cout << "inlier ratio: " << inlier_ratio << std::endl;
		for (int i = 0; i < matches[frmIdx1][frmIdx2].size(); ++i){
			const std::pair<Eigen::Vector2i, Eigen::Vector2i> &pr = matches[frmIdx1][frmIdx2][i];
			Eigen::Vector3d p1 = model1[frmIdx1].GetPoint(pr.first[0], pr.first[1]);
			Eigen::Vector3d p2 = model2[frmIdx2].GetPoint(pr.second[0], pr.second[1]);
			matchPoints[k].push_back(std::make_pair(p1, p2));
		}

		/*----------------------------------Solve SRT-------------------------------*/
		SRTSolver solver(ParamParser::iter_num);
		solver.SetInput(matchPoints[k], model1[frmIdx1].GetCamera(), model2[frmIdx2].GetCamera());
		solver.SetPrintFlag(false);
		solver.EstimateTransform(scales[k], Rs[k], ts[k]);
		std::cout << "pixel reproject error = " << solver.ResidualError(scales[k], Rs[k], ts[k]) << std::endl;
		for (int k0 = 0; k0 < k; ++k0){
			Rs[k0] = Rs[k] * Rs[k0];
			ts[k0] = scales[k] * Rs[k] * ts[k0] + ts[k];
			scales[k0] = scales[k] * scales[k0];
		}
		std::cout << "|------------------------------------------------------|" << std::endl;
		/*----------------------------------Solve SRT-------------------------------*/
	}
#pragma endregion
	std::cout << "|------------------------------------------|" << std::endl;
	for (int k = 0; k < selectFrames.size(); ++k){
		std::cout << "Seq " << k << " --> " << k + 1 << ": " << selectFrames[k].first << ", " << selectFrames[k].second << std::endl;
	}
	std::cout << "|------------------------------------------|" << std::endl;
}

void Processor::AlignmentSeq(){
	srand((unsigned int)time(NULL));

	if (cameras.size() == 0){
		LoadCameras();
	}

	CheckConsistency(cameras);

	std::vector<double> scales;
	std::vector<Eigen::Matrix3d> Rs;
	std::vector<Eigen::Vector3d> ts;

	std::vector<std::pair<int, int>> selectFramesPair;
	CalcSimilarityTransformationSeq(cameras, scales, Rs, ts, selectFramesPair);

	scales.push_back(1.0);
	Rs.push_back(Eigen::Matrix3d::Identity());
	ts.push_back(Eigen::Vector3d::Zero());

	std::ofstream ofs_srt;
	ofs_srt.open("./Result/SRT.txt", std::ofstream::out);
	int size = scales.size();
	for (int k = 0; k < size; ++k){
		std::cout << "|--------------------" << k << " ---> " << size - 1 << "--------------------|" << std::endl;
		std::cout << "Scale: " << scales[k] << std::endl;
		std::cout << "Rotation Matrix: " << std::endl;
		std::cout << Rs[k] << std::endl;
		std::cout << "Det(R): " << Rs[k].determinant() << std::endl;
		std::cout << "Translation: " << std::endl;
		std::cout << ts[k].transpose() << std::endl;
		
		ofs_srt << scales[k] << std::endl;
		ofs_srt << Rs[k] << std::endl;
		ofs_srt << ts[k].transpose() << std::endl;
	}
	ofs_srt.close();

#if 1
	for (int k = 0; k < selectFramesPair.size(); ++k){
		int w = cameras[k][0].W();
		int h = cameras[k][0].H();
		char fn1[128], fn2[128];
		sprintf_s(fn1, "%sDATA/_depth%d.raw", ParamParser::imgdirs[k].c_str(), selectFramesPair[k].first);
		sprintf_s(fn2, "%sDATA/_depth%d.raw", ParamParser::imgdirs[k + 1].c_str(), selectFramesPair[k].second);

		std::cout << fn1 << std::endl << fn2 << std::endl;

		std::vector<double> dsp1, dsp2;
		LoadDepth(fn1, dsp1, w, h);
		LoadDepth(fn2, dsp2, w, h);

		Depth2Model d2m1(ParamParser::m_fMinDsp, ParamParser::m_fMaxDsp, ParamParser::smooth_thres);
		d2m1.SaveModel(dsp1, cameras[k][selectFramesPair[k].first]);
		Depth2Model d2m2(ParamParser::m_fMinDsp, ParamParser::m_fMaxDsp, ParamParser::smooth_thres);
		d2m2.SaveModel(dsp2, cameras[k + 1][selectFramesPair[k].second]);

		std::vector<Eigen::Vector3d> &points1 = d2m1.point3d;
		std::vector<int> &facets1 = d2m1.facets;

		for (int i = 0; i < points1.size(); ++i){
			double scale = 1.0 / scales[k + 1] * scales[k];
			Eigen::Matrix3d R = Rs[k + 1].transpose() * Rs[k];
			Eigen::Vector3d t = 1.0 / scales[k + 1] * Rs[k + 1].transpose() * (ts[k] - ts[k + 1]);
			points1[i] = scale * R * points1[i] + t;
		}

		std::vector<Eigen::Vector3d> &points2 = d2m2.point3d;
		std::vector<int> &facets2 = d2m2.facets;

		Alignment align;
		align.RetainConnectRegion(points1, std::vector<Eigen::Vector3d>(), facets1);
		align.RetainConnectRegion(points2, std::vector<Eigen::Vector3d>(), facets2);

		sprintf_s(fn1, "./Result/pair%d_%d.obj", k, selectFramesPair[k].first);
		sprintf_s(fn2, "./Result/pair%d_%d.obj", k, selectFramesPair[k].second);
		WriteObj(fn1, points1, std::vector<Eigen::Vector3d>(), facets1);
		WriteObj(fn2, points2, std::vector<Eigen::Vector3d>(), facets2);
	}
#endif

	char fn[128];
	std::vector<std::vector<Eigen::Vector3d>> points(ParamParser::imgdirs.size());
	std::vector<std::vector<Eigen::Vector3d>> normals(ParamParser::imgdirs.size());
	for (int k = 0; k < ParamParser::imgdirs.size(); ++k){
		CreateDir(ParamParser::imgdirs[k] + "DATA/TMP/");
		char fn1[128], fn2[128];
		for (int i = 0; i < cameras[k].size(); ++i){
			sprintf_s(fn1, "%sDATA/_depth%d.raw", ParamParser::imgdirs[k].c_str(), i);
			sprintf_s(fn2, "%sDATA/TMP/_depth%d.raw", ParamParser::imgdirs[k].c_str(), i);
			MoveFileEx(fn1, fn2, MOVEFILE_REPLACE_EXISTING);
		}
		for (int i = 0; i < cameras[k].size(); ++i){
			sprintf_s(fn1, "%sDATA/CHECK/_depth%d.raw", ParamParser::imgdirs[k].c_str(), i);
			sprintf_s(fn2, "%sDATA/_depth%d.raw", ParamParser::imgdirs[k].c_str(), i);
			MoveFileEx(fn1, fn2, MOVEFILE_REPLACE_EXISTING);
		}
		std::vector<std::string> actsfiles = ScanNSortDirectory(ParamParser::imgdirs[k].c_str(), "act");
		GeometryRec gr;
		gr.Init(
			actsfiles[0],
			ParamParser::m_fMaxDsp,
			ParamParser::m_fMinDsp,
			ParamParser::sample_radius,
			ParamParser::dsp_err,
			ParamParser::conf_min,
			__min(ParamParser::nbr_frm_num, cameras[k].size()),
			ParamParser::nbr_frm_step,
			0,
			cameras[k].size() - 1,
			ParamParser::edge_sz_thres,
			ParamParser::psn_dpt_max,
			ParamParser::psn_dpt_min
			);
		gr.RunPointSample();

		std::string psrdir = ParamParser::imgdirs[k] + "Rec/";
		std::vector<std::string> psrfiles = ScanNSortDirectory(psrdir.c_str(), "npts");
		std::ifstream ifs(psrfiles[0], std::ifstream::in);
		if (!ifs.is_open()){
			std::cerr << "ERROR: Open File Failed, File " << __FILE__ << ", Line " << __LINE__ << std::endl;
			exit(-1);
		}
		float x, y, z, nx, ny, nz;
		while (ifs.peek() != EOF){
			ifs >> x >> y >> z >> nx >> ny >> nz;
			points[k].push_back(Eigen::Vector3d(x, y, z));
			normals[k].push_back(Eigen::Vector3d(nx, ny, nz));
		}
		ifs.close();

		int newSize = 0;
		for (int pIdx = 0; pIdx < points[k].size(); ++pIdx){
			const Eigen::Vector3d &p3d = points[k][pIdx];
			bool removed = false;
			int inMaskCount = 0, tot = 0;
			for (int k0 = 0; k0 < ParamParser::imgdirs.size(); ++k0){
				Eigen::Vector3d p3d_ = p3d;
				if (k0 != k){
					double scale = 1.0 / scales[k0] * scales[k];
					Eigen::Matrix3d R = Rs[k0].transpose() * Rs[k];
					Eigen::Vector3d t = 1.0 / scales[k0] * Rs[k0].transpose() * (ts[k] - ts[k0]);
					p3d_ = scale * R * p3d + t;
				}
				for (int cIdx = 0; cIdx < cameras[k0].size(); ++cIdx){
					int u, v;
					cameras[k0][cIdx].GetImgCoordFromWorld(p3d_, u, v);
					if (!CheckRange(u, v, cameras[k0][cIdx].W(), cameras[k0][cIdx].H())){
						removed = true;
						break;
					}
					if (ParamParser::isSegment && models[k0][cIdx].InMask(u, v)){
						inMaskCount++;
					}
					tot++;
				}
				if (removed) break;
			}
			//if (removed || (ParamParser::isSegment && (float)inMaskCount / (float)tot <= 0.1f)) continue;
			if (!removed){
				points[k][newSize] = points[k][pIdx];
				normals[k][newSize] = normals[k][pIdx];
				newSize++;
			}
		}
		for (int i = 0; i < cameras[k].size(); ++i){
			sprintf_s(fn1, "%sDATA/_depth%d.raw", ParamParser::imgdirs[k].c_str(), i);
			sprintf_s(fn2, "%sDATA/CHECK/_depth%d.raw", ParamParser::imgdirs[k].c_str(), i);
			MoveFileEx(fn1, fn2, MOVEFILE_REPLACE_EXISTING);
		}
		for (int i = 0; i < cameras[k].size(); ++i){
			sprintf_s(fn1, "%sDATA/TMP/_depth%d.raw", ParamParser::imgdirs[k].c_str(), i);
			sprintf_s(fn2, "%sDATA/_depth%d.raw", ParamParser::imgdirs[k].c_str(), i);
			MoveFileEx(fn1, fn2, MOVEFILE_REPLACE_EXISTING);
		}
	}

	CreateDir("./Result/");
	std::vector<std::vector<Eigen::Vector3d>> vpts(ParamParser::imgdirs.size());
	std::vector<std::vector<Eigen::Vector3d>> vnorm(ParamParser::imgdirs.size());
	for (int k = 0; k < ParamParser::imgdirs.size(); ++k){
		vpts[k].resize(points[k].size());
		vnorm[k].resize(normals[k].size());
		for (int i = 0; i < points[k].size(); ++i){
			vpts[k][i] = scales[k] * Rs[k] * points[k][i] + ts[k];
			vnorm[k][i] = Rs[k] * normals[k][i];
		}

		sprintf_s(fn, "./Result/PSR%d.obj", k);
		WriteObj(fn, vpts[k], vnorm[k]);
	}

	std::ofstream ofs_psr("./Result/PSR.npts", std::ofstream::out);
	for (int k = 0; k < vpts.size(); ++k){
		for (int i = 0; i < vpts[k].size(); ++i){
			ofs_psr << vpts[k][i][0] << " " << vpts[k][i][1] << " " << vpts[k][i][2] << " "
					<< vnorm[k][i][0] << " " << vnorm[k][i][1] << " " << vnorm[k][i][2] << std::endl;
		}
	}
	ofs_psr.close();

	GeometryRec gr;
	gr.Init(
		"",
		ParamParser::m_fMaxDsp,
		ParamParser::m_fMinDsp,
		ParamParser::sample_radius,
		ParamParser::dsp_err,
		ParamParser::conf_min,
		ParamParser::nbr_frm_num,
		ParamParser::nbr_frm_step,
		0,
		cameras[0].size() - 1,
		ParamParser::edge_sz_thres,
		ParamParser::psn_dpt_max,
		ParamParser::psn_dpt_min
		);
	gr.RunPoisson("./Result/");

	std::vector<Eigen::Vector3d> point3d, normal3d;
	std::vector<int> facets;
	ReadObj("./Result/Model.obj", point3d, normal3d, facets);

	int newSize = 0;
	std::unordered_map<int, int> mp;
	for (int pIdx = 0; pIdx < point3d.size(); ++pIdx){
		const Eigen::Vector3d &p3d = point3d[pIdx];
		bool removed = false;
		for (int k0 = 0; k0 < ParamParser::imgdirs.size(); ++k0){
			Eigen::Vector3d p3d_ = 1.0 / scales[k0] * Rs[k0].transpose() * (p3d - ts[k0]);
			for (int cIdx = 0; cIdx < cameras[k0].size(); ++cIdx){
				int u, v;
				cameras[k0][cIdx].GetImgCoordFromWorld(p3d_, u, v);
				if (!CheckRange(u, v, cameras[k0][cIdx].W(), cameras[k0][cIdx].H())){
					removed = true;
					break;
				}
			}
			if (removed) break;
		}
		if (!removed){
			point3d[newSize] = point3d[pIdx];
			normal3d[newSize] = normal3d[pIdx];
			mp[pIdx] = newSize;
			newSize++;
		}
	}
	std::cout << newSize << " | " << point3d.size() << std::endl;
	point3d.resize(newSize);
	normal3d.resize(newSize);

	std::vector<int> facets_;
	for (int fIdx = 0; fIdx < facets.size(); fIdx += 3){
		if (mp.count(facets[fIdx]) && mp.count(facets[fIdx + 1]) && mp.count(facets[fIdx + 2])){
			facets_.push_back(mp[facets[fIdx]]);
			facets_.push_back(mp[facets[fIdx + 1]]);
			facets_.push_back(mp[facets[fIdx + 2]]);
		}
	}
	std::swap(facets, facets_);

	Alignment align;
	align.RetainConnectRegion(point3d, normal3d, facets);
	WriteObj("./Result/Model.obj", point3d, normal3d, facets);
}

void Processor::Deform(){
	/*std::vector<std::vector<Camera>> cameras(ParamParser::imgdirs.size());
	for (int k = 0; k < cameras.size(); ++k){
		std::vector<std::string> actsfiles = ScanNSortDirectory(ParamParser::imgdirs[k].c_str(), "act");
		cameras[k] = LoadCalibrationFromActs(actsfiles[0]);
	}*/

	if (cameras.size() == 0){
		LoadCameras();
	}

	std::vector<Eigen::Vector3d> tgt, tgt_normals;
	std::vector<int> tgt_facets;
	ReadObj("./Result/Model.obj", tgt, tgt_normals, tgt_facets);

	std::vector<Eigen::Vector3d> src, src_normals;
	std::vector<int> src_facets;
	ReadObj("./Template/meanbody.obj", src, src_normals, src_facets);

	Eigen::Matrix3d R;
	Eigen::Vector3d t;
	cameras[0][0].GetRT(R, t);
	Alignment align;
	align.Align(src, src_normals, src_facets, tgt, tgt_normals, tgt_facets, R.transpose().col(2));
	//align.RemoveGround(point3d, normal3d, facets);

	std::cout << "Deformation" << std::endl;
	Deformation deform(src, src_normals, src_facets);
	deform.Deform(tgt, tgt_normals, 100.0, 100.0);
	exportOBJ("./Result/deform.obj", &deform.Polyhedron());
}

void Processor::Render(int argc, char *argv[]){

	std::vector<double> scales(ParamParser::imgdirs.size());
	std::vector<Eigen::Matrix3d> Rs(ParamParser::imgdirs.size());
	std::vector<Eigen::Vector3d> ts(ParamParser::imgdirs.size());
	std::ifstream ifs("./Result/SRT.txt", std::ifstream::in);
	if (!ifs.is_open()){
		std::cerr << "ERROR: Open File Failed, File " << __FILE__ << ", Line " << __LINE__ << std::endl;
		exit(-1);
	}
	float x, y, z, s;
	for (int k = 0; k < ParamParser::imgdirs.size(); ++k){
		ifs >> s;
		scales[k] = s;
		for (int i = 0; i < 3; ++i){
			ifs >> x >> y >> z;
			Rs[k](i, 0) = x; Rs[k](i, 1) = y; Rs[k](i, 2) = z;
		}
		ifs >> x >> y >> z;
		ts[k] = Eigen::Vector3d(x, y, z);
		
		std::cout << scales[k] << std::endl;
		std::cout << Rs[k] << std::endl;
		std::cout << ts[k] << std::endl;
	}
	ifs.close();

	if (cameras.size() == 0){
		LoadCameras();
	}

	std::vector<Eigen::Vector3d> points, normals;
	std::vector<int> facets;
	ReadObj("./Result/deform.obj", points, normals, facets);
	for (int k = 0; k < ParamParser::imgdirs.size(); ++k){
		std::vector<Eigen::Vector3d> points_(points.size());
		std::vector<Eigen::Vector3d> normals_(normals.size());
		std::vector<int> facets_ = facets;

		for (int i = 0; i < points.size(); ++i){
			points_[i] = 1.0 / scales[k] * Rs[k].transpose() * (points[i] - ts[k]);
			normals_[i] = Rs[k].transpose() * normals[i];
		}
		
		WriteObj("./render.obj", points_, normals_, facets_);

		Model2Depth::SetInput(points_, normals_, facets_, cameras[k], ParamParser::imgdirs[k]);
		Model2Depth::Run(argc, argv);
	}
}

void Processor::RenderDepthMap(){
	LoadCameras();
	char fn[128];
	for (int i = 0; i < cameras[0].size(); ++i){
		std::vector<double> depth;
		sprintf_s(fn, "%sDATA/_depth%d.raw", ParamParser::imgdirs[0].c_str(), i);
		LoadDepth(fn, depth, cameras[0][i].W(), cameras[0][i].H());
		sprintf_s(fn, "%s_depth%d.jpg", ParamParser::imgdirs[0].c_str(), i);
		::RenderDepthMap(fn, depth, cameras[0][i].W(), cameras[0][i].H());
	}

}