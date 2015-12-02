#include "Utils.h"
#include "Camera.h"
#include "FeatureProc.h"
#include "SRTSolver.h"
#include "PlyObj.h"
#include "Processor.h"
#include "Vector.h"
#include "Depth2Model.h"
#include "Alignment.h"
#include <algorithm>
#include <set>
#include <unordered_map>

#define PRINT_INFO

void Processor::SetParamFromFile(const std::string filename){
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
		if ("ViewCount"			 == tip){ ifs >> view_count;		}
		else if ("MinMatchCount" == tip){ ifs >> min_match_count;	}
		else if ("IterNum"		 == tip){ ifs >> iter_num;			}
		else if ("SampleIterval" == tip){ ifs >> sample_interval;	}
		else if ("SSDWin"		 == tip){ ifs >> ssd_win;			}
		else if ("Axis"			 == tip){ ifs >> axis;				}
		else if ("RotAngle"		 == tip){ ifs >> rot_angle;			}
		else if ("PixelError"	 == tip){ ifs >> pixel_err;			}
		else if ("SSDError"		 == tip){ ifs >> ssd_err;			}
		else if ("DistMax"		 == tip){ ifs >> distmax;			}
		else if ("RatioMax"		 == tip){ ifs >> ratiomax;			}
		else if ("HMarginRatio"  == tip){ ifs >> h_margin_ratio;	}
		else if ("VMarginRatio"  == tip){ ifs >> v_margin_ratio;	}
		else if ("MinDsp"		 == tip){ ifs >> m_fMinDsp;			}
		else if ("MaxDsp"		 == tip){ ifs >> m_fMaxDsp;			}
		else if ("ImgPathList"	 == tip){ ifs >> imgPathListFile;	}
	}
	ifs.close();

	ifs.open(imgPathListFile.c_str(), std::ifstream::in);
	if (!ifs.is_open()){
		std::cerr << "ERROR: Open File " << imgPathListFile << " Failed in File " << __FILE__ << ", Line " << __LINE__ << std::endl;
		exit(-1);
	}
	std::vector<std::string>().swap(imgdirs);
	std::string imgdir;
	while (ifs.peek() != EOF){
		ifs >> imgdir;
		if (imgdir.size() == 0) continue;
		else if (imgdir[0] == '#') continue;
		imgdirs.push_back(imgdir);
	}
	ifs.close();
}

void Processor::RemoveDupPoints(
	const std::vector<Image3D> &im,
	const std::vector<Image3D> &jm,
	std::vector<std::vector<std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>>>> &matches){
//#ifdef PRINT_INFO
//	std::cout << "Remove duplicated points..." << std::endl;
//#endif
	
	std::vector<std::vector<std::set<std::pair<Vec2i, Vec2i>>>> uniqueMatches(im.size(),
		std::vector<std::set<std::pair<Vec2i, Vec2i>>>(jm.size()));
	int w = im[0].GetWidth();
	int h = im[0].GetHeight();
	for (int i = 0; i < matches.size(); ++i){
		int frmNo1 = i / view_count;
		for (int j = 0; j < matches[i].size(); ++j){
			int frmNo2 = j / view_count;
			for (int k = 0; k < matches[i][j].size(); ++k){
				std::pair<Eigen::Vector2i, Eigen::Vector2i> &pr = matches[i][j][k];
				int idx1 = im[frmNo1].GetTexIndex(i % view_count, pr.first[0], pr.first[1]);
				int idx2 = jm[frmNo2].GetTexIndex(j % view_count, pr.second[0], pr.second[1]);
				if (idx1 != -1 && idx2 != -1 && im[frmNo1].IsValid(pr.first[0], pr.first[1]) && jm[frmNo2].IsValid(pr.second[0], pr.second[1])){
					uniqueMatches[frmNo1][frmNo2].insert(std::make_pair(Vec2i(idx1 % w, idx1 / w), Vec2i(idx2 % w, idx2 / w)));
				}
			}
		}
	}
	std::vector<std::vector<std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>>>>().swap(matches);
	matches.resize(im.size(), std::vector<std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>>>(jm.size()));

	double pixelErr = sample_interval * sample_interval;
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
#ifdef PRINT_INFO
			std::cout << "Frame# " << i << ", " << j << " | " << matches[i][j].size() << " sift matches found" << std::endl;
#endif
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

#ifdef PRINT_INFO
	std::cout << "+--------------------------------------+" << std::endl;
#endif
	for (int k = 0; k < 2; ++k){
		double scale_;
		Eigen::Matrix3d R_;
		Eigen::Vector3d t_;
		SRTSolver solver(80);
		solver.SetPrintFlag(false);
		solver.SetInput(matchPoints, im.GetCamera(), jm.GetCamera());
		solver.EstimateTransform(scale_, R_, t_);

		std::vector<double> dists(size);
		double err_inlier_ = 0.0, err_all_ = 0.0;
		int newSize = 0;
#if 0
		double maxDist = 1 - HUGE_VAL;//std::numeric_limits<double>::min();
		for (int i = 0; i < size; ++i){
			const Vec3d &p1 = matchPoints[i].first;
			const Vec3d &p2 = matchPoints[i].second;
			dists[i] = (scale_ * R_ * p1 + t_ - p2).norm();
			maxDist = __max(dists[i], maxDist);
		}
		for (int i = 0; i < size; ++i){
			if (dists[i] <= threshold * maxDist){
				matchPoints[newSize] = matchPoints[i];
				matches[newSize++] = matches[i];
			}
		}
		matchPoints.resize(newSize);
		matches.resize(newSize);

		err_ = solver.ResidualError(scale_, R_, t_);
		if (err > err_){
			err = err_;
			scale = scale_;
			R = R_;
			t = t_;
			//std::cout << "Residual Error£º " << err_ << std::endl;
			std::cout << "pixel reproject error = " << err_ << std::endl;
			std::cout << "Removed outliers£º " << size - newSize << " | " << size << std::endl;
		}
#else
		for (int i = 0; i < size; ++i){
			const Eigen::Vector3d &p1 = matchPoints[i].first;
			const Eigen::Vector3d &p2 = matchPoints[i].second;
			Eigen::Vector3d tp = scale_ * R_ * p1 + t_;

			int u1, v1, u2, v2;
			jm.GetCamera().GetImgCoordFromWorld(tp, u1, v1);
			jm.GetCamera().GetImgCoordFromWorld(p2, u2, v2);

			double pixel_err_ = sqrt((u1 - u2) * (u1 - u2) + (v1 - v2) * (v1 - v2));
			err_all_ += pixel_err_;
			if (pixel_err_ <= pixel_err){
				matchPoints[newSize] = matchPoints[i];
				matches[newSize++] = matches[i];
				err_inlier_ += pixel_err_;
			}
		}
		matchPoints.resize(newSize);
		matches.resize(newSize);

		err_all_ /= size;
		if (err > err_all_) err = err_all_;
		if (newSize > 0){
			err_inlier_ /= newSize;
			if (inlier_ratio < (err_inlier_ / err_all_)){
				inlier_ratio = err_inlier_ / err_all_;
				scale = scale_;
				R = R_;
				t = t_;
			}
		}
		std::cout << "pixel reproject error = " << err_inlier_ << " / " << err << std::endl;
		std::cout << "inlier ratio: " << inlier_ratio << std::endl;
		std::cout << "Removed outliers£º " << size - newSize << " | " << size << std::endl;
#endif
		size = newSize;
		if (newSize < 3 || fabs(inlier_ratio - 1.0) <= 1e-9) break;
	}
#ifdef PRINT_INFO
	std::cout << "+--------------------------------------+" << std::endl;
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
	std::string actsfile1 = imgdirs[0] + "seq.act";
	std::vector<Camera> cameras1 = LoadCalibrationFromActs(actsfile1);
	std::vector<Image3D> im(cameras1.size(), Image3D(view_count, axis, rot_angle));
	CreateDir(imgdirs[0] + "Views/");
	for (int i = 0; i < im.size(); ++i){
		char imgpath[128], rawpath[128];
		sprintf_s(imgpath, "%s%05d.jpg", imgdirs[0].c_str(), i);
		sprintf_s(rawpath, "%sDATA/_depth%d.raw", imgdirs[0].c_str(), i);
		im[i].LoadModel(imgpath, rawpath, cameras1[i]);
	}

	std::string actsfile2 = imgdirs[1] + "seq.act";
	std::vector<Camera> cameras2 = LoadCalibrationFromActs(actsfile2);
	std::vector<Image3D> jm(cameras2.size(), Image3D(view_count, axis, rot_angle));
	CreateDir(imgdirs[1] + "Views/");
	for (int i = 0; i < (int)jm.size(); ++i){
		char imgpath[128], rawpath[128];
		sprintf_s(imgpath, "%s%05d.jpg", imgdirs[1].c_str(), i);
		sprintf_s(rawpath, "%sDATA/_depth%d.raw", imgdirs[1].c_str(), i);
		jm[i].LoadModel(imgpath, rawpath, cameras2[i]);
	}
#pragma endregion

#pragma region //Feature Detecting
	CreateDir(imgdirs[0] + "Views/Sift/");
	char fn[128];
	std::vector<std::string> proj_view_path1;// = ScanNSortDirectory((imgdir1 + "Views/").c_str(), "jpg");
	for (int i = 0; i < cameras1.size(); ++i){
		for (int j = 0; j < view_count; ++j){
			sprintf_s(fn, "%sViews/proj%d_%d.jpg", imgdirs[0].c_str(), i, j);
			proj_view_path1.push_back(fn);
		}
	}
	std::vector<std::vector<SiftGPU::SiftKeypoint>> keys1;
	std::vector<std::vector<float>> descs1;
	FeatureProc::DetectFeature(proj_view_path1, h_margin_ratio, v_margin_ratio, keys1, descs1);

	CreateDir(imgdirs[1] + "Views/Sift/");
	std::vector<std::string> proj_view_path2;// = ScanNSortDirectory((imgdir2 + "Views/").c_str(), "jpg");
	for (int i = 0; i < cameras2.size(); ++i){
		for (int j = 0; j < view_count; ++j){
			sprintf_s(fn, "%sViews/proj%d_%d.jpg", imgdirs[1].c_str(), i, j);
			proj_view_path2.push_back(fn);
		}
	}
	std::vector<std::vector<SiftGPU::SiftKeypoint>> keys2;
	std::vector<std::vector<float>> descs2;
	FeatureProc::DetectFeature(proj_view_path2, h_margin_ratio, v_margin_ratio, keys2, descs2);
#pragma endregion

#pragma region //Feature Matching
	std::vector<std::vector<std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>>>> matches;
	FeatureProc::MatchFeature(keys1, keys2, descs1, descs2, distmax, ratiomax, matches);
	RemoveDupPoints(im, jm, matches);

	double err = std::numeric_limits<double>::max();
	std::cout << "Remove Outliers..." << std::endl;
	int maxMatchCount = 0, frmIdx1, frmIdx2;
	for (int i = 0; i < matches.size(); ++i){
		for (int j = 0; j < matches[i].size(); ++j){
			if (matches[i][j].size() < min_match_count) continue;
			double res_err = HUGE_VAL;
			double inlier_ratio_ = 0.0;
			RemoveOutliers(im[i], jm[j], inlier_ratio_, res_err, matches[i][j]);
			if (res_err < err && matches[i][j].size() >= min_match_count){
				maxMatchCount = matches[i][j].size();
				err = res_err;
				frmIdx1 = i;
				frmIdx2 = j;
			}
		}
	}
	if (maxMatchCount < min_match_count){
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
		sprintf_s(imgpath, "%s%05d.jpg", imgdirs[0].c_str(), i);
		img1[i] = cv::imread(imgpath);
	}
	for (int i = 0; i < img2.size(); ++i){
		char imgpath[128];
		sprintf_s(imgpath, "%s%05d.jpg", imgdirs[1].c_str(), i);
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

	SRTSolver solver(iter_num);
	solver.SetInput(matchPoints, cameras1[frmIdx1], cameras2[frmIdx2]);
	solver.EstimateTransform(scale, R, t);
}

void Processor::CalcSimilarityTransformationSeq(
	const std::vector<std::vector<Camera>> &cameras,
	std::vector<double> &scales,
	std::vector<Eigen::Matrix3d> &Rs,
	std::vector<Eigen::Vector3d> &ts
	){

	char fn[128];
	std::vector<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> matchPoints;

#pragma region //Generate Views
	std::vector<std::vector<Image3D>> models(imgdirs.size());
	for (int k = 0; k < models.size(); ++k){
		//std::string actsfile = imgdirs[k] + "seq.act";
		//std::vector<Camera> cameras = LoadCalibrationFromActs(actsfile);
		models[k].resize(cameras[k].size(), Image3D(view_count, axis, rot_angle));
		CreateDir(imgdirs[k] + "Views/");
		for (int i = 0; i < models[k].size(); ++i){
			char imgpath[128], rawpath[128];
			sprintf_s(imgpath, "%s%05d.jpg", imgdirs[k].c_str(), i);
			sprintf_s(rawpath, "%sDATA/_depth%d.raw", imgdirs[k].c_str(), i);
			models[k][i].LoadModel(imgpath, rawpath, cameras[k][i]);
		}
	}
#pragma endregion

	std::vector<std::vector<cv::Mat>> imgs(models.size());
	for (int k = 0; k < imgs.size(); ++k){
		imgs[k].resize(models[k].size());
		for (int i = 0; i < imgs[k].size(); ++i){
			char imgpath[128];
			sprintf_s(imgpath, "%s%05d.jpg", imgdirs[k].c_str(), i);
			imgs[k][i] = cv::imread(imgpath);
		}
	}

#pragma region //Feature Detecting
	std::vector<std::vector<std::vector<SiftGPU::SiftKeypoint>>> keys(models.size());
	std::vector<std::vector<std::vector<float>>> descs(models.size());
	for (int k = 0; k < models.size(); ++k){
		CreateDir(imgdirs[k] + "Views/Sift/");
		CreateDir(imgdirs[k] + "Views/Sift1/");
		std::vector<std::string> proj_view_path;
		for (int i = 0; i < models[k].size(); ++i){
			for (int j = 0; j < view_count; ++j){
				sprintf_s(fn, "%sViews/proj%d_%d.jpg", imgdirs[k].c_str(), i, j);
				proj_view_path.push_back(fn);
			}
		}
		FeatureProc::DetectFeature(proj_view_path, h_margin_ratio, v_margin_ratio, keys[k], descs[k]);
		
#if 1
		//remove background points
		for (int i = 0; i < keys[k].size(); ++i){ //sequence
			const int curfrmIdx = i / view_count;
			const int view = i % view_count;
			const Image3D &curIm = models[k][curfrmIdx];
			const int w = curIm.GetWidth();
			const int h = curIm.GetHeight();
			int newSize = 0;
			for (int j = 0; j < keys[k][i].size(); ++j){ //frame
				const int idx = curIm.GetTexIndex(view, keys[k][i][j].x, keys[k][i][j].y);
				if (idx != -1){
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
			}
			keys[k][i].resize(newSize);
			descs[k][i].resize(newSize * 128);
		}
#endif
		
		for (int i = 0; i < keys[k].size(); ++i){
			const int frmIdx = i / view_count;
			const int view = i % view_count;
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

	for (int k = 0; k < models.size() - 1; ++k){
		std::cout << "|-------------------- Seq " << k << " ----> " << k + 1 << "--------------------|" << std::endl;
		std::vector<std::vector<std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>>>> matches;
		FeatureProc::MatchFeature(keys[k], keys[k + 1], descs[k], descs[k + 1], distmax, ratiomax, matches);
		RemoveDupPoints(models[k], models[k + 1], matches);

		const int w = models[k][0].GetWidth();
		const int h = models[k][0].GetHeight();
		for (int i = 0; i < matches.size(); ++i){
			for (int j = 0; j < matches[i].size(); ++j){
				int newSize = 0;
				for (int idx = 0; idx < matches[i][j].size(); ++idx){
					const std::pair<Eigen::Vector2i, Eigen::Vector2i> &pr = matches[i][j][idx];
					//int u1 = int(pr.first[0] + 0.5);
					//int v1 = int(pr.first[1] + 0.5);
					//int u2 = int(pr.second[0] + 0.5);
					//int v2 = int(pr.second[1] + 0.5);
					int u1 = pr.first[0];
					int v1 = pr.first[1];
					int u2 = pr.second[0];
					int v2 = pr.second[1];
					if (u1 >= ssd_win && v1 >= ssd_win && u2 >= ssd_win && v2 >= ssd_win &&
						u1 < w - ssd_win && v1 < h - ssd_win && u2 < w - ssd_win && v2 < h - ssd_win){
						double err = FeatureProc::SSD(imgs[k][i], pr.first[0], pr.first[1], imgs[k + 1][j], pr.second[0], pr.second[1], ssd_win);
						if (err <= ssd_err){
							matches[i][j][newSize++] = matches[i][j][idx];
						}
					}
				}
				matches[i][j].resize(newSize);
#ifdef PRINT_INFO
				std::cout << "Frame# " << i << ", " << j << " | " << newSize << " matched sift feature after ssd" << std::endl;
#endif
			}
		}

		cv::RNG rng;
#if 0
		for (int i = 0; i < matches.size(); ++i){
			for (int j = 0; j < matches[i].size(); ++j){
				cv::Mat img(imgs[k][i].rows, imgs[k][i].cols, imgs[k][i].type()); 
				cv::Mat img1(imgs[k][i].rows, imgs[k][i].cols, imgs[k][i].type());
				imgs[k][i].copyTo(img1);
				cv::Mat img2(imgs[k + 1][j].rows, imgs[k + 1][j].cols, imgs[k + 1][j].type());
				imgs[k + 1][j].copyTo(img2);
				//cv::vconcat(imgs[k][i], imgs[k + 1][j], img);
				cv::vconcat(img1, img2, img);
				for (int idx = 0; idx < (int)matches[i][j].size(); ++idx){
					uchar r = rng.uniform(0.0, 1.0) * 255;
					uchar g = rng.uniform(0.0, 1.0) * 255;
					uchar b = rng.uniform(0.0, 1.0) * 255;
					Vec2i xy1 = matches[i][j][idx].first;
					Vec2i xy2 = matches[i][j][idx].second;
					cv::circle(img, cv::Point2f(xy1[0], xy1[1]), 1, cv::Scalar(255, 255, 0));
					cv::circle(img, cv::Point2f(xy2[0], imgs[k][i].rows + xy2[1]), 1, cv::Scalar(255, 255, 0));
					cv::line(img, cv::Point2f(xy1[0], xy1[1]), cv::Point2f(xy2[0], imgs[k][i].rows + xy2[1]), cv::Scalar(b, g, r), 2);
				}
				sprintf_s(fn, "./Match1/match%d_%d_%d.jpg", k, i, j);
				cv::imshow("match1", img);
				cv::imwrite(fn, img);
				cv::waitKey(1);
			}
		}
#endif

		double err = HUGE_VAL;
		double inlier_ratio = 0.0;
		std::cout << "select keyframe..." << std::endl;
		int maxMatchCount = 0, frmIdx1, frmIdx2;
		for (int i = 0; i < matches.size(); ++i){
			for (int j = 0; j < matches[i].size(); ++j){
				if (matches[i][j].size() < min_match_count) continue;
				double inlier_ratio_ = 0.0;
				double res_err = HUGE_VAL;
				RemoveOutliers(models[k][i], models[k + 1][j], inlier_ratio_, res_err, matches[i][j]);
				if (res_err < err && matches[i][j].size() >= min_match_count){
					maxMatchCount = matches[i][j].size();
					err = res_err;
					inlier_ratio = inlier_ratio_;
					frmIdx1 = i;
					frmIdx2 = j;
				}
			}
		}
#if 1
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
		if (maxMatchCount < min_match_count){
			std::cout << "sift match: " << maxMatchCount << std::endl;
			std::cout << "residual error: " << err << std::endl;
			std::cout << "inlier ratio: " << inlier_ratio << std::endl;
			std::cerr << "No Enough Sift Feature Matches! File " << __FILE__ << ", Line " << __LINE__ << std::endl;
			exit(-1);
		}
		std::cout << "selected keyframe: " << frmIdx1 << " " << frmIdx2 << std::endl;
		std::cout << "sift match: " << matches[frmIdx1][frmIdx2].size() << std::endl;
		std::cout << "residual error: " << err << std::endl;
		std::cout << "inlier ratio: " << inlier_ratio << std::endl;
		for (int i = 0; i < matches[frmIdx1][frmIdx2].size(); ++i){
			const std::pair<Eigen::Vector2i, Eigen::Vector2i> &pr = matches[frmIdx1][frmIdx2][i];
			Eigen::Vector3d p1 = models[k][frmIdx1].GetPoint(pr.first[0], pr.first[1]);
			Eigen::Vector3d p2 = models[k + 1][frmIdx2].GetPoint(pr.second[0], pr.second[1]);
			matchPoints[k].push_back(std::make_pair(p1, p2));
		}

		SRTSolver solver(iter_num);
		solver.SetInput(matchPoints[k], models[k][frmIdx1].GetCamera(), models[k + 1][frmIdx2].GetCamera());
		solver.SetPrintFlag(false);
		solver.EstimateTransform(scales[k], Rs[k], ts[k]);
		std::cout << "pixel reproject error = " << solver.ResidualError(scales[k], Rs[k], ts[k]) << std::endl;
		for (int k0 = 0; k0 < k; ++k0){
			Rs[k0] = Rs[k] * Rs[k0];
			ts[k0] = scales[k] * Rs[k] * ts[k0] + ts[k];
			scales[k0] = scales[k] * scales[k0];
		}
		std::cout << "|------------------------------------------------------|" << std::endl;
	}
#pragma endregion
}

void Processor::AlignmentSeq(){
	srand((unsigned int)time(NULL));

	std::vector<std::vector<Camera>> cameras(imgdirs.size());
	for (int k = 0; k < cameras.size(); ++k){
		//std::string actsfile = imgdirs[k] + "seq.act";
		std::vector<std::string> actsfiles = ScanNSortDirectory(imgdirs[k].c_str(), "act");
		cameras[k] = LoadCalibrationFromActs(actsfiles[0]);
	}

	std::vector<double> scales;
	std::vector<Eigen::Matrix3d> Rs;
	std::vector<Eigen::Vector3d> ts;
	CalcSimilarityTransformationSeq(cameras, scales, Rs, ts);

	scales.push_back(1.0);
	Rs.push_back(Eigen::Matrix3d::Identity());
	ts.push_back(Eigen::Vector3d::Zero());

	int size = scales.size();
	for (int k = 0; k < size; ++k){
		std::cout << "|--------------------" << k << " ---> " << size << "--------------------|" << std::endl;
		std::cout << "Scale: " << scales[k] << std::endl;
		std::cout << "Rotation Matrix: " << std::endl;
		std::cout << Rs[k] << std::endl;
		std::cout << "Det(R): " << Rs[k].determinant() << std::endl;
		std::cout << "Translation: " << std::endl;
		std::cout << ts[k].transpose() << std::endl;
	}
	char fn[128];
#if 0
	for (int k = 0; k < size; ++k){
		std::vector<Eigen::Vector3f> points;
		std::vector<int> facets;
		std::string mpath = imgdirs[k] + "Model.obj";
		ReadObj(mpath, points, std::vector<Eigen::Vector3f>(), facets);
		for (int i = 0; i < points.size(); ++i){
			points[i] = (scales[k] * Rs[k] * points[i].cast<double>() + ts[k]).cast<float>();
		}
		sprintf_s(fn, "./ans%d.obj", k);
		WriteObj(fn, points, std::vector<Eigen::Vector3f>(), facets);
	}
#else
	std::vector<std::string> rawpaths;
	for (int k = 0; k < imgdirs.size(); ++k){
		int frmNoHalf = cameras[k].size() / 2;
		const Camera cam = cameras[k][frmNoHalf];
		int w = cam.W();
		int h = cam.H();

		sprintf_s(fn, "%sDATA/_depth%d.raw", imgdirs[k].c_str(), frmNoHalf);

		std::vector<double> dsp(w * h);
		float *raw = new float[w * h];
		std::ifstream ifs(fn, std::ifstream::in | std::ifstream::binary);
		ifs.read((char*)raw, w * h * sizeof(float));
		ifs.close();

		for (int i = 0; i < w * h; ++i) dsp[i] = raw[i];

		Depth2Model d2m(m_fMinDsp, m_fMaxDsp, 0.4);
		d2m.SaveModel(dsp, cam);

		std::vector<Eigen::Vector3d> &point3d = d2m.point3d;
		std::vector<int> facets(d2m.facets.size() * 3);
		for (int i = 0; i < d2m.facets.size(); ++i){
			facets[i * 3] = d2m.facets[i][0];
			facets[i * 3 + 1] = d2m.facets[i][1];
			facets[i * 3 + 2] = d2m.facets[i][2];
		}

		int newSize = 0;
		std::unordered_map<int, int> mp;
		for (int pIdx = 0; pIdx < point3d.size(); ++pIdx){
			const Eigen::Vector3d &p3d = point3d[pIdx];
			bool removed = false;
			for (int cIdx = 0; cIdx < cameras[k].size(); ++cIdx){
				if (cIdx != frmNoHalf){
					int u, v;
					cameras[k][cIdx].GetImgCoordFromWorld(p3d, u, v);
					if (!CheckRange(u, v, w, h)){
						removed = true;
						break;
					}
				}
			}
			if (!removed){
				point3d[newSize] = point3d[pIdx];
				mp[pIdx] = newSize;
				newSize++;
			}
		}
		std::cout << newSize << " | " << point3d.size() << std::endl;
		point3d.resize(newSize);

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
		align.RemoveGround(point3d, std::vector<Eigen::Vector3d>(), facets);

		std::vector<Eigen::Vector3f> point3f(point3d.size());
		for (int i = 0; i < point3d.size(); ++i){
			point3f[i] = (scales[k] * Rs[k] * point3d[i] + ts[k]).cast<float>();
		}
		sprintf_s(fn, "./ans%d.obj", k);
		WriteObj(fn, point3f, std::vector<Eigen::Vector3f>(), facets);
	}

#endif
}