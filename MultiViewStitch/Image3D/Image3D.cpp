#include <fstream>
#include <limits>
#include "Image3D.h"
#include "../Common/Utils.h"
#include "../PlyObj/PlyObj.h"
#include "../Depth2Model/Depth2Model.h"
#include "../Alignment/Alignment.h"
#include "../Parameter/ParamParser.h"

#if _DEBUG
#pragma comment(lib, "D:/opencv/build/x64/vc12/lib/opencv_ts300d.lib")
#pragma comment(lib, "D:/opencv/build/x64/vc12/lib/opencv_world300d.lib")
#else
#pragma comment(lib, "D:/opencv/build/x64/vc12/lib/opencv_ts300.lib")
#pragma comment(lib, "D:/opencv/build/x64/vc12/lib/opencv_world300.lib")
#endif


void Image3D::LoadModel(const std::string imgpath, const std::string rawpath, const Camera &cam){
	this->cam = cam;
	image = cv::imread(imgpath);

	if (ParamParser::isSegment){
		double scale = 0.5;
		cv::Mat imgHalf;
		cv::resize(image, imgHalf, cv::Size(image.cols * scale, image.rows * scale));
		//cv::pyrDown(image, imgHalf, image.size() / scale/*cv::Size(image.cols * scale, image.rows * scale)*/);
		int rowLeft = imgHalf.rows * ParamParser::vl_margin_ratio;
		int colLeft = imgHalf.cols * ParamParser::hl_margin_ratio;
		int rowRight = imgHalf.rows * ParamParser::vr_margin_ratio;
		int colRight = imgHalf.cols * ParamParser::hr_margin_ratio;

		cv::Rect rectangle(colLeft, rowLeft, imgHalf.cols - colRight - colLeft, imgHalf.rows - rowRight - rowLeft);

		cv::Mat bgModel, fgModel;
		cv::grabCut(imgHalf, mask, rectangle, bgModel, fgModel, 3, cv::GC_INIT_WITH_RECT);

		cv::compare(mask, cv::GC_PR_FGD, mask, cv::CMP_EQ);

		//cv::Mat foreground(imgHalf.size(), CV_8UC3, cv::Scalar(255, 255, 255));
		//imgHalf.copyTo(foreground, mask);

		cv::resize(mask, mask, image.size());
		//cv::resize(foreground, foreground, image.size());
		//cv::pyrUp(mask, mask, image.size());
		//cv::pyrUp(foreground, foreground, image.size());

		cv::imshow("mask", mask);
		//cv::imshow("segment", foreground);
		cv::waitKey(1);
	}

	int pos = imgpath.find_last_of('/');
	path = imgpath.substr(0, pos + 1) + "Views/";
	int pos1 = imgpath.find_last_of('.');
	frmNo = (int)(std::atoi(imgpath.substr(pos + 1, pos1 - pos - 1).c_str()));

	std::cout << "Load " << rawpath <<"..." << std::endl;
	std::vector<double> depth;
	LoadDepthMap(rawpath, depth);

	SolveUnProjectionD(depth);
	GenNewViews();
}

void Image3D::LoadDepthMap(const std::string rawpath, std::vector<double> &depth){
	int w = cam.W();
	int h = cam.H();
	LoadDepth(rawpath, depth, w, h);

	//int pos = rawpath.find_last_of('/');
	//char fn[128];
	//sprintf_s(fn, "%s_depth%d.jpg", rawpath.substr(0, pos + 1).c_str(), frmNo);
	//std::cout << fn << std::endl;
	//RenderDepthMap(fn, depth, w, h);
}

void Image3D::SolveUnProjectionD(const std::vector<double> &depth){
	if (ParamParser::writeMesh){
		double fMinDepth = HUGE_VAL;
		double fMaxDepth = 1.0 - HUGE_VAL;
		for (const auto & d : depth){
			fMinDepth = __min(fMinDepth, d);
			fMaxDepth = __max(fMaxDepth, d);
		}

		char fn[128];
		static int no = 0;
		sprintf_s(fn, "./mesh%d.obj", no++);
		Depth2Model d2m = Depth2Model(fMinDepth - 0.0001, fMaxDepth + 0.0001, 0.6);
		d2m.SaveModel(depth, cam, fn, false);

		std::vector<int> facets(d2m.facets.size() * 3);
		for (int i = 0; i < d2m.facets.size(); ++i){
			facets[i * 3] = d2m.facets[i][0];
			facets[i * 3 + 1] = d2m.facets[i][1];
			facets[i * 3 + 2] = d2m.facets[i][2];
		}
		Alignment align;
		align.RetainConnectRegion(d2m.point3d, std::vector<Eigen::Vector3d>(), facets);
		//std::vector<Eigen::Vector3f> point3f(d2m.point3d.size());
		//for (int i = 0; i < d2m.point3d.size(); ++i){
		//	point3f[i] = d2m.point3d[i].cast<float>();
		//}
		//WriteObj(fn, point3f, std::vector<Eigen::Vector3f>(), facets);
		WriteObj(fn, d2m.point3d, std::vector<Eigen::Vector3d>(), facets);
	}

	int w = cam.W();
	int h = cam.H();
	point3d.resize(w * h);
	valid.resize(w * h, true);
	for (int i = 0; i < w; ++i){
		for (int j = 0; j < h; ++j){
			if(depth[j * w + i] < ParamParser::m_fMinDsp ||
				depth[j * w + i] > ParamParser::m_fMaxDsp){
				valid[j * w + i] = false;
			}
			else{
				cam.GetWorldCoordFromImg(i, j, (1.0 / depth[j * w + i]), point3d[j * w + i]);
			}
		}
	}
}

void Image3D::GenNewViews(){
	char fn[128];
	static int no = 0;

	Eigen::Matrix3d R, R_, K, K_, H;
	Eigen::Vector3d t;
	cam.GetK(K);
	cam.GetRT(R, t);

	double scale = 2.0;
	double scale_ = 1.0 / scale, scale2_ = scale_ * scale_;
	int w_ = cam.W(), w = w_ * scale;
	int h_ = cam.H(), h = h_ * scale;

	K_(0, 0) = 1.0 / K(0, 0); K_(0, 1) = 0.0; K_(0, 2) = -K(0, 2) / K(0, 0);
	K_(1, 0) = 0.0; K_(1, 1) = 1.0 / K(1, 1); K_(1, 2) = -K(1, 2) / K(1, 1);
	K_(2, 0) = 0.0; K_(2, 1) = 0.0; K_(2, 2) = 1.0;
	//K_ = K.inverse();

	//Eigen::Vector3d axis(R(axis, 0), R(axis, 1), R(axis, 2));
	Eigen::Vector3d axis(R(ParamParser::axis, 0), R(ParamParser::axis, 1), R(ParamParser::axis, 2));

	std::vector<double> angle;
	for (int i = ParamParser::view_count / 2; i > 0; --i){ angle.push_back(-ParamParser::rot_angle * i); }
	for (int i = 0; i <= ParamParser::view_count / 2; ++i){ angle.push_back(ParamParser::rot_angle * i); }

	std::cout << "Generating views#: ";
	for (int k = 0; k < ParamParser::view_count; ++k){
		std::cout << k << " ";

		std::vector<int> texIndex_(w_ * h_, -1);
		cv::Mat img(h_, w_, CV_8UC3);

		RotationMatrix(angle[k] / 180 * M_PI, axis, R_);

		H = K * (R_ * K_);

		std::vector<Eigen::Vector2d> xy(w * h);
		double minu, minv, maxu, maxv;
		minu = minv = 1000000000.0;
		maxu = maxv = -1000000000.0;
		for (int i = 0; i < w * h; ++i){
			int u = i % w - w * scale2_;
			int v = i / w - h * scale2_;
			double wf = H(2, 0) * u + H(2, 1) * v + H(2, 2);
			double uf = (H(0, 0) * u + H(0, 1) * v + H(0, 2)) / wf;
			double vf = (H(1, 0) * u + H(1, 1) * v + H(1, 2)) / wf;
			if (CheckRange(uf, vf, w_, h_)){
				minu = min(minu, u + w * scale2_);
				minv = min(minv, v + h * scale2_);
				maxu = max(maxu, u + w * scale2_);
				maxv = max(maxv, v + h * scale2_);
			}
			xy[i] = Eigen::Vector2d(uf, vf);
		}
		double centerx = (maxu + minu) * 0.5;
		double centery = (maxv + minv) * 0.5;
		double offsetx = centerx - w * scale2_;
		double offsety = centery - h * scale2_;


		for (int i = 0; i < w * h; ++i){
			double uf = xy[i][0];
			double vf = xy[i][1];

			double u11 = floor(uf), v11 = floor(vf);
			double u22 = ceil(uf), v22 = ceil(vf);
			int u = int(i % w - offsetx + 0.5);
			int v = int(i / w - offsety + 0.5);
			if (CheckRange(u11, v11, w_, h_) && CheckRange(u22, v22, w_, h_) && CheckRange(u, v, w_, h_)){
				if (fabs(u11 - u22) <= 1e-9 && fabs(v11 - v22) <= 1e-9){
					img.at<cv::Vec3b>(v, u) = image.at<cv::Vec3b>(v11, u11);
					texIndex_[v * w_ + u] = (v11 * w_ + u11);
				}
				else{
					cv::Vec3b rgb11 = image.at<cv::Vec3b>(v11, u11);
					cv::Vec3b rgb12 = image.at<cv::Vec3b>(v22, u11);
					cv::Vec3b rgb21 = image.at<cv::Vec3b>(v11, u22);
					cv::Vec3b rgb22 = image.at<cv::Vec3b>(v22, u22);
					cv::Vec3b rgb;
					if (fabs(u11 - u22) <= 1e-9){
						double s1 = (vf - v11) / (v22 - v11);
						double s2 = 1 - s1;
						rgb[0] = uchar(rgb11[0] * s2 + rgb12[0] * s1);
						rgb[1] = uchar(rgb11[1] * s2 + rgb12[1] * s1);
						rgb[2] = uchar(rgb11[2] * s2 + rgb12[2] * s1);
					}
					else if (fabs(v11 - v22) <= 1e-9){
						double s1 = (uf - u11) / (u22 - u11);
						double s2 = 1 - s1;
						rgb[0] = uchar(rgb11[0] * s2 + rgb21[0] * s1);
						rgb[1] = uchar(rgb11[1] * s2 + rgb21[1] * s1);
						rgb[2] = uchar(rgb11[2] * s2 + rgb21[2] * s1);
					}
					else{
						double s1 = (u22 - uf) * (v22 - vf);
						double s2 = (uf - u11) * (v22 - vf);
						double s3 = (u22 - uf) * (vf - v11);
						double s4 = (uf - u11) * (vf - v11);
						rgb[0] = uchar(rgb11[0] * s1 + rgb21[0] * s2 + rgb12[0] * s3 + rgb22[0] * s4);
						rgb[1] = uchar(rgb11[1] * s1 + rgb21[1] * s2 + rgb12[1] * s3 + rgb22[1] * s4);
						rgb[2] = uchar(rgb11[2] * s1 + rgb21[2] * s2 + rgb12[2] * s3 + rgb22[2] * s4);
					}
					img.at<cv::Vec3b>(v, u) = rgb;
					texIndex_[v * w_ + u] = (int(vf + 0.5) * w_ + int(uf + 0.5));
				}
			}
		}
		texIndex.push_back(texIndex_);
		sprintf_s(fn, "%s/proj%d_%d.jpg", path.c_str(), frmNo, k);
		cv::imwrite(fn, img);
	}
	std::cout << std::endl;
}