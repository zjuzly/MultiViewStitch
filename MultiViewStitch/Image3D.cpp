#include <fstream>
#include <limits>
#include <opencv2\opencv.hpp>
#include "Image3D.h"
#include "Utils.h"
#include "Depth2Model.h"

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
	std::vector<double>().swap(depth);
	depth.resize(w * h);
	float *raw = new float[w * h];
	std::ifstream ifs(rawpath.c_str(), std::ifstream::in | std::ifstream::binary);
	ifs.read((char*)raw, w * h * sizeof(float));
	for (int i = 0; i < w * h; ++i){ depth[i] = raw[i]; }
	delete[]raw;
	ifs.close();
}

void Image3D::SolveUnProjectionD(const std::vector<double> &depth){
	//double fMinDepth = std::numeric_limits<double>::max();
	//double fMaxDepth = std::numeric_limits<double>::min();
	//for (const auto & d : depth){
	//	fMinDepth = std::min(fMinDepth, d);
	//	fMaxDepth = std::max(fMaxDepth, d);
	//}
	//
	//char fn[128];
	//static int no = 0;
	//sprintf_s(fn, "./mesh%d.obj", no++);
	//d2m = Depth2Model(fMinDepth - 0.0001, fMaxDepth + 0.0001, 2.0);
	//d2m.SaveModel(depth, cam, fn);
	int w = cam.W();
	int h = cam.H();
	point3d.resize(w * h);
	valid.resize(w * h, true);
	for (int i = 0; i < w; ++i){
		for (int j = 0; j < h; ++j){
			if (depth[j * w + i] <= 1e-6){
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
	//std::cout << K_ * K << std::endl;

	Eigen::Vector3d axis(R(1, 0), R(1, 1), R(1, 2));

	std::vector<double> angle;
	double step = 10.0;
	for (int i = viewCount / 2; i > 0; --i){ angle.push_back(-step * i); }
	for (int i = 0; i <= viewCount / 2; ++i){ angle.push_back(step * i); }

	std::cout << "Generating views#: ";
	for (int k = 0; k < viewCount; ++k){
		std::cout << k << " ";

		std::vector<int> texIndex_(w_ * h_, -1);
		cv::Mat img(h_, w_, CV_8UC3);

		RotationMatrix(angle[k] / 180 * M_PI, axis, R_);

		H = K * (R_ * K_);
		//Eigen::Matrix3d H_;
		//for(int i = 0; i < 3; ++i){
		//	H_(i, 0) = R_(i, 0) * K_(0, 0) + R_(i, 1) * K_(1, 0) + R_(i, 2) * K_(2, 0);
		//	H_(i, 1) = R_(i, 0) * K_(0, 1) + R_(i, 1) * K_(1, 1) + R_(i, 2) * K_(2, 1);
		//	H_(i, 2) = R_(i, 0) * K_(0, 2) + R_(i, 1) * K_(1, 2) + R_(i, 2) * K_(2, 2);
		//}
		//for(int i = 0; i < 3; ++i){
		//	H(i, 0) = K(i, 0) * H_(0, 0) + K(i, 1) * H_(1, 0) + K(i, 2) * H_(2, 0);
		//	H(i, 1) = K(i, 0) * H_(0, 1) + K(i, 1) * H_(1, 1) + K(i, 2) * H_(2, 1);
		//	H(i, 2) = K(i, 0) * H_(0, 2) + K(i, 1) * H_(1, 2) + K(i, 2) * H_(2, 2);
		//}

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
					//double s = (u22 - u11) * (v22 - v11);
					double s1 = (u22 - uf) * (v22 - vf);// / s;
					double s2 = (uf - u11) * (v22 - vf);// / s;
					double s3 = (u22 - uf) * (vf - v11);// / s;
					double s4 = (uf - u11) * (vf - v11);// / s;
					cv::Vec3b rgb;
					rgb[0] = uchar(rgb11[0] * s1 + rgb21[0] * s2 + rgb12[0] * s3 + rgb22[0] * s4);
					rgb[1] = uchar(rgb11[1] * s1 + rgb21[1] * s2 + rgb12[1] * s3 + rgb22[1] * s4);
					rgb[2] = uchar(rgb11[2] * s1 + rgb21[2] * s2 + rgb12[2] * s3 + rgb22[2] * s4);
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