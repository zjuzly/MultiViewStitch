#include "SRTSolver.h"
#include "../Common/Utils.h"
#include <fstream>
#include <iostream>

double SRTSolver::ResidualError(const double scale, Eigen::Matrix3d &R, Eigen::Vector3d &t){
	double err = 0.0;
	for (int i = 0; i < matches.size(); ++i){
		const Eigen::Vector3d &p1 = matches[i].first;
		const Eigen::Vector3d &p2 = matches[i].second;
		
		Eigen::Vector3d tp = scale * R * p1 + t;
		int u1, v1, u2, v2;
		cam2.GetImgCoordFromWorld(tp, u1, v1);
		cam2.GetImgCoordFromWorld(p2, u2, v2);

		//err = err + sqrt((u1 - u2) * (u1 - u2) + (v1 - v2) * (v1 - v2));

		Eigen::Vector3d tp_ = 1.0 / scale * R.transpose() * (p2 - t);
		int u1_, v1_, u2_, v2_;
		cam1.GetImgCoordFromWorld(tp_, u2_, v2_);
		cam1.GetImgCoordFromWorld(p1, u1_, v1_);
		double err1 = sqrt((u1 - u2) * (u1 - u2) + (v1 - v2) * (v1 - v2));
		double err2 = sqrt((u1_ - u2_) * (u1_ - u2_) + (v1_ - v2_) * (v1_ - v2_));
		err = err + (err1 + err2) * 0.5;
	}
	err /= matches.size();
	return err;
}

double SRTSolver::EstimateScale(){
	double scale = 0.0;
	int size = matches.size();
	Eigen::Vector3d baryCenter1(0.0, 0.0, 0.0);
	Eigen::Vector3d baryCenter2(0.0, 0.0, 0.0);
	for (int i = 0; i < size; ++i){
		baryCenter1 = baryCenter1 + matches[i].first;
		baryCenter2 = baryCenter2 + matches[i].second;
	}
	baryCenter1 = baryCenter1 / size;
	baryCenter2 = baryCenter2 / size;
	for (int i = 0; i < size; ++i){
		scale += (matches[i].second - baryCenter2).norm() / (matches[i].first - baryCenter1).norm();
	}
	return scale / size;
}
double SRTSolver::EstimateScaleRansac(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &matches_){
	double scale = 0.0;
	int size = matches_.size();
	Eigen::Vector3d baryCenter1(0.0, 0.0, 0.0);
	Eigen::Vector3d baryCenter2(0.0, 0.0, 0.0);
	for (int i = 0; i < size; ++i){
		baryCenter1 = baryCenter1 + matches_[i].first;
		baryCenter2 = baryCenter2 + matches_[i].second;
	}
	baryCenter1 = baryCenter1 / size;
	baryCenter2 = baryCenter2 / size;
	for (int i = 0; i < size; ++i){
		scale += (matches_[i].second - baryCenter2).norm() / (matches_[i].first - baryCenter1).norm();
	}
	return scale / size;
}

//http://blog.csdn.net/kfqcome/article/details/9358853
void SRTSolver::EstimateRT(const double scale, Eigen::Matrix3d &R, Eigen::Vector3d &t){
	int size = matches.size();
	Eigen::Vector3d baryCenter1(0.0, 0.0, 0.0);
	Eigen::Vector3d baryCenter2(0.0, 0.0, 0.0);
	for (int i = 0; i < size; ++i){
		baryCenter1 = baryCenter1 + matches[i].first;
		baryCenter2 = baryCenter2 + matches[i].second;
	}
	baryCenter1 = baryCenter1 / size;
	baryCenter2 = baryCenter2 / size;
	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> matches_(size);
	for (int i = 0; i < size; ++i){
		matches_[i].first = (matches[i].first - baryCenter1) * scale;
		matches_[i].second = matches[i].second - baryCenter2;
	}

#if 0
	std::ofstream ofs1("./middle1.obj", std::ofstream::out);
	for (int i = 0; i < matches_.size(); ++i){
		ofs1 << "v " << matches_[i].first[0] << " " << matches_[i].first[1] << " " << matches_[i].first[2] << std::endl;
	}
	ofs1.close();
	std::ofstream ofs2("./middle2.obj", std::ofstream::out);
	for (int i = 0; i < matches_.size(); ++i){
		ofs2 << "v " << matches_[i].second[0] << " " << matches_[i].second[1] << " " << matches_[i].second[2] << std::endl;
	}
	ofs2.close();
#endif
	
	Eigen::MatrixXd X(3, size);
	for (int i = 0; i < size; ++i){
		X(0, i) = matches_[i].first[0];
		X(1, i) = matches_[i].first[1];
		X(2, i) = matches_[i].first[2];
	}
	Eigen::MatrixXd Y(3, size);
	for (int i = 0; i < size; ++i){
		Y(0, i) = matches_[i].second[0];
		Y(1, i) = matches_[i].second[1];
		Y(2, i) = matches_[i].second[2];
	}
	Y.transposeInPlace();
	Eigen::Matrix3d S = X * Y;

	Eigen::JacobiSVD<Eigen::Matrix3d> svd(S, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();
	Eigen::Matrix3d UT = U;
	UT.transposeInPlace();
	R = V * UT;
	if (fabs(R.determinant() + 1) <= 1e-9){ //如果R是反射矩阵，需对R进行纠正
		Eigen::Matrix3d Sigma = Eigen::Matrix3d::Identity();
		Sigma(2, 2) = -1;
		R = V * Sigma * UT; 
	}
	t = baryCenter2 - scale * R * baryCenter1;
	double res_err = ResidualError(scale, R, t);
	if (isPrint){
		std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;
		std::cout << "Its left singular vectors are the columns of the thin U matrix:" << std::endl << U << std::endl;
		std::cout << "Its right singular vectors are the columns of the thin V matrix:" << std::endl << V << std::endl;
		std::cout << "det(R): " << R.determinant() << std::endl;
		std::cout << "pixel reproject error = " << res_err << std::endl;
	}
}

void SRTSolver::EstimateRTRansac(const double scale, Eigen::Matrix3d &R, Eigen::Vector3d &t){
	int size = matches.size();
	Eigen::Vector3d baryCenter1(0.0, 0.0, 0.0);
	Eigen::Vector3d baryCenter2(0.0, 0.0, 0.0);
	for (int i = 0; i < size; ++i){
		baryCenter1 = baryCenter1 + matches[i].first;
		baryCenter2 = baryCenter2 + matches[i].second;
	}
	baryCenter1 = baryCenter1 / size;
	baryCenter2 = baryCenter2 / size;

	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> matches_(size);
	for (int i = 0; i < size; ++i){
		matches_[i].first = (matches[i].first - baryCenter1) * scale;
		matches_[i].second = matches[i].second - baryCenter2;
	}

	double err = HUGE_VAL;//std::numeric_limits<double>::max();
	for (int k = 0; k < iter_num; ++k){
		if(isPrint) std::cout << "Iteration#" << k << " | ";
		int idx[3];
		Shuffle(idx, size, 3);
		Eigen::Matrix3d X, Y;
		for (int i = 0; i < 3; ++i){
			X(0, i) = matches_[idx[i]].first[0];
			X(1, i) = matches_[idx[i]].first[1];
			X(2, i) = matches_[idx[i]].first[2];
			Y(0, i) = matches_[idx[i]].second[0];
			Y(1, i) = matches_[idx[i]].second[1];
			Y(2, i) = matches_[idx[i]].second[2];
		}
		Y.transposeInPlace();
		Eigen::Matrix3d S = X * Y;

		Eigen::JacobiSVD<Eigen::Matrix3d> svd(S, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();
		Eigen::Matrix3d UT = U;
		UT.transposeInPlace();
		Eigen::Matrix3d R_ = V * UT;
		if (fabs(R_.determinant() + 1) <= 1e-9){ //如果R是反射矩阵，需对R进行纠正
			Eigen::Matrix3d Sigma = Eigen::Matrix3d::Identity();
			Sigma(2, 2) = -1;
			R_ = V * Sigma * UT;
		}
		Eigen::Vector3d t_ = baryCenter2 - scale * R_ * baryCenter1;
		double res_err = ResidualError(scale, R_, t_);
		if (res_err < err){
			err = res_err;
			R = R_;
			t = t_;
		}
		if (isPrint) std::cout << "pixel reproject error = " << err << std::endl;
	}
}

void SRTSolver::EstimateSRTRansac(double &scale, Eigen::Matrix3d &R, Eigen::Vector3d &t){
	int size = matches.size();
	srand((unsigned int)time(NULL));
	double err = HUGE_VAL;// std::numeric_limits<double>::max();
	for (int k = 0; k < iter_num; ++k){
		if (isPrint) std::cout << "Iteration#" << k << "..." << std::endl;
		int idx[3];
		while (true){
			for (int j = 0; j < 3; ++j){
				idx[j] = rand() % size;
			}
			if (idx[0] != idx[1] && idx[0] != idx[2] && idx[1] != idx[2]) break;
		}

		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> matches_{matches[idx[0]], matches[idx[1]], matches[idx[2]]};
		double scale_ = EstimateScaleRansac(matches_);

		Eigen::Vector3d baryCenter1(0.0, 0.0, 0.0);
		Eigen::Vector3d baryCenter2(0.0, 0.0, 0.0);
		for (int i = 0; i < 3; ++i){
			baryCenter1 = baryCenter1 + matches_[idx[i]].first;
			baryCenter2 = baryCenter2 + matches_[idx[i]].second;
		}
		baryCenter1 = baryCenter1 / 3;
		baryCenter2 = baryCenter2 / 3;
		for (int i = 0; i < 3; ++i){
			matches_[i].first = (matches_[idx[i]].first - baryCenter1) * scale_;
			matches_[i].second = matches_[idx[i]].second - baryCenter2;
		}

		Eigen::Matrix3d X;
		for (int i = 0; i < 3; ++i){
			X(0, i) = matches_[idx[i]].first[0];
			X(1, i) = matches_[idx[i]].first[1];
			X(2, i) = matches_[idx[i]].first[2];
		}
		Eigen::Matrix3d Y;
		for (int i = 0; i < 3; ++i){
			Y(0, i) = matches_[idx[i]].second[0];
			Y(1, i) = matches_[idx[i]].second[1];
			Y(2, i) = matches_[idx[i]].second[2];
		}
		Y.transposeInPlace();
		Eigen::Matrix3d S = X * Y;

		Eigen::JacobiSVD<Eigen::Matrix3d> svd(S, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();

		Eigen::Matrix3d UT = U;
		UT.transposeInPlace();
		Eigen::Matrix3d R_ = V * UT;
		if (R_.determinant() == -1){ //如果R是反射矩阵，需对R进行纠正
			Eigen::Matrix3d Sigma = Eigen::Matrix3d::Identity();
			Sigma(2, 2) = -1;
			R_ = V * Sigma * UT;
		}
		Eigen::Vector3d t_ = baryCenter2 - scale_ * R_ * baryCenter1;
		double res_err = ResidualError(scale_, R_, t_);
		if (res_err < err){
			err = res_err;
			R = R_;
			t = t_;
			scale = scale_;
		}
		if (isPrint) std::cout << "residual error: " << err << std::endl;
	}
}

void SRTSolver::EstimateTransform(double &scale, double R[3][3], double t[3]){
	Eigen::Matrix3d R_;
	Eigen::Vector3d t_;
#if 1
	scale = EstimateScale();
	//EstimateRT(scale, R_, t_);
	EstimateRTRansac(scale, R_, t_);
#else
	EstimateSRTRansac(scale, R_, t_);
#endif
	R[0][0] = R_(0, 0); R[0][1] = R_(0, 1); R[0][2] = R_(0, 2);
	R[1][0] = R_(1, 0); R[1][1] = R_(1, 1); R[1][2] = R_(1, 2);
	R[2][0] = R_(2, 0); R[2][1] = R_(2, 1); R[2][2] = R_(2, 2);
	t[0] = t_(0); t[1] = t_(1); t[2] = t_(2);
}

void SRTSolver::EstimateTransform(double &scale, Eigen::Matrix3d &R, Eigen::Vector3d &t){
	scale = EstimateScale();
	EstimateRT(scale, R, t);
}

void SRTSolver::EstimateTransformRansac(double &scale, Eigen::Matrix3d &R, Eigen::Vector3d &t){
	scale = EstimateScale();
	EstimateRTRansac(scale, R, t);
}