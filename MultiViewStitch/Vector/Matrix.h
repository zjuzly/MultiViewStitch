#ifndef MATRIX_H
#define MATRIX_H
#include <fstream>
#include <iostream>
#include <iomanip>
#include <Eigen\Dense>
#include "Vector.h"

template<typename Real, int M, int N>
class Mtx{
public:
	typedef Mtx<Real, M, N> Self;

	Mtx(){
		for (int i = 0; i < M; ++i){
			for (int j = 0; j < N; ++j){
				x[i][j] = 0;
			}
		}
	}
	
	Mtx(const Self &other){
		for (int i = 0; i < M; ++i){
			for (int j = 0; j < N; ++j){
				x[i][j] = other(i, j);
			}
		}
	}

	Mtx(Real x_[M][N]){
		for (int i = 0; i < M; ++i){
			for (int j = 0; j < N; ++j){
				x[i][j] = x_[i][j];
			}
		}
	}

	Real &operator()(int i, int j){ return x[i][j]; }
	const Real &operator()(int i, int j) const{ return x[i][j]; }
	//Real &operator[](int i, int j){ return x[i][j]; }
	//const Real &operator[](int i, int j) const{ return x[i][j]; }

	Self operator + (const Self &other) const{
		Self ret;
		for (int i = 0; i < M; ++i){
			for (int j = 0; j < N; ++j){
				ret(i, j) = x[i][j] + other(i, j);
			}
		}
		return ret;
	}

	Self operator - (const Self &other) const{
		Self ret;
		for (int i = 0; i < M; ++i){
			for (int j = 0; j < N; ++j){
				ret(i, j) = x[i][j] - other(i, j);
			}
		}
		return ret;
	}

	Self operator * (const Real s) const{
		Self ret; 
		for (int i = 0; i < M; ++i){
			for (int j = 0; j < N; ++j){
				ret(i, j) = x[i][j] * s;
			}
		}
		return ret;
	}

	Self operator / (const Real s) const{
		Self ret;
		for (int i = 0; i < M; ++i){
			for (int j = 0; j < N; ++j){
				ret(i, j) = x[i][j] / s;
			}
		}
		return ret;
	}

	Mtx<Real, N, M> transpose() const{
		Mtx<Real, N, M> ret;
		for (int i = 0; i < M; ++i){
			for (int j = 0; j < N; ++j){
				ret(i, j) = x[i][j];
			}
		}
		return ret;
	}

	Self inverse() const{
		if (M != N){
			std::cout << "ERROR: Matrix rows and cols don't match! File " << __FILE__ << ", Line " << __LINE__ << std::endl;
			exit(-1);
		}
		Self ret;
		Eigen::MatrixXd m(M, M);
		for (int i = 0; i < M; ++i){
			for (int j = 0; j < M; ++j){
				m(i, j) = x[i][j];
			}
		}
		Eigen::MatrixXd minv = m.inverse();
		std::cout << minv << std::endl;
		for (int i = 0; i < M; ++i){
			for (int j = 0; j < M; ++j){
				ret(i, j) = minv(i, j);
			}
		}
		return ret;
	}

	Real det() const{
		if (M != N){
			std::cout << "ERROR: Matrix rows and cols don't match! File " << __FILE__ << ", Line " << __LINE__ << std::endl;
			exit(-1);
		}
		Eigen::MatrixXd m(M, M);
		for (int i = 0; i < M; ++i){
			for (int j = 0; j < M; ++j){
				m(i, j) = x[i][j];
			}
		}
		return m.determinant();
	}

	static Self identity(){
		Self ret;
		for (int i = 0; i < __min(M, N); ++i){
			ret(i, i) = 1;
		}
		return ret;
	}

private:
	Real x[M][N];
};

typedef Mtx<int, 3, 3> Mtx3i;
typedef Mtx<int, 4, 4> Mtx4i;
typedef Mtx<float, 3, 3> Mtx3f;
typedef Mtx<float, 4, 4> Mtx4f;
typedef Mtx<double, 3, 3> Mtx3d;
typedef Mtx<double, 4, 4> Mtx4d;

//M * N x M_ * N_ = M * N_
template<typename Real, int M, int N, int K>
Mtx<Real, M, K> operator * (const Mtx<Real, M, N> &other1, const Mtx<Real, N, K> &other2){
	Mtx<Real, M, K> ret;
	for (int i = 0; i < M; ++i){
		for (int j = 0; j < K; ++j){
			for (int k = 0; k < N; ++k){
				ret(i, j) += other1(i, k) * other2(k, j);
				//ret[i][j] += other1[i][k] * other2[k][j];
			}
		}
	}
	return ret;
}

template<typename Real, int M, int N>
Vec<Real, M> operator * (const Mtx<Real, M, N> &other, const Vec<Real, N> &vec){
	Vec<Real, M> ret;
	for (int i = 0; i < M; ++i){
		for (int j = 0; j < N; ++j){
			ret[i] = ret[i] + other(i, j) * vec[j];
		}
	}
	return ret;
}

template<typename Real, int M, int N>
Vec<Real, N> operator * (const Vec<Real, M> &vec, const Mtx<Real, M, N> &other){
	Vec<Real, N> ret;
	for (int i = 0; i < N; ++i){
		for (int j = 0; j < M; ++j){
			ret[i] = ret[i] + other(j, i) * vec[j];
		}
	}
	return ret;
}

template<typename Real, int M, int N>
Mtx<Real, M, N> operator * (const Mtx<Real, M, N> &other, Real scale){
	Mtx<Real, M, N> ret;
	for (int i = 0; i < M; ++i){
		for (int j = 0; j < N; ++j){
			ret(i, j) = other(i, j) * scale;
		}
	}
	return ret;
}

template<typename Real, int M, int N>
Mtx<Real, M, N> operator * (Real scale, const Mtx<Real, M, N> &other){
	return other * scale;
}

template<typename Real, int M, int N>
std::ostream& operator << (std::ostream &out, Mtx<Real, M, N> &other){
	out << std::setprecision(5);
	for (int i = 0; i < M; ++i){
		out << std::setfill(' ') << std::setw(8) << other(i, 0);
		for (int j = 1; j < N; ++j){
			out << std::setfill(' ') << std::setw(8) << other(i, j);
		}
		if(i < M - 1) out << std::endl;
	}
	out << std::setprecision(6);
	return out;
}

template<typename Real, int M, int N>
std::istream& operator << (std::istream &in, Mtx<Real, M, N> &other){
	for (int i = 0; i < M; ++i){
		for (int j = 0; j < N; ++j){
			in >> other(i, j);
		}
	}
	return in;
}

#endif