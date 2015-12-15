#ifndef VECTOR_H
#define VECTOR_H

#if 1
#include <iostream>
#include <Eigen/Dense>

template<typename Real, int D>
class Vec{
public:
	typedef Vec<Real, D> Self;
	
	Vec(){ for (int i = 0; i < D; ++i) x[i] = 0; }
	Vec(const Self &other){ for (int i = 0; i < D; ++i) x[i] = other[i]; }
	Vec(const Real x_, const Real y_){ 
		CheckDim(2);
		x[0] = x_; x[1] = y_;
	}
	Vec(const Real x_, const Real y_, const Real z_){
		CheckDim(3);
		x[0] = x_; x[1] = y_; x[2] = z_;
	}
	Vec(const Real x_, const Real y_, const Real z_, const Real w_){
		CheckDim(4);
		x[0] = x_; x[1] = y_; x[2] = z_; x[3] = w_;
	}

	Real& operator[](int i){ return x[i]; }
	const Real& operator[](int i) const{ return x[i]; }

	Self operator+(const Self &other) const{ 
		Self ret;
		for (int i = 0; i < D; ++i){ ret[i] = x[i] + other[i]; } 
		return ret; 
	}
	Self operator-(const Self &other) const{
		Self ret;
		for (int i = 0; i < D; ++i){ ret[i] = x[i] - other[i]; } 
		return ret; 
	}
	Self operator-() const{
		Self ret;
		for (int i = 0; i < D; ++i){ ret[i] = -x[i]; }
		return ret; 
	}
	Self operator*(const Real s) const{
		Self ret;
		for (int i = 0; i < D; ++i){ ret[i] = x[i] * s; }
		return ret; 
	}
	Self operator/(const Real s) const{
		Self ret;
		for (int i = 0; i < D; ++i){ ret[i] = x[i] / s; } 
		return ret; 
	}

	bool operator < (const Self &other) const{
		for (int i = 0; i < D; ++i){
			if (x[i] < other[i]) return true;
			else if (x[i] > other[i]) return false;
		}
		return false;
	}

	Real Length() const{ 
		Real res = 0;
		for (int i = 0; i < D; ++i){
			res += x[i] * x[i];
		}
		return std::sqrt(res);
	}
	void Normalize(){
		Real len = Length();
		for (int i = 0; i < D; ++i){
			x[i] /= len;
		}
	}
private:
	void CheckDim(int m){
		if (D != m){
			std::cerr << "ERROR: the function parameters dimension mismatch!" << std::endl;
			exit(-1);
		}
	}

	Real x[D];
};

typedef Vec<double, 4> Vec4d;
typedef Vec<float,	4> Vec4f;
typedef Vec<int,	4> Vec4i;
typedef Vec<double, 3> Vec3d;
typedef Vec<float,	3> Vec3f;
typedef Vec<int,	3> Vec3i;
typedef Vec<double, 2> Vec2d;
typedef Vec<float,	2> Vec2f;
typedef Vec<int,	2> Vec2i;

template<typename Real, int D>
std::ostream& operator << (std::ostream &out, Vec<Real, D> &other){
	out << std::setprecision(5);
	for (int i = 0; i < D; ++i){
		out << std::setfill(' ') << std::setw(8) << other[i];
	}
	out << std::setprecision(6);
	return out;
}

template<typename Real, int D>
std::istream& operator << (std::istream &in, Vec<Real, D> &other){
	for (int i = 0; i < D; ++i){
		in >> other[i];
	}
	return in;
}

#else
#include <Eigen/Dense>
typedef Eigen::VectorXd VecXd;
typedef Eigen::VectorXf VecXf;
typedef Eigen::VectorXi VecXi;
typedef Eigen::Vector4d Vec4d;
typedef Eigen::Vector4f Vec4f;
typedef Eigen::Vector4i Vec4i;
typedef Eigen::Vector3d Vec3d;
typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector3i Vec3i;
typedef Eigen::Vector2d Vec2d;
typedef Eigen::Vector2f Vec2f;
typedef Eigen::Vector2i Vec2i;

typedef Eigen::MatrixXd MatrixXd;
typedef Eigen::MatrixXf MatrixXf;
typedef Eigen::MatrixXi MatrixXi;
typedef Eigen::Matrix4d Matrix4d;
typedef Eigen::Matrix4f Matrix4f;
typedef Eigen::Matrix4i Matrix4i;
typedef Eigen::Matrix3d Matrix3d;
typedef Eigen::Matrix3f Matrix3f;
typedef Eigen::Matrix3i Matrix3i;
typedef Eigen::Matrix2d Matrix2d;
typedef Eigen::Matrix2f Matrix2f;
typedef Eigen::Matrix2i Matrix2i;
#endif

#endif