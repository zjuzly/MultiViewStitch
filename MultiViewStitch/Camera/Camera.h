#ifndef CAMERA_H
#define CAMERA_H
#include <string>
#include <vector>
#include <Eigen/Dense>

class Camera{
public:
	void SetK(Eigen::Matrix3d &K){ 
		this->K = K;
	}
	void SetRT(Eigen::Matrix3d &R, Eigen::Vector3d &t){
		this->R = R;
		this->t = t;
	}
	void SetW(int w){ this->w = w; }
	void SetH(int h){ this->h = h; }
	void GetK(Eigen::Matrix3d &K) const{
		K = this->K;
	}
	void GetRT(Eigen::Matrix3d &R, Eigen::Vector3d &t) const{
		R = this->R;
		t = this->t;
	}
	int W(){ return w; }
	int W() const{ return w; }
	int H(){ return h; }
	int H() const{ return h; }

	void ValidateModel(){ valid = true; }
	bool isValid(){ return valid; }

	Eigen::Matrix4f GetObjAbsTransformGL();
	void GetFrustumGL(float znear, float &left, float &right, float &bottom, float &top);
	Eigen::Matrix4f GetProjectGL(float left, float right, float bottom, float top, float znear, float zfar);

	void GetCamCoordFromImg(const int u, const int v, const double d, Eigen::Vector3d &p3d) const;
	void GetImgCoordFromCam(const Eigen::Vector3d &p3d, int &u, int &v) const;
	void GetWorldCoordFromImg(const int u, const int v, const double d, Eigen::Vector3d &p3d) const;
	void GetImgCoordFromWorld(const Eigen::Vector3d &p3d, int &u, int &v) const;
	void GetWorldCoordFromCam(const Eigen::Vector3d &p3d_c, Eigen::Vector3d &p3d_w) const;
	void GetCamCoordFromWorld(const Eigen::Vector3d &p3d_w, Eigen::Vector3d &p3d_c) const;
private:
	Eigen::Matrix3d K;
	Eigen::Matrix3d R;
	Eigen::Vector3d t;

	int w, h;
	bool valid;
};

std::vector<Camera> LoadCalibrationFromActs( std::string filename);

#endif