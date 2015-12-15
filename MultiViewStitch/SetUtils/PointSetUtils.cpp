#include "PointSetUtils.h"

void PointSetUtils::SetInput(const std::vector<Eigen::Vector3d> &points_){
	std::vector<Eigen::Vector3d>().swap(points);
	std::copy(points_.begin(), points_.end(), std::back_inserter(points));
	CalcBarycenter();
	CalcBoundingBox();
}
void PointSetUtils::CalcPivots(Eigen::MatrixXd &pivots, int pivot_num){
	using namespace std;
	using namespace Eigen;

	assert(pivot_num <= 3 && pivot_num > 0);

	int size = (int)points.size();
	vector<Vector3d> points_ = points;
	for (int i = 0; i < size; ++i){ points_[i] -= baryCenter; }

	MatrixXd Ci(size, pivot_num);
	for (int i = 0; i < size; ++i){
		Ci(i, 0) = points_[i][0];
		Ci(i, 1) = points_[i][1];
		Ci(i, 2) = points_[i][2];
	}

	MatrixXd C = Ci.transpose() * Ci / (size - 1);
	SelfAdjointEigenSolver<MatrixXd> solver(C);
#if 0
	cout << "+--------------------------------------------+" << endl;
	cout << "Solver Statue: " << solver.info() << endl;
	cout << "eigenval: " << solver.eigenvalues().transpose() << endl;
	cout << "+--------------------------------------------+" << endl;
#endif
	pivots = MatrixXd(3, pivot_num);
	MatrixXd vectors = solver.eigenvectors();
	for (int i = 0; i < pivot_num; ++i){
		pivots.col(i) = vectors.col(2 - i);
		pivots.col(i).normalize();
	}
	return;
}

void PointSetUtils::CalcBarycenter(){
	baryCenter = Eigen::Vector3d(0.0, 0.0, 0.0);
	for (int i = 0; i < (int)points.size(); ++i){ baryCenter += points[i]; }
	baryCenter /= points.size();
}
void PointSetUtils::CalcBoundingBox(){
	using namespace Eigen;
	Vector3d lt = points[0];
	Vector3d rb = points[0];
	for (int i = 1; i < (int)points.size(); ++i){
		lt[0] = __min(lt[0], points[i][0]);
		lt[1] = __min(lt[1], points[i][1]);
		lt[2] = __min(lt[2], points[i][2]);
		rb[0] = __max(rb[0], points[i][0]);
		rb[1] = __max(rb[1], points[i][1]);
		rb[2] = __max(rb[2], points[i][2]);
	}
	leftTop = lt;
	rightBot = rb;
}