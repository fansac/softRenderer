#include "util.hpp"


bool util_rd::is_file_exists_ifstream(const std::string name) {
	std::ifstream f(name.c_str());
	return f.good();
}

std::string util_rd::find_file_name(const std::string file_path) {
	size_t start = file_path.find_last_of('/') + 1;
	size_t end = file_path.find_last_of('.');
	std::string s = file_path.substr(start, end - start);
	return file_path.substr(start, end - start);
}


Eigen::Vector3d util_rd::homo_to_v3(const Eigen::Vector4d homo) {
	assert(abs(homo(3)) > MYEPSILON);
	return { homo(0) / homo(3), homo(1) / homo(3) , homo(2) / homo(3) };
}

std::tuple<double, double, double> util_rd::compute_barycentric_2D(double x, double y, std::vector<Eigen::Vector2d> v){
	double c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
	double c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
	double c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
	return { c1,c2,c3 };
}

//Eigen::Vector3d util_rd::v3d_to_v4d(const Eigen::Vector3d& v, double w){
//	return Eigen::Vector4d(v.x(), v.y(), v.z(), w);
//}


