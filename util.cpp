#include "util.hpp"

namespace util_rd {
	bool is_file_exists_ifstream(const std::string name) {
		std::ifstream f(name.c_str());
		return f.good();
	}

	std::string find_file_name(const std::string file_path) {
		size_t start = file_path.find_last_of('/') + 1;
		size_t end = file_path.find_last_of('.');
		return file_path.substr(start, end);
	}

	Eigen::Vector2d homo_to_v2(const Eigen::Vector4d homo) {
		assert(abs(homo(3)) > MYEPSON);
		return { homo(0) / homo(3), homo(1) / homo(3) };
	}

	std::tuple<double, double, double> compute_barycentric_2D(double x, double y, std::vector<Eigen::Vector2d> v){
		double c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
		double c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
		double c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
		return { c1,c2,c3 };
	}


	std::pair<size_t, size_t> get_range_of_three(const size_t a, const size_t b, const size_t c) {
		size_t min_three, max_three;
		if (a <= b) {
			min_three = a, max_three = b;
		}
		else {
			min_three = b, max_three = a;
		}
		if (c < min_three) {
			min_three = c;
		}
		else if (c > max_three) {
			max_three = c;
		}
		return { min_three, max_three };
	}
}