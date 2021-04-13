#pragma once
#ifndef UTILRD_H
#define UTILRD_H

#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "global.h"

namespace util_rd {
	template <typename T>
	int sign_int(T x, T y) {
		// (x - y) < 0 ?
		if (x == y)     return 0;
		else if (x > y) return -1;
		else            return 1;
	};

	template <typename V>
	V linear_interpolate(double x, double x0, V v0, double x1, V v1) {
		double d_1_0 = x1 - x0;
		if (abs(d_1_0) < MYEPSILON) {
			return  (v0 + v1) / 2;
		}
		return v0 + (v1 - v0) * ((x - x0) / d_1_0);
	};


	template <typename T>
	std::pair<T, T> get_range_of_three(const T a, const T b, const T c) {
		T min_three, max_three;
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
	};


	template <typename T>
	T clip(const T& n, const T& lower, const T& upper) {
		return std::max(lower, std::min(n, upper));
	};
	

	bool is_file_exists_ifstream(const std::string name);
	std::string find_file_name(const std::string file_path);
	Eigen::Vector2d homo_to_v2(const Eigen::Vector4d homo);
	Eigen::Vector3d homo_to_v3(const Eigen::Vector4d homo);
	std::tuple<double, double, double> compute_barycentric_2D(double x, double y, std::vector<Eigen::Vector2d> v);
	//Eigen::Vector3d v3d_to_v4d(const Eigen::Vector3d& v, double w = 1.0f);
}


#endif

