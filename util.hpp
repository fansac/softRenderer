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
		if (abs(d_1_0) < MYEPSON) {
			return  (v0 + v1) / 2;
		}
		return v0 + (v1 - v0) * ((x - x0) / d_1_0);
	};

	template <typename T>
	T clip(const T& n, const T& lower, const T& upper) {
		return std::max(lower, std::min(n, upper));
	}

	bool is_file_exists_ifstream(const std::string name);
	std::string find_file_name(const std::string file_path);
	Eigen::Vector2d homo_to_v2(const Eigen::Vector4d homo);
	Eigen::Vector3d homo_to_v3(const Eigen::Vector4d homo);
	std::pair<size_t, size_t> get_range_of_three(const size_t a, const size_t b, const size_t c);
	std::tuple<double, double, double> compute_barycentric_2D(double x, double y, std::vector<Eigen::Vector2d> v);
	
}


#endif

