#pragma once
#ifndef UTILRD_H
#define UTILRD_H

#include <fstream>
#include <string>

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


	bool is_file_exists_ifstream(const std::string name);
	std::string find_file_name(const std::string file_path);
}


#endif

