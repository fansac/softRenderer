#ifndef RASTERIZER_H
#define RASTERIZER_H

#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "util.hpp"
#include "global.h"

namespace rst {

	class Pixel {
	public:
		Pixel(size_t x, size_t y, Eigen::Vector3d color) {
			this->x = x;
			this->y = y;
			c = color;
		}
		size_t x;
		size_t y;
		Eigen::Vector3d c;
	};
	

	class Rasterizer {
	public:
		Rasterizer() = delete;
		Rasterizer(size_t width, size_t height);

		//void set_screen_size(size_t width, size_t height);
		std::pair<size_t, size_t> get_screen_size();
		std::vector<Eigen::Vector3d> get_canvas();
		std::vector<Eigen::Vector3d> canvas_2_screen();
		void draw_pixel(Pixel p);
		void draw_line(Pixel p0, Pixel p1);

	private:
		size_t w;
		size_t h;
		std::vector<Eigen::Vector3d>canvas;
	};
}

#endif // !RASTERIZER_H

