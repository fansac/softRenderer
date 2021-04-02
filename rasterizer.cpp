#include "rasterizer.hpp"

rst::Rasterizer::Rasterizer(size_t width, size_t height) : w(width), h(height) {
	assert(w <= 1920 && h <= 1080);
	canvas = std::vector<Eigen::Vector3d>(w * h, Eigen::Vector3d(0, 0, 0));
}

//void rst::Rasterizer::set_screen_size(size_t width, size_t height) {
//	this->w = width;
//	this->h = height;
//};

std::pair<size_t, size_t> rst::Rasterizer::get_screen_size() {
	return std::make_pair(w, h);
}

std::vector<Eigen::Vector3d> rst::Rasterizer::get_canvas() { return canvas; }


std::vector<Eigen::Vector3d> rst::Rasterizer::canvas_2_screen() {
	std::vector<Eigen::Vector3d> screen(w * h, Eigen::Vector3d(0, 0, 0));
	for (size_t y = 0; y < h; ++y) {
		for (size_t x = 0; x < w; ++x) {
			screen[(h - 1 - y) * w + x] = canvas[y * w + x];
		}
	}
	return screen;
}

void rst::Rasterizer::draw_pixel(Pixel p) {
	assert(canvas.size() == w * h && p.x < w && p.y < h);
	assert(p.y * w + p.x < w * h);
	canvas[p.y * w + p.x] = p.c;
}

void rst::Rasterizer::draw_line(rst::Pixel p0, rst::Pixel p1) {
	// Bresenham algorithm https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	assert(canvas.size() == w * h && p0.x < w && p0.y < h && p1.x < w && p1.y < h);
	int sign_x = util_rd::sign_int(p0.x, p1.x);
	int sign_y = util_rd::sign_int(p0.y, p1.y);

	if (p0.x == p1.x) {
		size_t x = p0.x;
		for (auto y = p0.y; y != p1.y; y += sign_y) draw_pixel({ x, y, util_rd::linear_interpolate(y, p0.y, p0.c, p1.y, p1.c) });
		draw_pixel({ p1.x, p1.y, p1.c });
		return;
	}
	if (p0.y == p1.y) {
		size_t y = p0.y;
		for (auto x = p0.x; x != p1.x; x += sign_x) draw_pixel({ x, y, util_rd::linear_interpolate(x, p0.x, p0.c, p1.x, p1.c)});
		draw_pixel({p1.x, p1.y, p1.c});
		return;
	}

	int d_x = sign_x * (p1.x - p0.x);
	int d_y = sign_y * (p0.y - p1.y);
	int err = d_x + d_y;
	auto x = p0.x, y = p0.y;
	do {
		draw_pixel({ x, y, util_rd::linear_interpolate(y, p0.y, p0.c, p1.y, p1.c) });
		int e2 = 2 * err;
		if (e2 >= d_y) {
			err += d_y;
			x += sign_x;
		}
		if (e2 <= d_x) {
			err += d_x;
			y += sign_y;
		}
	} while (!(x == p1.x && y == p1.y));
}