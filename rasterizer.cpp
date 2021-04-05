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

Eigen::Matrix4d rst::Rasterizer::get_M() {
	return this->M_trans;
}


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

void rst::Rasterizer::draw_triangle(Pixel p0, Pixel p1, Pixel p2) {
	auto x_range = util_rd::get_range_of_three(p0.x, p1.x, p2.x);
	auto y_range = util_rd::get_range_of_three(p0.y, p1.y, p2.y);

	for (auto x = x_range.first; x <= x_range.second; ++x) {
		for (auto y = y_range.first; y <= y_range.second; ++y) {
			auto barycentric_coordinates = util_rd::compute_barycentric_2D(x, y, { {p0.x, p0.y}, {p1.x, p1.y},{p2.x, p2.y} });
			auto alpha = std::get<0>(barycentric_coordinates);
			auto beta = std::get<1>(barycentric_coordinates);
			auto gamma = std::get<2>(barycentric_coordinates);
			if (alpha > 0 && beta > 0 && gamma > 0) {
				auto c = alpha * p0.c + beta * p1.c + gamma * p2.c;
				draw_pixel({ x, y, c });
			}

		}
	}

}

Eigen::Matrix4d rst::Rasterizer::set_viewport_matirx(double n_x, double n_y) {
	Eigen::Matrix4d M_vp;
	M_vp << n_x / 2, 0, 0, (n_x - 1) / 2,
		0, n_y / 2, 0, (n_y - 1) / 2,
		0, 0, 1, 0,
		0, 0, 0, 1;
	return M_vp;
}


Eigen::Matrix4d rst::Rasterizer::set_orthographic_projection_matrix(double l, double r, double b, double t, double f, double n) {
	Eigen::Matrix4d M_orth;
	M_orth << 2 / (r - l), 0, 0, -(r + l) / (r - l),
		0, 2 / (t - b), 0, -(t + b) / (t - b),
		0, 0, 2 / (n - f), -(n + f) / (n - f),
		0, 0, 0, 1;
	return M_orth;
}


Eigen::Matrix4d rst::Rasterizer::set_perspective_projection_matrix(double l, double r, double b, double t, double f, double n) {
	Eigen::Matrix4d M_per;
	M_per << 2 * n / (r - l), 0, (l + r) / (l - r), 0,
		0, 2 * n / (t - b), (b + t) / (b - t), 0,
		0, 0, (f + n) / (n - f), 2 * f * n / (f - n),
		0, 0, 1, 0;
	return M_per;
}


Eigen::Matrix4d rst::Rasterizer::set_camera_tranformation_matrix(Eigen::Vector3d e, Eigen::Vector3d g, Eigen::Vector3d t) {
	// param: e - eye position
	// param: g - gaze direction
	// param: t - view-up vector

	Eigen::Vector3d w = -g.normalized();
	Eigen::Vector3d u = (t.cross(w)).normalized();
	Eigen::Vector3d v = w.cross(u);

	Eigen::Matrix4d M_e_t;
	M_e_t << 1, 0, 0, -e(0),
		0, 1, 0, -e(1),
		0, 0, 1, -e(2),
		0, 0, 0, 1;

	Eigen::Matrix4d M_uvw;
	M_uvw << u(0), u(1), u(2), 0,
		v(0), v(1), v(2), 0,
		w(0), w(1), w(2), 0,
		0, 0, 0, 1;

	Eigen::Matrix4d M_cam = M_uvw * M_e_t;

	return M_cam;
}

void rst::Rasterizer::set_view_volume(double theta, double near, double far) {
	this->top = abs(near) * tan(theta / 360 * MYPI);
	this->bottom = -top;

	double n_x = this->w;
	double n_y = this->h; 
	this->right = (n_x / n_y) * this->top;
	this->left = -right;
	
	this->near = near;
	this->far = far;
}

void rst::Rasterizer::set_camera(Eigen::Vector3d eye, Eigen::Vector3d gaze, Eigen::Vector3d view_up) {
	this->eye = eye;
	this->gaze = gaze;
	this->up = view_up;
}

void rst::Rasterizer::calculate_matrix() {
	double n_x = this->w;
	double n_y = this->h;
	auto M_vp = set_viewport_matirx(n_x, n_y);

	//auto M_orth = set_orthographic_projection_matrix(l, r, b, t, f, n);
	auto M_per = set_perspective_projection_matrix(left, right, bottom, top, far, near);
	auto M_cam = set_camera_tranformation_matrix(eye, gaze, up);

	this->M_trans = M_vp * M_per * M_cam;
}



