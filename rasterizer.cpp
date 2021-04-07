#include "rasterizer.hpp"



rst::Rasterizer::Rasterizer(size_t width, size_t height) : w(width), h(height) {
	assert(w <= 1920 && h <= 1080);
	canvas = std::vector<Eigen::Vector3d>(w * h, Eigen::Vector3d(0, 0, 0));
	z_buffer = std::vector<uint16_t>(w * h, UINT16_MAX);
}

//void rst::Rasterizer::set_screen_size(size_t width, size_t height) {
//	this->w = width;
//	this->h = height;
//};

uint32_t rst::Rasterizer::to_z_buffer_value(double z) {
	double z_value = ((z - this->near) / (this->far - this->near) * static_cast<double>(UINT32_MAX));
	assert(z_value > 0);
	return z_value;
}

bool rst::Rasterizer::compare_pixel_in_z_buffer(size_t x, size_t y, uint32_t z) {
	bool is_near = false;
	if (z <= z_buffer[y * w + x]) {
		is_near = true;
		z_buffer[y * w + x] = z;
	}
	return is_near;
}

std::pair<size_t, size_t> rst::Rasterizer::get_screen_size() {
	return std::make_pair(w, h);
}

std::vector<Eigen::Vector3d> rst::Rasterizer::get_canvas() { return canvas; }


Eigen::Matrix4d rst::Rasterizer::get_M() {
	return this->mvp;
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

void rst::Rasterizer::draw_triangle(const std::vector<Pixel> pixels) {
	auto& p0 = pixels[0];
	auto& p1 = pixels[1];
	auto& p2 = pixels[2];

	auto x_range = util_rd::get_range_of_three(p0.x, p1.x, p2.x);
	auto y_range = util_rd::get_range_of_three(p0.y, p1.y, p2.y);

	auto x_min = util_rd::clip(x_range.first, static_cast<size_t>(0), this->w);
	auto x_max = util_rd::clip(x_range.second, static_cast<size_t>(0), this->w);
	auto y_min = util_rd::clip(y_range.first, static_cast<size_t>(0), this->h);
	auto y_max = util_rd::clip(y_range.second, static_cast<size_t>(0), this->h);
	for (auto x = x_min; x <= x_max; ++x) {
		for (auto y = y_min; y <= y_max; ++y) {
			auto barycentric_coordinates = util_rd::compute_barycentric_2D(x, y, { {p0.x, p0.y}, {p1.x, p1.y},{p2.x, p2.y} });
			auto alpha = std::get<0>(barycentric_coordinates);
			auto beta = std::get<1>(barycentric_coordinates);
			auto gamma = std::get<2>(barycentric_coordinates);
			if (alpha >= 0 && beta >= 0 && gamma >= 0) {
				uint16_t z = (alpha * p0.z + beta * p1.z + gamma * p2.z);
				if (compare_pixel_in_z_buffer(x, y, z))
				{
					auto c = alpha * p0.c + beta * p1.c + gamma * p2.c;
					draw_pixel({ x, y, c });
				}
			}

		}
	}
}

void rst::Rasterizer::draw_triangle(const std::vector<Pixel> pixels, const std::vector<Eigen::Vector3d> normals) {
	auto& p0 = pixels[0];
	auto& p1 = pixels[1];
	auto& p2 = pixels[2];

	auto x_range = util_rd::get_range_of_three(p0.x, p1.x, p2.x);
	auto y_range = util_rd::get_range_of_three(p0.y, p1.y, p2.y);

	auto x_min = util_rd::clip(x_range.first, static_cast<size_t>(0), this->w);
	auto x_max = util_rd::clip(x_range.second, static_cast<size_t>(0), this->w);
	auto y_min = util_rd::clip(y_range.first, static_cast<size_t>(0), this->h);
	auto y_max = util_rd::clip(y_range.second, static_cast<size_t>(0), this->h);
	for (auto x = x_min; x <= x_max; ++x) {
		for (auto y = y_min; y <= y_max; ++y) {
			auto barycentric_coordinates = util_rd::compute_barycentric_2D(x, y, { {p0.x, p0.y}, {p1.x, p1.y},{p2.x, p2.y} });
			auto alpha = std::get<0>(barycentric_coordinates);
			auto beta = std::get<1>(barycentric_coordinates);
			auto gamma = std::get<2>(barycentric_coordinates);
			if (alpha >= 0 && beta >= 0 && gamma >= 0) {
				uint16_t z = (alpha * p0.z + beta * p1.z + gamma * p2.z);
				if (compare_pixel_in_z_buffer(x, y, z))
				{
					auto c = alpha * p0.c + beta * p1.c + gamma * p2.c;
					auto n = alpha * normals[0] + beta * normals[1] + gamma * normals[2];
					draw_pixel({ x, y, c });
				}
			}

		}
	}
}

void rst::Rasterizer::set_viewport_matirx() {
	m_vp << w / 2, 0, 0, (w - 1) / 2,
		0, h / 2, 0, (h - 1) / 2,
		0, 0, 1, 0,
		0, 0, 0, 1;
}


void rst::Rasterizer::set_orthographic_projection_matrix() {
	m_orth << 2 / (right - left), 0, 0, -(right + left) / (right - left),
		0, 2 / (top - bottom), 0, -(top + bottom) / (top - bottom),
		0, 0, 2 / (near - far), -(near + far) / (near - far),
		0, 0, 0, 1;
}


void rst::Rasterizer::set_perspective_projection_matrix() {
	m_per << 2 * near / (right - left), 0, (left + right) / (left - right), 0,
		0, 2 * near / (top - bottom), (bottom + top) / (bottom - top), 0,
		0, 0, (far + near) / (near - far), 2 * far * near / (far - near),
		0, 0, 1, 0;
}


void rst::Rasterizer::set_camera_tranformation_matrix() {
	// param: e - eye position
	// param: g - gaze direction
	// param: t - view-up vector

	Eigen::Vector3d w = -gaze.normalized();
	Eigen::Vector3d u = (up.cross(w)).normalized();
	Eigen::Vector3d v = w.cross(u);

	Eigen::Matrix4d M_e_t;
	M_e_t << 1, 0, 0, -eye(0),
		0, 1, 0, -eye(1),
		0, 0, 1, -eye(2),
		0, 0, 0, 1;

	Eigen::Matrix4d M_uvw;
	M_uvw << u(0), u(1), u(2), 0,
		v(0), v(1), v(2), 0,
		w(0), w(1), w(2), 0,
		0, 0, 0, 1;

	m_cam = M_uvw * M_e_t;
}

void rst::Rasterizer::set_view_volume(double theta, double near, double far) {
	top = abs(near) * tan(theta / 360 * MYPI);
	bottom = -top;

	right = (w / h) * top;
	left = -right;
	
	near = near;
	far = far;
}

void rst::Rasterizer::set_camera(Eigen::Vector3d eye, Eigen::Vector3d gaze, Eigen::Vector3d view_up) {
	this->eye = eye;
	this->gaze = gaze;
	this->up = view_up;
}

void rst::Rasterizer::set_model_tranformation_matrix() {
	m_model = Eigen::Matrix4d::Identity();
}

void rst::Rasterizer::calculate_matrix() {

	set_viewport_matirx();

	set_orthographic_projection_matrix();
	set_perspective_projection_matrix();
	set_orthographic_projection_matrix();
	set_camera_tranformation_matrix();
	set_model_tranformation_matrix();
	mvp = m_vp * m_per * m_cam * m_model;
	
}



