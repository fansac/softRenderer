#include "rasterizer.hpp"


bool rst::Triangle2D::is_inside(Point2D p) {
	double cross_a = (p.x - a.x) * (b.y - a.y) - (b.x - a.x) * (p.y - a.y);
	double cross_b = (p.x - b.x) * (c.y - b.y) - (c.x - b.x) * (p.y - b.y);
	double cross_c = (p.x - c.x) * (a.y - c.y) - (a.x - c.x) * (p.y - c.y);

	return (cross_a > 0) && (cross_b > 0) && (cross_c > 0)
		|| (cross_a < 0) && (cross_b < 0) && (cross_c < 0);
}

rst::Rasterizer::Rasterizer(size_t width, size_t height) : w(width), h(height), n_x(width), n_y(height) {
	assert(n_x <= 1920 && n_y <= 1080);
	canvas = std::vector<Eigen::Vector3d>(n_x * n_y, Eigen::Vector3d(0, 0, 0));
	z_buffer = std::vector<uint16_t>(n_x * n_y, UINT16_MAX);
}

//void rst::Rasterizer::set_screen_size(size_t width, size_t height) {
//	this->w = width;
//	this->h = height;
//};

uint16_t rst::Rasterizer::to_z_buffer_value(double z) {
	double z_value = ((z - this->near) / (this->far - this->near) * static_cast<double>(UINT16_MAX));
	assert(z_value > 0);
	return z_value;
}

bool rst::Rasterizer::compare_pixel_in_z_buffer(size_t x, size_t y, uint16_t z) {
	bool is_near = false;
	if (z <= z_buffer[y * n_x + x]) {
		is_near = true;
		z_buffer[y * n_x + x] = z;
	}
	return is_near;
}

std::pair<size_t, size_t> rst::Rasterizer::get_screen_size() {
	return std::make_pair(n_x, n_y);
}

std::vector<Eigen::Vector3d> rst::Rasterizer::get_canvas() { return canvas; }


Eigen::Matrix4d rst::Rasterizer::get_M() {
	return this->mvp;
}


std::vector<Eigen::Vector3d> rst::Rasterizer::canvas_2_screen() {
	std::vector<Eigen::Vector3d> screen(n_x * n_y, Eigen::Vector3d(0, 0, 0));
	for (size_t y = 0; y < n_y; ++y) {
		for (size_t x = 0; x < n_x; ++x) {
			screen[(n_y - 1 - y) * n_x + x] = canvas[y * n_x + x];
		}
	}
	return screen;
}




void rst::Rasterizer::draw_pixel(Pixel p) {
	assert(canvas.size() == n_x * n_y && p.x < n_x && p.y < n_y);
	assert(p.y * n_x + p.x < n_x * n_y);
	canvas[p.y * n_x + p.x] = p.c;
}

void rst::Rasterizer::draw_line(rst::Pixel p0, rst::Pixel p1) {
	// Bresenham algorithm https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	assert(canvas.size() == n_x * n_y && p0.x < n_x && p0.y < n_y && p1.x < n_x && p1.y < n_y);
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

	auto x_min = util_rd::clip(static_cast<size_t>(x_range.first - 0.5), static_cast<size_t>(0), this->n_x);
	auto x_max = util_rd::clip(static_cast<size_t>(x_range.second + 0.5), static_cast<size_t>(0), this->n_x);
	auto y_min = util_rd::clip(static_cast<size_t>(y_range.first - 0.5), static_cast<size_t>(0), this->n_y);
	auto y_max = util_rd::clip(static_cast<size_t>(y_range.second + 0.5), static_cast<size_t>(0), this->n_y);
	Triangle2D tri({ static_cast<double>(p0.x), static_cast<double>(p0.y) }, { static_cast<double>(p1.x), static_cast<double>(p1.y) }
				, { static_cast<double>(p2.x), static_cast<double>(p2.y) });
	for (auto x = x_min; x <= x_max; ++x) {
		for (auto y = y_min; y <= y_max; ++y) {
			if (tri.is_inside({ static_cast<double>(x), static_cast<double>(y) })) {
				auto barycentric_coordinates = util_rd::compute_barycentric_2D(x, y, { {p0.x, p0.y}, {p1.x, p1.y},{p2.x, p2.y} });
				auto alpha = std::get<0>(barycentric_coordinates);
				auto beta = std::get<1>(barycentric_coordinates);
				auto gamma = std::get<2>(barycentric_coordinates);
				auto c = alpha * p0.c + beta * p1.c + gamma * p2.c;
				draw_pixel({ x, y, c });
			}
		}
	}
}


void rst::Rasterizer::set_model_transformation(double angle, double scale) {
	this->angle = angle;
	this->scale = scale;
}

void rst::Rasterizer::set_view_volume(double theta, double near, double far) {
	this->top = abs(near) * tan(theta / 360 * MYPI);
	this->bottom = -this->top;

	this->right = this->w / this->h * this->top;
	this->left = -right;

	this->near = near;
	this->far = far;
}

void rst::Rasterizer::set_camera(Eigen::Vector3d eye, Eigen::Vector3d gaze, Eigen::Vector3d view_up) {
	this->eye = eye;
	this->gaze = gaze;
	this->up = view_up;
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

	m_uvw = M_uvw.block(0,0,3,3);
	m_cam = M_uvw * M_e_t;
}



void rst::Rasterizer::set_model_tranformation_matrix() {
	
	Eigen::Matrix4d rotation;
	angle = angle * MYPI / 180.0;
	rotation << cos(angle), 0, sin(angle), 0,
		0, 1, 0, 0,
		-sin(angle), 0, cos(angle), 0,
		0, 0, 0, 1;

	Eigen::Matrix4d scale;
	scale << this->scale, 0, 0, 0,
		0, this->scale, 0, 0,
		0, 0, this->scale, 0,
		0, 0, 0, 1;
	
	Eigen::Matrix4d translate = Eigen::Matrix4d::Identity();

	m_model = translate * rotation * scale;
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



