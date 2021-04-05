#include <iostream>
//#include <cmath>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <Eigen\Dense>

#include "mesh.h"
#include "util.hpp"
#include "global.h"

using namespace std;
using namespace cv;
using namespace Eigen;




void simple_shade(TriangleMesh& mesh) {
	assert(mesh.obj_name == "triangle");
	mesh.vertices[0].set_color({ 255, 0, 0 });
	mesh.vertices[1].set_color({ 0, 255, 0 });
	mesh.vertices[2].set_color({ 0, 0, 255 });
}

void render_triangle(TriangleMesh& mesh, vector<Vector3d>& canvas) {
	for (auto iter = mesh.triangles.begin(); iter != mesh.triangles.end(); ++iter) {
		auto a = mesh.vertices[iter->v[0]].get_position();
		auto b = mesh.vertices[iter->v[1]].get_position();
		auto c = mesh.vertices[iter->v[2]].get_position();

		auto c_a = mesh.vertices[iter->v[0]].get_color();
		auto c_b = mesh.vertices[iter->v[1]].get_color();
		auto c_c = mesh.vertices[iter->v[2]].get_color();

		auto n = (b - a).cross(c - a);
		auto n_2 = n.squaredNorm();

		int z = 0;
		for (unsigned int i = 90; i < 410; ++i) {
			for (unsigned int j = 90; j < 410; ++j) {
				unsigned int k = 0;
				Vector3d p(i, j, k);

				auto n_a = (c - b).cross(p - b);
				auto n_b = (a - c).cross(p - c);
				auto n_c = (b - a).cross(p - a);

				double alpha = n.dot(n_a) / n_2;
				double beta = n.dot(n_b) / n_2;
				//double gamma = 1.0 - alpha - beta;
				double gamma = n.dot(n_c) / n_2;
				if (alpha > 0 && beta > 0  && gamma > 0) {
					auto color = alpha * c_a + beta * c_b + gamma * c_c;
					set_pixel(canvas, p(0), p(1), color);
				}

			}
		}

	}
}

void test_triangle_rasterization(vector<Vector3d>& canvas) {
	TriangleMesh mesh;
	string file_name = "triangle.obj";
	read_mesh_from_obj_file(mesh, file_name);
	simple_shade(mesh);
	render_triangle(mesh, canvas);
}




// Eigen::Matrix4d set_viewport_matirx(double n_x, double n_y) {
// 	Eigen::Matrix4d M_vp;
// 	M_vp << n_x / 2, 0, 0, (n_x - 1) / 2,
// 		0, n_y / 2, 0, (n_y - 1) / 2,
// 		0, 0, 1, 0,
// 		0, 0, 0, 1;
//
// 	return M_vp;
// }
//
//
// struct orthographic_view_volume {
// 	double l;
// 	double r;
// 	double b;
// 	double t;
// 	double n;
// 	double f;
// };
//
//
// Eigen::Matrix4d set_orthographic_projection_matrix(double l, double r, double b, double t, double f, double n) {
// 	Eigen::Matrix4d M_orth;
// 	M_orth << 2 / (r - l), 0, 0, -(r + l) / (r - l),
// 		0, 2 / (t - b), 0, -(t + b) / (t - b),
// 		0, 0, 2 / (n - f), -(n + f) / (n - f),
// 		0, 0, 0, 1;
//
// 	return M_orth;
// }
//
//
// Eigen::Matrix4d set_perspective_projection_matrix(double l, double r, double b, double t, double f, double n) {
// 	Eigen::Matrix4d M_per;
// 	M_per << 2 * n / (r - l), 0, (l + r) / (l - r), 0,
// 		0, 2 * n / (t - b), (b + t) / (b - t), 0,
// 		0, 0, (f + n) / (n - f), 2 * f * n / (f - n),
// 		0, 0, 1, 0;
// 	return M_per;
// }
//
//
// Eigen::Matrix4d set_camera_tranformation_matrix(Eigen::Vector3d e, Eigen::Vector3d g, Eigen::Vector3d t) {
// 	// param: e - eye position
// 	// param: g - gaze direction
// 	// param: t - view-up vector
//
// 	Eigen::Vector3d w = -g.normalized();
// 	Eigen::Vector3d u = (t.cross(w)).normalized();
// 	Eigen::Vector3d v = w.cross(u);
//
// 	cout << "w : " << endl << w << endl;
// 	cout << "u : " << endl << w << endl;
// 	cout << "v : " << endl << w << endl;
//
// 	Eigen::Matrix4d M_e_t;
// 	M_e_t << 1, 0, 0, -e(0),
// 			 0, 1, 0, -e(1),
// 		     0, 0, 1, -e(2),
// 		     0, 0, 0, 1;
//
// 	Eigen::Matrix4d M_uvw;
// 	M_uvw << u(0), u(1), u(2), 0,
// 		     v(0), v(1), v(2), 0,
// 		     w(0), w(1), w(2), 0,
// 		     0, 0, 0, 1;
//
// 	Eigen::Matrix4d M_cam = M_uvw * M_e_t;
//
// 	return M_cam;
// }

// Vector2d homo_to_v2(const Vector4d homo) {
// 	assert(abs(homo(3)) > EPSON);
// 	return { homo(0) / homo(3), homo(1) / homo(3) };
// }

void test_matrix(vector<Vector3d> &canvas) {
	double n_x = WINDOW_WIDTH;
	double n_y = WINDOW_HEIGHT;
	auto M_vp = set_viewport_matirx(n_x, n_y);

	// perspective
	double theta = 45;
	double n = -0.1, f = -50;
	double t = abs(n) * tan(theta / 2 / 180 * PI);
	double b = -t;
	double r = (n_x / n_y) * t;
	double l = -r;
	cout << "show plane: " << "n = " << n << " t = " << t << " r = " << r << endl;

	//auto M_orth = set_orthographic_projection_matrix(l, r, b, t, f, n);
	auto M_per = set_perspective_projection_matrix(l, r, b, t, f, n);
	Eigen::Vector3d eye_point = { 0, 0, 5 };
	Eigen::Vector3d gaze = { 0, 0, -1 };
	Eigen::Vector3d view_up = { -1, 0, 0 };
	auto M_cam = set_camera_tranformation_matrix(eye_point, gaze, view_up);

	Eigen::Matrix4d M = M_vp * M_per * M_cam;
	cout << " M_vp : " << endl;
	cout << M_vp << endl;

	cout << " M_per : " << endl;
	cout << M_per << endl;

	cout << " M_cam : " << endl;
	cout << M_cam << endl;

	cout << " M : " << endl;
	cout << M << endl;
	Vector3d v0 = { 2.0, 0.0, -2. };
	Vector3d v1 = { 0.0, 2.0, -2. };
	Vector3d v2 = { -2.0, 0.0, -2. };
	auto p_a = homo_to_v2(M * v0.homogeneous());
	auto p_b = homo_to_v2(M * v1.homogeneous());
	auto p_c = homo_to_v2(M * v2.homogeneous());
	cout << "a " << p_a(0) << " " << p_a(1) << endl;
	cout << "b " << p_b(0) << " " << p_b(1) << endl;
	cout << "c " << p_c(0) << " " << p_c(1) << endl;
	draw_line(canvas, p_a(0), p_a(1), p_b(0), p_b(1), {0, 0, 255});
	draw_line(canvas, p_b(0), p_b(1), p_c(0), p_c(1), { 0, 0, 255 });
	draw_line(canvas, p_c(0), p_c(1), p_a(0), p_a(1), { 0, 0, 255 });
}





//
// int implicit_line_equation(int x, int y, int x0, int y0, int x1, int y1) {
// 	int result = (y0 - y1) * x + (x1 - x0) * y + x0 * y1 - x1 * y0;
// 	cout << "result: " << result << endl;
// 	return result;
// }
//
//
// void draw_line_mid_point(vector<Vector3d>& canvas, int x0, int y0, int x1, int y1, Vector3d color) {
// 	// mid point algorithm
// 	assert(canvas.size() == WINDOW_WIDTH * WINDOW_HEIGHT && x1 >= 0 && x1 < WINDOW_WIDTH&& y1 >= 0 && y1 < WINDOW_HEIGHT
// 		&& x0 >= 0 && x0 < WINDOW_WIDTH&& y0 >= 0 && y0 < WINDOW_HEIGHT);
//
// 	if (x1 == x0) {
// 		if (y0 < y1) for (int y = y0; y <= y1; ++y) set_pixel(canvas, x0, y, color);
// 		else for (int y = y0; y >= y1; --y) set_pixel(canvas, x0, y, color);
// 		return;
// 	}
//
// 	if (x1 < x0) {
// 		swap(x0, x1);
// 		swap(y0, y1);
// 	}
// 	int d_y = x1 - x0, d_x = y0 - y1;
// 	double m = (static_cast<double>(y1) - y0) / (static_cast<double>(x1) - x0);
// 	if (m <= 1 && m >= 0) {
// 		int y = y0;
// 		int d = implicit_line_equation(x0 + 1, y0 + 0.5, x0, y0, x1, y1);
// 		for (int x = x0; x <= x1 && y <= y1; ++x) {
// 			set_pixel(canvas, x, y, color);
// 			if (d < 0) {
// 				++y;
// 				d += d_x + d_y;
// 			}
// 			else d += d_x;
// 		}
// 	}
// 	else if (m >= -1 && m < 0) {
// 		int y = y0;
// 		int d = implicit_line_equation(x0 + 1, y0 - 0.5, x0, y0, x1, y1);
// 		for (int x = x0; x <= x1 && y >= y1; ++x) {
// 			set_pixel(canvas, x, y, color);
// 			if (d > 0) {
// 				--y;
// 				d += d_x - d_y;
// 			}
// 			else d += d_x;
// 		}
// 	}
// 	else if (m > 1) {
// 		int x = x0;
// 		int d = implicit_line_equation(x0 + 0.5, y0 + 1, x0, y0, x1, y1);
// 		for (int y = y0; y <= y1; ++y) {
// 			set_pixel(canvas, x, y, color);
// 			if (d > 0) {
// 				++x;
// 				d += d_x + d_y;
// 			}
// 			else d += d_y;
//
// 		}
// 	}
// 	else {
// 		int x = x0;
// 		int d = implicit_line_equation(x0 + 0.5, y0 - 1, x0, y0, x1, y1);
// 		for (int y = y0; y >= y1; --y) {
// 			set_pixel(canvas, x, y, color);
// 			if (d < 0) {
// 				++x;
// 				d += d_x - d_y;
// 			}
// 			else d -= d_y;
// 		}
// 	}
//
// }
