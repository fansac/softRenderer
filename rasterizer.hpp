#ifndef RASTERIZER_H
#define RASTERIZER_H

#include <utility>
#include <vector>
#include <iostream>
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
		Eigen::Matrix4d get_M();

		void set_view_volume(double theta, double near, double far);
		void set_camera(Eigen::Vector3d eye, Eigen::Vector3d gaze, Eigen::Vector3d view_up);

		void draw_pixel(Pixel p);
		void draw_line(Pixel p0, Pixel p1);
		void draw_triangle(Pixel p0, Pixel p1, Pixel p2);
		void calculate_matrix();

	private:
		size_t w;
		size_t h;
		std::vector<Eigen::Vector3d>canvas;
		double left, right, bottom, top, near, far, fov;
		Eigen::Vector3d eye, gaze, up;
		Eigen::Matrix4d M_trans;

		Eigen::Matrix4d set_viewport_matirx(double n_x, double n_y);
		Eigen::Matrix4d set_orthographic_projection_matrix(double l, double r, double b, double t, double f, double n);
		Eigen::Matrix4d set_perspective_projection_matrix(double l, double r, double b, double t, double f, double n);
		Eigen::Matrix4d set_camera_tranformation_matrix(Eigen::Vector3d e, Eigen::Vector3d g, Eigen::Vector3d t);
	};
}

#endif // !RASTERIZER_H

