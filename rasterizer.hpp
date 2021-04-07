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
		Pixel() = default;
		Pixel(size_t pos_x, size_t pos_y, Eigen::Vector3d color) : x(pos_x), y(pos_y), c(color) {};
		Pixel(size_t pos_x, size_t pos_y, size_t pos_z, Eigen::Vector3d color) : x(pos_x), y(pos_y), z(pos_z), c(color) {};
		Pixel(size_t pos_x, size_t pos_y, size_t pos_z) : x(pos_x), y(pos_y), z(pos_z) {};
		size_t x = 0;
		size_t y = 0;
		size_t z = 0;
		Eigen::Vector3d c;
	};
	

	class Rasterizer {
	public:
		Rasterizer() = delete;
		Rasterizer(size_t width, size_t height);

		int test_index = 0;

		size_t w;
		size_t h;
		double left, right, bottom, top, near, far, fov;
		Eigen::Vector3d eye, gaze, up;
		Eigen::Matrix4d m_vp, m_orth, m_per, m_cam, m_trans, m_model, mvp;
		std::vector<Eigen::Vector3d>canvas;
		std::vector<uint16_t>z_buffer;
		

		//void set_screen_size(size_t width, size_t height);
		std::pair<size_t, size_t> get_screen_size();
		std::vector<Eigen::Vector3d> get_canvas();
		std::vector<Eigen::Vector3d> canvas_2_screen();
		Eigen::Matrix4d get_M();

		void set_view_volume(double theta, double near, double far);
		void set_camera(Eigen::Vector3d eye, Eigen::Vector3d gaze, Eigen::Vector3d view_up);

		void draw_pixel(Pixel p);
		void draw_line(Pixel p0, Pixel p1);
		void draw_triangle(const std::vector<Pixel> pixels);
		void draw_triangle(const std::vector<Pixel> pixels, const std::vector<Eigen::Vector3d> normals);
		void calculate_matrix();
		bool compare_pixel_in_z_buffer(size_t x, size_t y, uint32_t z);

		uint32_t to_z_buffer_value(double z);

		
		void set_viewport_matirx();
		void set_orthographic_projection_matrix();
		void set_perspective_projection_matrix();
		void set_camera_tranformation_matrix();
		void set_model_tranformation_matrix();
	};
}

#endif // !RASTERIZER_H