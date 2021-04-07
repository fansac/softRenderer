#include "shading.hpp"

void gouraud_shading(TriangleMesh& mesh, rst::Rasterizer& r, const Eigen::Vector3d eye_point, const Eigen::Matrix4d M) {
	Eigen::Vector3d light = { -1, -1, -1 };
	Eigen::Vector3d c_l = { 1.0, 1.0, 1.0 };
	Eigen::Vector3d c_a = Eigen::Vector3d(10, 10, 10) / 255;
	Eigen::Vector3d c_p = { 0.7937, 0.7937, 0.7937 };
	auto direc_light = -(light.normalized());
	for (auto iter = mesh.triangles.begin(); iter != mesh.triangles.end(); ++iter) {
		std::vector<rst::Pixel> pixels;
		for (unsigned int i = 0; i < 3; ++i) {
			Eigen::Vector3d position_i = mesh.vertices[iter->v[i]].get_position();
			Eigen::Vector3d e = (eye_point - position_i).normalized();
			Eigen::Vector3d h = (e + direc_light).normalized();
			Eigen::Vector3d n_i = mesh.normals[iter->n[i]].normalized();
			Eigen::Vector3d c_r = mesh.colors[iter->c[i]] / 255.0;

			double cosine = util_rd::clip(n_i.dot(direc_light), 0.0, 1.0);

			Eigen::Vector3d pos_3_i = util_rd::homo_to_v3(M * position_i.homogeneous());
			double phong_coe = pow(max(h.dot(direc_light), 0.0), PHONGEXP);
			Eigen::Vector3d c = ((c_r.array() * (c_a + c_l * cosine).array()) + c_l.array() * c_p.array() * phong_coe).min(Eigen::Array3d(1, 1, 1)).max(Array3d(0, 0, 0));

			rst::Pixel pixel_i(pos_3_i[0], pos_3_i[1], r.to_z_buffer_value(pos_3_i[2]), c * 255.0);
			pixels.push_back(pixel_i);
		}
		r.draw_triangle(pixels);
	}
}

void phong_shading(TriangleMesh& mesh, rst::Rasterizer& r) {
	Eigen::Vector3d light = { -1, -1, -1 };
	Eigen::Vector3d c_l = { 1.0, 1.0, 1.0 };
	Eigen::Vector3d c_a = Eigen::Vector3d(10, 10, 10) / 255;
	Eigen::Vector3d c_p = { 0.7937, 0.7937, 0.7937 };
	auto direc_light = -(light.normalized());
	for (auto iter = mesh.triangles.begin(); iter != mesh.triangles.end(); ++iter) {
		Eigen::Vector3d pixels[3];
		Eigen::Vector3d normals[3];
		Eigen::Vector3d points_in_view[3];
		Eigen::Vector3d colors[3];
		auto f1 = (r.near - r.far) / 2;
		auto f2 = (r.near + r.far) / 2;
		auto m_eye = r.m_cam * r.m_model;
		for (unsigned int i = 0; i < 3; ++i) {
			Eigen::Vector3d position_i = mesh.vertices[iter->v[i]].get_position();
			// (pixel.x, pixel.y) in [WIDTH, HEIGHT]
			pixels[i] = util_rd::homo_to_v3(r.mvp * position_i.homogeneous());
			// conver z from [-1, 1] to [far, near];
			pixels[i].z() = f1 * pixels[i].z() + f2;
			// view/camera space
			normals[i] = util_rd::homo_to_v3(m_eye.inverse().transpose() * (mesh.vertices[iter->v[i]].get_normal().homogeneous()));
			points_in_view[i] = util_rd::homo_to_v3(m_eye * position_i.homogeneous());
			colors[i] = {0, 0, 255};
		}
		auto p0 = pixels[0];
		auto p1 = pixels[1];
		auto p2 = pixels[2];
		auto pv0 = points_in_view[0];
		auto pv1 = points_in_view[1];
		auto pv2 = points_in_view[2];
		auto x_range = util_rd::get_range_of_three(p0.x(), p1.x(), p2.x());
		auto y_range = util_rd::get_range_of_three(p0.y(), p1.y(), p2.y());

		auto x_min = util_rd::clip(x_range.first, static_cast<size_t>(0), r.w);
		auto x_max = util_rd::clip(x_range.second, static_cast<size_t>(0), r.w);
		auto y_min = util_rd::clip(y_range.first, static_cast<size_t>(0), r.h);
		auto y_max = util_rd::clip(y_range.second, static_cast<size_t>(0), r.h);

		for (auto x = x_min; x <= x_max; ++x) {
			for (auto y = y_min; y <= y_max; ++y) {
				auto barycentric_coordinates = util_rd::compute_barycentric_2D(x, y, { {p0.x(), p0.y()}, {p1.x(), p1.y()},{p2.x(), p2.y()} });
				auto alpha = std::get<0>(barycentric_coordinates);
				auto beta = std::get<1>(barycentric_coordinates);
				auto gamma = std::get<2>(barycentric_coordinates);
				if (alpha >= 0 && beta >= 0 && gamma >= 0) {
					double z_view = 1.0 / (alpha * pv0.z() + beta * pv1.z() + gamma * pv2.z());
					if (r.compare_pixel_in_z_buffer(x, y, z_view)){
						double x_view = (alpha * pv0.x() / pv0.z() + beta * pv1.x() / pv1.z() + gamma * pv2.x() / pv2.z()) * z_view;
						double y_view = (alpha * pv0.y() / pv0.z() + beta * pv1.y() / pv1.z() + gamma * pv2.y() / pv2.z()) * z_view;
						Eigen::Vector3d e = -Eigen::Vector3d(x_view, y_view, z_view).normalized();
						Eigen::Vector3d h = (e + direc_light).normalized();
						auto c_r = ((alpha * colors[0] / pv0.z() + beta * colors[1] / pv1.z() + gamma * colors[2] / pv2.z()) * z_view) / 255.0;
						auto n = ((alpha * normals[0] / pv0.z() + beta * normals[1] / pv1.z() + gamma * normals[2] / pv2.z()) * z_view).normalized();
						double cosine = util_rd::clip(n.dot(direc_light), 0.0, 1.0);
						double phong_coe = pow(max(h.dot(direc_light), 0.0), PHONGEXP);
						Eigen::Vector3d c = ((c_r.array() * (c_a + c_l * cosine).array()) + c_l.array() * c_p.array() * phong_coe).min(Eigen::Array3d(1, 1, 1)).max(Array3d(0, 0, 0));						
						r.draw_pixel({ x, y, c });
					}
				}

			}
		}
	}
}



