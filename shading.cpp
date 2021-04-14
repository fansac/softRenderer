#include "shading.hpp"

void gouraud_shading(mesh::TriangleMesh& mesh, rst::Rasterizer& r) {
	Eigen::Vector3d light = { -1, -1, -1 };
	Eigen::Vector3d c_l = Eigen::Vector3d(1, 1, 1) * 0.9;
	Eigen::Vector3d c_a = Eigen::Vector3d(1, 1, 1) * 0.15;
	Eigen::Vector3d c_p = Eigen::Vector3d(1, 1, 1) * 0.8;
	auto m_eye = r.m_cam * r.m_model;
	auto m_n = (r.m_uvw * r.m_model.block(0, 0, 3, 3)).inverse().transpose();
	Eigen::Vector3d direc_light = -(r.m_uvw * light).normalized();
	std::cout << "light direction: " << std::endl << direc_light << std::endl;
	auto f1 = (r.near - r.far) / 2;
	auto f2 = (r.near + r.far) / 2;
	int num_t = 0;
	clock_t start_time, end_time;
	start_time = clock();
	for (auto iter = mesh.triangles.begin(); iter != mesh.triangles.end(); ++iter) {
		std::cout << "number of triangle: " << ++num_t << std::endl;
		Eigen::Vector3d pixels[3];
		Eigen::Vector3d colors[3];
		double w[3];
		for (unsigned int i = 0; i < 3; ++i) {
			Eigen::Vector3d position_i = mesh.vertices[iter->v[i]].get_position();
			// (pixel.x, pixel.y) in [WIDTH, HEIGHT]
			auto homo = r.mvp * position_i.homogeneous();
			w[i] = homo.w();
			pixels[i] = util_rd::homo_to_v3(homo);
			// convert z depth value from [-1, 1] to [far, near];
			pixels[i].z() = f1 * pixels[i].z() + f2;
			Eigen::Vector3d e = (-position_i).normalized();
			Eigen::Vector3d h = (e + direc_light).normalized();
			Eigen::Vector3d normal_i = (m_n * mesh.normals[iter->n[i]]).normalized();
			Eigen::Vector3d c_r = { 0, 0, 1.0 };

			pixels[i] = util_rd::homo_to_v3(r.mvp * position_i.homogeneous());

			auto c_ambient = c_r.array() * c_a.array();
			auto c_diffuse = c_r.array() * (c_l * util_rd::clip(normal_i.dot(direc_light), 0.0, 1.0)).array();
			auto c_highlight = c_l.array() * c_p.array() * pow(std::max(h.dot(normal_i), 0.0), PHONGEXP);
			colors[i] = (c_ambient + c_diffuse + c_highlight).min(Eigen::Array3d(1, 1, 1)).max(Eigen::Array3d(0, 0, 0));


		}

		auto p0 = pixels[0];
		auto p1 = pixels[1];
		auto p2 = pixels[2];

		auto x_range = util_rd::get_range_of_three(p0.x(), p1.x(), p2.x());
		auto y_range = util_rd::get_range_of_three(p0.y(), p1.y(), p2.y());

		auto x_min = util_rd::clip(static_cast<size_t>(x_range.first), static_cast<size_t>(0), r.n_x - 1);
		auto x_max = util_rd::clip(static_cast<size_t>(x_range.second + 1), static_cast<size_t>(0), r.n_x - 1);
		auto y_min = util_rd::clip(static_cast<size_t>(y_range.first), static_cast<size_t>(0), r.n_y - 1);
		auto y_max = util_rd::clip(static_cast<size_t>(y_range.second + 1), static_cast<size_t>(0), r.n_y - 1);
		rst::Triangle2D tri({ p0.x(), p0.y() }, { p1.x(), p1.y() }, { p2.x(), p2.y() });
		for (auto x = x_min; x <= x_max; ++x) {
			for (auto y = y_min; y <= y_max; ++y) {
				if (tri.is_inside({ static_cast<double>(x), static_cast<double>(y) })) {
					auto barycentric_coordinates = util_rd::compute_barycentric_2D(x, y, { {p0.x(), p0.y()}, {p1.x(), p1.y()},{p2.x(), p2.y()} });
					auto alpha = std::get<0>(barycentric_coordinates);
					auto beta = std::get<1>(barycentric_coordinates);
					auto gamma = std::get<2>(barycentric_coordinates);
					double z_view = 1.0 / (alpha / w[0] + beta / w[1] + gamma / w[2]);
					double z_depth_correct = (alpha * p0.z() / w[0] + beta * p1.z() / w[1] + gamma * p2.z() / w[2]) * z_view;
					if (r.compare_pixel_in_z_buffer(x, y, r.to_z_buffer_value(z_depth_correct))) {
						auto c = (alpha * colors[0] / w[0] + beta * colors[1] / w[1] + gamma * colors[2] / w[2]) * z_view;
						r.draw_pixel({ x, y, c * 255 });
					}
				}

			}
		}
	}
}

void phong_shading(mesh::TriangleMesh& mesh, rst::Rasterizer& r) {
	Eigen::Vector3d light = { -1, -1, -1 };
	Eigen::Vector3d c_l = Eigen::Vector3d(1, 1, 1) * 0.9;
	Eigen::Vector3d c_a = Eigen::Vector3d(1, 1, 1) * 0.15;
	Eigen::Vector3d c_p = Eigen::Vector3d(1, 1, 1) * 0.8;
	auto m_eye = r.m_cam * r.m_model;
	auto m_n = (r.m_uvw * r.m_model.block(0, 0, 3, 3)).inverse().transpose();
	Eigen::Vector3d direc_light = -(r.m_uvw * light).normalized();
	std::cout << "light direction: " << std::endl << direc_light << std::endl;
	auto f1 = (r.near - r.far) / 2;
	auto f2 = (r.near + r.far) / 2;
	int num_t = 0;
	clock_t start_time, end_time;
	start_time = clock();
	for (auto iter = mesh.triangles.begin(); iter != mesh.triangles.end(); ++iter) {
		std::cout << "number of triangle: " << ++num_t << std::endl;
		Eigen::Vector3d pixels[3];
		Eigen::Vector3d normals[3];
		double w[3];
		Eigen::Vector3d points_in_view[3];
		Eigen::Vector3d colors[3];
		for (unsigned int i = 0; i < 3; ++i) {
			Eigen::Vector3d position_i = mesh.vertices[iter->v[i]].get_position();
			// (pixel.x, pixel.y) in [WIDTH, HEIGHT]
			auto homo = r.mvp * position_i.homogeneous();
			w[i] = homo.w();
			pixels[i] = util_rd::homo_to_v3(homo);
			// convert z depth value from [-1, 1] to [far, near];
			pixels[i].z() = f1 * pixels[i].z() + f2;
			// view/camera space
			//normals[i] = (m_n * mesh.vertices[iter->v[i]].get_normal()).normalized();
			normals[i] = (m_n * mesh.normals[iter->n[i]]).normalized();
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

		auto x_min = util_rd::clip(static_cast<size_t>(x_range.first), static_cast<size_t>(0), r.n_x-1);
		auto x_max = util_rd::clip(static_cast<size_t>(x_range.second + 1), static_cast<size_t>(0), r.n_x-1);
		auto y_min = util_rd::clip(static_cast<size_t>(y_range.first), static_cast<size_t>(0), r.n_y-1);
		auto y_max = util_rd::clip(static_cast<size_t>(y_range.second + 1), static_cast<size_t>(0), r.n_y-1);
		rst::Triangle2D tri({ p0.x(), p0.y() }, { p1.x(), p1.y() }, { p2.x(), p2.y() });
		for (auto x = x_min; x <= x_max; ++x) {
			for (auto y = y_min; y <= y_max; ++y) {
				if (tri.is_inside({static_cast<double>(x), static_cast<double>(y) })){
					auto barycentric_coordinates = util_rd::compute_barycentric_2D(x, y, { {p0.x(), p0.y()}, {p1.x(), p1.y()},{p2.x(), p2.y()} });
					auto alpha = std::get<0>(barycentric_coordinates);
					auto beta = std::get<1>(barycentric_coordinates);
					auto gamma = std::get<2>(barycentric_coordinates);
					double z_view = 1.0 / (alpha / w[0] + beta / w[1] + gamma / w[2]);
					double z_depth_correct = (alpha * p0.z() / w[0] + beta * p1.z() / w[1] + gamma * p2.z() / w[2]) * z_view;
					if (r.compare_pixel_in_z_buffer(x, y, r.to_z_buffer_value(z_depth_correct))) {
						double x_view = (alpha * pv0.x() / w[0] + beta * pv1.x() / w[1] + gamma * pv2.x() / w[2]) * z_view;
						double y_view = (alpha * pv0.y() / w[0] + beta * pv1.y() / w[1] + gamma * pv2.y() / w[2]) * z_view;
						Eigen::Vector3d e = (-Eigen::Vector3d(x_view, y_view, z_view)).normalized();
						Eigen::Vector3d h = (e + direc_light).normalized();
						
						auto c_r = ((alpha * colors[0] / w[0] + beta * colors[1] / w[1] + gamma * colors[2] / w[2]) * z_view) / 255.0;
						Eigen::Vector3d n = ((alpha * normals[0] / w[0] + beta * normals[1] / w[1] + gamma * normals[2] / w[2]) * z_view).normalized();

						auto c_ambient = c_r.array() * c_a.array();
						auto c_diffuse = c_r.array() * (c_l * util_rd::clip(n.dot(direc_light), 0.0, 1.0)).array();
						auto c_highlight = c_l.array() * c_p.array() * pow(std::max(h.dot(n), 0.0), PHONGEXP);
						Eigen::Vector3d c = (c_ambient + c_diffuse + c_highlight).min(Eigen::Array3d(1, 1, 1)).max(Eigen::Array3d(0, 0, 0));						
						r.draw_pixel({ x, y, c*255 });

					}
				}
				
			}
		}
		//system("pause");
		//auto screen = r.canvas_2_screen();
		//cv::Mat img(WINDOW_HEIGHT, WINDOW_WIDTH, CV_64FC3, screen.data());
		//img.convertTo(img, CV_8UC3, 1.0f);
		//cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
		// CV_WINDOW_NORMAL
		//cv::namedWindow("test_draw_line", CV_WINDOW_NORMAL);
		//cv::imshow("test_draw_line", img);
		//cv::waitKey();
		//cv::imwrite(mesh.obj_name + ".jpg", img);
		//cv::destroyWindow("test_draw_line");
		//cout << "drawing times " << r.test_index << endl;
	}
	end_time = clock();
	double lasting_time = (static_cast<double>(end_time) - start_time) / CLOCKS_PER_SEC;
	std::cout << "time of shading: " << lasting_time << "s" << std::endl;
}

void phong_shading(mesh::TriangleMesh& mesh, rst::Rasterizer& r, tex::Texture& tex) {
	Eigen::Vector3d light_position = { 15, 15, 15 };
	Eigen::Vector3d light_intensity = { 500, 500, 500 };
	Eigen::Vector3d ambient_light_intensity = { 10, 10, 10 };
	Eigen::Vector3d ka = { 0.005, 0.005, 0.005 };
	Eigen::Vector3d ks = { 0.7937, 0.7937, 0.7937 };


	auto m_eye = r.m_cam * r.m_model;
	auto m_n = (r.m_uvw * r.m_model.block(0,0,3,3)).inverse().transpose();
	auto light_pos_view = util_rd::homo_to_v3(r.m_cam * light_position.homogeneous());
	//Eigen::Vector3d direc_light = -(r.m_uvw * light).normalized();
	//std::cout << "light direction: " << std::endl << direc_light << std::endl;
	auto f1 = (r.near - r.far) / 2;
	auto f2 = (r.near + r.far) / 2;
	int num_tri = 0;

	clock_t start_time, end_time;
	start_time = clock();
	for (auto iter = mesh.triangles.begin(); iter != mesh.triangles.end(); ++iter) {
		std::cout << "number of triangle: " << ++num_tri << std::endl;
		Eigen::Vector3d pixels[3];
		Eigen::Vector3d normals[3];
		Eigen::Vector3d points_in_view[3];
		Eigen::Vector2d texcoord[3];
		double w[3];
		for (unsigned int i = 0; i < 3; ++i) {
			Eigen::Vector3d position_i = mesh.vertices[iter->v[i]].get_position();
			// (pixel.x, pixel.y) in [WIDTH, HEIGHT]
			auto homo = r.mvp * position_i.homogeneous();
			w[i] = homo.w();
			pixels[i] = util_rd::homo_to_v3(homo);
			// convert z depth value from [-1, 1] to [far, near];
			pixels[i].z() = f1 * pixels[i].z() + f2;
			// view/camera space
			//normals[i] = (m_n * mesh.vertices[iter->v[i]].get_normal()).normalized();
			normals[i] = (m_n * mesh.normals[iter->n[i]]).normalized();
			points_in_view[i] = util_rd::homo_to_v3(m_eye * position_i.homogeneous());
			texcoord[i] = mesh.texcoords[iter->t[i]];
		}
		auto p0 = pixels[0];
		auto p1 = pixels[1];
		auto p2 = pixels[2];
		auto pv0 = points_in_view[0];
		auto pv1 = points_in_view[1];
		auto pv2 = points_in_view[2];
		auto x_range = util_rd::get_range_of_three(p0.x(), p1.x(), p2.x());
		auto y_range = util_rd::get_range_of_three(p0.y(), p1.y(), p2.y());

		auto x_min = util_rd::clip(static_cast<size_t>(x_range.first), static_cast<size_t>(0), r.n_x - 1);
		auto x_max = util_rd::clip(static_cast<size_t>(x_range.second + 1), static_cast<size_t>(0), r.n_x - 1);
		auto y_min = util_rd::clip(static_cast<size_t>(y_range.first), static_cast<size_t>(0), r.n_y - 1);
		auto y_max = util_rd::clip(static_cast<size_t>(y_range.second + 1), static_cast<size_t>(0), r.n_y - 1);
		rst::Triangle2D tri({ p0.x(), p0.y() }, { p1.x(), p1.y() }, { p2.x(), p2.y() });
		for (auto x = x_min; x <= x_max; ++x) {
			for (auto y = y_min; y <= y_max; ++y) {
				if (tri.is_inside({ static_cast<double>(x), static_cast<double>(y) })) {
					auto barycentric_coordinates = util_rd::compute_barycentric_2D(x, y, { {p0.x(), p0.y()}, {p1.x(), p1.y()},{p2.x(), p2.y()} });
					auto alpha = std::get<0>(barycentric_coordinates);
					auto beta = std::get<1>(barycentric_coordinates);
					auto gamma = std::get<2>(barycentric_coordinates);
					double z_view = 1.0 / (alpha / w[0] + beta / w[1] + gamma / w[2]);
					double z_depth_correct = (alpha * p0.z() / w[0] + beta * p1.z() / w[1] + gamma * p2.z() / w[2]) * z_view;
					if (r.compare_pixel_in_z_buffer(x, y, r.to_z_buffer_value(z_depth_correct))) {
						double x_view = (alpha * pv0.x() / w[0] + beta * pv1.x() / w[1] + gamma * pv2.x() / w[2]) * z_view;
						double y_view = (alpha * pv0.y() / w[0] + beta * pv1.y() / w[1] + gamma * pv2.y() / w[2]) * z_view;
						Eigen::Vector3d p = Eigen::Vector3d(x_view, y_view, z_view);
						Eigen::Vector3d e = (-p).normalized();
						double r_2 = (light_pos_view - p).squaredNorm();
						Eigen::Vector3d l = (light_pos_view - p).normalized();
						Eigen::Vector3d h = (e + l).normalized();
						double u = (alpha * texcoord[0][0] / w[0] + beta * texcoord[1][0] / w[1] + gamma * texcoord[2][0] / w[2]) * z_view;
						double v = (alpha * texcoord[0][1] / w[0] + beta * texcoord[1][1] / w[1] + gamma * texcoord[2][1] / w[2]) * z_view;
						u = tex::texcoord_wrap(u);
						v = tex::texcoord_wrap(v);
						auto kd = tex.get_color(u, v) / 255;
						Eigen::Vector3d n = ((alpha * normals[0] / pv0.z() + beta * normals[1] / pv1.z() + gamma * normals[2] / pv2.z()) * z_view).normalized();

						//double cosine = util_rd::clip(n.dot(l), 0.0, 1.0);
						//double phong_coe = pow(std::max(h.dot(n), 0.0), PHONGEXP);
						// Blinn Phong lighting model
						auto c_ambient = ka.array() * ambient_light_intensity.array();
						auto c_diffuse = kd.array() * (light_intensity / r_2).array() * util_rd::clip(n.dot(l), 0.0, 1.0); //c_r.array() * (c_l * cosine).array();
						auto c_highlight = ks.array() * (light_intensity / r_2).array() * pow(std::max(h.dot(n), 0.0), PHONGEXP);
						Eigen::Vector3d c = (c_ambient + c_diffuse + c_highlight).min(Eigen::Array3d(1, 1, 1)).max(Eigen::Array3d(0, 0, 0));

						r.draw_pixel({ x, y, c * 255 });

					}
				}

			}
		}
	}
	end_time = clock();
	double lasting_time = (static_cast<double>(end_time) - start_time) / CLOCKS_PER_SEC;
	std::cout << "time of shading: " << lasting_time << "s" << std::endl;
}
