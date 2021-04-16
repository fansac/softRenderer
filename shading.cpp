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
	for (const auto& iter : mesh.triangles) {
		std::cout << "number of triangle: " << ++num_t << std::endl;
		Eigen::Vector3d pixels[3];
		Eigen::Vector3d colors[3];
		double w[3];
		for (unsigned int i = 0; i < 3; ++i) {
			Eigen::Vector3d position_i = mesh.vertices[iter.v[i]].get_position();
			// (pixel.x, pixel.y) in [WIDTH, HEIGHT]
			auto homo = r.mvp * r.m_model * position_i.homogeneous();
			w[i] = homo.w();
			pixels[i] = util_rd::homo_to_v3(homo);
			// convert z depth value from [-1, 1] to [far, near];
			pixels[i].z() = f1 * pixels[i].z() + f2;
			Eigen::Vector3d e = (-position_i).normalized();
			Eigen::Vector3d h = (e + direc_light).normalized();
			Eigen::Vector3d normal_i = (m_n * mesh.normals[iter.n[i]]).normalized();
			Eigen::Vector3d c_r = { 0, 0, 1.0 };

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
	end_time = clock();
	double lasting_time = (static_cast<double>(end_time) - start_time) / CLOCKS_PER_SEC;
	std::cout << "time of shading: " << lasting_time << "s" << std::endl;
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
	for (const auto& iter : mesh.triangles) {
		std::cout << "number of triangle: " << ++num_t << std::endl;
		Eigen::Vector3d pixels[3];
		Eigen::Vector3d normals[3];
		double w[3];
		Eigen::Vector3d points_in_view[3];
		Eigen::Vector3d colors[3];
		for (unsigned int i = 0; i < 3; ++i) {
			Eigen::Vector3d position_i = mesh.vertices[iter.v[i]].get_position();
			// (pixel.x, pixel.y) in [WIDTH, HEIGHT]
			auto homo = r.mvp * r.m_model * r.m_model * position_i.homogeneous();
			w[i] = homo.w();
			pixels[i] = util_rd::homo_to_v3(homo);
			// convert z depth value from [-1, 1] to [far, near];
			pixels[i].z() = f1 * pixels[i].z() + f2;
			// view/camera space
			//normals[i] = (m_n * mesh.vertices[iter->v[i]].get_normal()).normalized();
			normals[i] = (m_n * mesh.normals[iter.n[i]]).normalized();
			points_in_view[i] = util_rd::homo_to_v3(r.m_cam * r.m_model * position_i.homogeneous());
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


	auto m_n = (r.m_uvw * r.m_model.block(0,0,3,3)).inverse().transpose();
	auto light_pos_view = util_rd::homo_to_v3(r.m_cam * light_position.homogeneous());
	//Eigen::Vector3d direc_light = -(r.m_uvw * light).normalized();
	//std::cout << "light direction: " << std::endl << direc_light << std::endl;
	auto f1 = (r.near - r.far) / 2;
	auto f2 = (r.near + r.far) / 2;
	int num_tri = 0;

	clock_t start_time, end_time;
	start_time = clock();
	for (const auto& iter : mesh.triangles) {
		std::cout << "number of triangle: " << ++num_tri << std::endl;
		Eigen::Vector3d pixels[3];
		Eigen::Vector3d normals[3];
		Eigen::Vector3d points_in_view[3];
		Eigen::Vector2d texcoord[3];
		double w[3];
		for (unsigned int i = 0; i < 3; ++i) {
			Eigen::Vector3d position_i = mesh.vertices[iter.v[i]].get_position();
			// (pixel.x, pixel.y) in [WIDTH, HEIGHT]
			auto homo = r.mvp * r.m_model * position_i.homogeneous();
			w[i] = homo.w();
			pixels[i] = util_rd::homo_to_v3(homo);
			// convert z depth value from [-1, 1] to [far, near];
			pixels[i].z() = f1 * pixels[i].z() + f2;
			// view/camera space
			//normals[i] = (m_n * mesh.vertices[iter->v[i]].get_normal()).normalized();
			normals[i] = (m_n * mesh.normals[iter.n[i]]).normalized();
			points_in_view[i] = util_rd::homo_to_v3(r.m_cam * r.m_model * position_i.homogeneous());
			texcoord[i] = mesh.texcoords[iter.t[i]];
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

void phong_shading_shadow(mesh::TriangleMesh& mesh, rst::Rasterizer& r) {
	// light
	Eigen::Vector3d light_position = { 5, 15, 5 };
	Eigen::Vector3d light_intensity = { 500, 500, 500 };
	Eigen::Vector3d ambient_light_intensity = { 20, 20, 20 };
	Eigen::Vector3d ka = { 0.005, 0.005, 0.005 };
	Eigen::Vector3d ks = { 0.7937, 0.7937, 0.7937 };

	// plane
	tex::Texture plane_tex = tex::Texture("floor.jpg");
	std::vector<Eigen::Vector3d> plane_vertices = { {-4, -1, -4}, {-4, -1, 4}, {4, -1, 4}, {4, -1, -4} };
	std::vector<Eigen::Vector2d> plane_texcoords = { {-1, 1},     {-1, -1},    {1, -1},    {1, 1} };
	std::vector<Eigen::Vector3d> phane_triangles = { {0, 2, 3}, {0, 1, 2} };
	


	auto m_eye = r.m_cam * r.m_model;
	auto m_n = (r.m_uvw * r.m_model.block(0, 0, 3, 3)).inverse().transpose();
	auto light_pos_view = util_rd::homo_to_v3(r.m_cam * light_position.homogeneous());

	// shadow mapping begin
	/*std::vector<Eigen::Vector3d> aabb = { {mesh.x_min, mesh.y_min, mesh.z_min},
										  {mesh.x_max, mesh.y_min, mesh.z_min},
										  {mesh.x_max, mesh.y_max, mesh.z_min},
										  {mesh.x_min, mesh.y_max, mesh.z_min},
										  {mesh.x_min, mesh.y_min, mesh.z_max},
										  {mesh.x_max, mesh.y_min, mesh.z_max},
										  {mesh.x_max, mesh.y_max, mesh.z_max},
										  {mesh.x_min, mesh.y_max, mesh.z_max} };*/

	rst::Rasterizer s(480, 480);
	s.set_model_transformation(140, 2.5);
	s.set_view_volume(45, -1, -30);
	s.set_camera(light_position, -light_position.normalized(), Eigen::Vector3d(0, 1, 0));
	//s.set_camera({ 0, 10, 10 }, {0, -1, -1}, Eigen::Vector3d(0, 1, 0));
	s.calculate_matrix();
	auto s_f1 = (s.near - s.far) / 2;
	auto s_f2 = (s.near + s.far) / 2;


	for (auto iter: phane_triangles) {
		Eigen::Vector3d pixels[3];
		Eigen::Vector2d texcoord[3];
		double w[3];
		for (unsigned int i = 0; i < 3; ++i) {
			Eigen::Vector3d position_i = plane_vertices[iter[i]];
			// (pixel.x, pixel.y) in [WIDTH, HEIGHT]
			auto homo = s.mvp * position_i.homogeneous();
			w[i] = homo.w();
			pixels[i] = util_rd::homo_to_v3(homo);
			// convert z depth value from [-1, 1] to [far, near];
			pixels[i].z() = s_f1 * pixels[i].z() + s_f2;
			// view/camera space
			//normals[i] = (m_n * mesh.vertices[iter->v[i]].get_normal()).normalized();
			
			texcoord[i] = plane_texcoords[iter[i]];
		}
		auto p0 = pixels[0];
		auto p1 = pixels[1];
		auto p2 = pixels[2];
		auto x_range = util_rd::get_range_of_three(p0.x(), p1.x(), p2.x());
		auto y_range = util_rd::get_range_of_three(p0.y(), p1.y(), p2.y());

		auto x_min = util_rd::clip(static_cast<size_t>(x_range.first), static_cast<size_t>(0), s.n_x - 1);
		auto x_max = util_rd::clip(static_cast<size_t>(x_range.second + 1), static_cast<size_t>(0), s.n_x - 1);
		auto y_min = util_rd::clip(static_cast<size_t>(y_range.first), static_cast<size_t>(0), s.n_y - 1);
		auto y_max = util_rd::clip(static_cast<size_t>(y_range.second + 1), static_cast<size_t>(0), s.n_y - 1);
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
					auto _ = s.compare_pixel_in_z_buffer(x, y, s.to_z_buffer_value(z_depth_correct));
				}
			}
		}
	}

	int s_num_tri = 0;
	for (const auto& iter : mesh.triangles) {
		std::cout << "shadow mapping - number of triangle: " << ++s_num_tri << std::endl;
		Eigen::Vector3d pixels[3];
		double w[3];
		for (unsigned int i = 0; i < 3; ++i) {
			Eigen::Vector3d position_i = mesh.vertices[iter.v[i]].get_position();
			// (pixel.x, pixel.y) in [WIDTH, HEIGHT]
			auto homo = s.mvp * r.m_model * position_i.homogeneous();
			w[i] = homo.w();
			pixels[i] = util_rd::homo_to_v3(homo);
			// convert z depth value from [-1, 1] to [far, near];
			pixels[i].z() = s_f1 * pixels[i].z() + s_f2;
		}

		auto p0 = pixels[0];
		auto p1 = pixels[1];
		auto p2 = pixels[2];

		auto x_range = util_rd::get_range_of_three(p0.x(), p1.x(), p2.x());
		auto y_range = util_rd::get_range_of_three(p0.y(), p1.y(), p2.y());

		auto x_min = util_rd::clip(static_cast<size_t>(x_range.first), static_cast<size_t>(0), s.n_x - 1);
		auto x_max = util_rd::clip(static_cast<size_t>(x_range.second + 1), static_cast<size_t>(0), s.n_x - 1);
		auto y_min = util_rd::clip(static_cast<size_t>(y_range.first), static_cast<size_t>(0), s.n_y - 1);
		auto y_max = util_rd::clip(static_cast<size_t>(y_range.second + 1), static_cast<size_t>(0), s.n_y - 1);
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
					auto temp = s.compare_pixel_in_z_buffer(x, y, s.to_z_buffer_value(z_depth_correct));
					
				}
			}
		}
	}

	system("pause");
	std::vector<uint8_t> shadow_map(s.n_x * s.n_y, 255);
	int index = 0;
	for (auto iter : s.z_buffer) {
		shadow_map[index++] = static_cast<double>(iter) / static_cast<double>(UINT32_MAX) * 255.0;
	}
	cv::Mat s_img(s.n_x, s.n_y, CV_8UC1, shadow_map.data());
	cv::flip(s_img, s_img, 0);
	//CV_WINDOW_NORMAL
	cv::namedWindow("test_shadow_map", CV_WINDOW_NORMAL);
	cv::imshow("test_shadow_map", s_img);
	cv::waitKey();
	cv::destroyWindow("test_shadow_map");

	// shadow map done

	auto f1 = (r.near - r.far) / 2;
	auto f2 = (r.near + r.far) / 2;
	int num_tri = 0;

	clock_t start_time, end_time;
	start_time = clock();
	for (const auto& iter : mesh.triangles) {
		std::cout << "number of triangle: " << ++num_tri << std::endl;
		Eigen::Vector3d pixels[3];
		Eigen::Vector3d normals[3];
		Eigen::Vector3d points_in_view[3];
		Eigen::Vector2d texcoord[3];
		double w[3];
		for (unsigned int i = 0; i < 3; ++i) {
			Eigen::Vector3d position_i = mesh.vertices[iter.v[i]].get_position();
			// (pixel.x, pixel.y) in [WIDTH, HEIGHT]
			auto homo = r.mvp * r.m_model * position_i.homogeneous();
			w[i] = homo.w();
			pixels[i] = util_rd::homo_to_v3(homo);
			// convert z depth value from [-1, 1] to [far, near];
			pixels[i].z() = f1 * pixels[i].z() + f2;
			// view/camera space
			//normals[i] = (m_n * mesh.vertices[iter->v[i]].get_normal()).normalized();
			normals[i] = (m_n * mesh.normals[iter.n[i]]).normalized();
			points_in_view[i] = util_rd::homo_to_v3(r.m_cam * r.m_model * position_i.homogeneous());
			texcoord[i] = mesh.texcoords[iter.t[i]];
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
						// depth test
						int is_visible = 1;
						Eigen::Vector3d p_world = util_rd::homo_to_v3(r.m_cam.inverse() * p.homogeneous());
						Eigen::Vector3d p_pixel = util_rd::homo_to_v3(s.mvp * r.m_cam.inverse() * p.homogeneous());

						size_t p_pixel_x = round(p_pixel.x());
						size_t p_pixel_y = round(p_pixel.y());
						double p_pixel_z = s_f1 * p_pixel.z() + s_f2;
						auto p_depth = s.to_z_buffer_value(p_pixel_z);
						
						if (p_depth <= s.z_buffer[p_pixel_y * s.n_x + p_pixel_x] + static_cast<double>(UINT32_MAX) / 500) {
							is_visible = 1;
						}
						else {
							is_visible = 0;
						}
						Eigen::Vector3d e = (-p).normalized();
						double r_2 = (light_pos_view - p).squaredNorm();
						Eigen::Vector3d l = (light_pos_view - p).normalized();
						Eigen::Vector3d h = (e + l).normalized();

						auto kd = Eigen::Vector3d(0, 0, 1);
						Eigen::Vector3d n = ((alpha * normals[0] / pv0.z() + beta * normals[1] / pv1.z() + gamma * normals[2] / pv2.z()) * z_view).normalized();

						//double cosine = util_rd::clip(n.dot(l), 0.0, 1.0);
						//double phong_coe = pow(std::max(h.dot(n), 0.0), PHONGEXP);
						// Blinn Phong lighting model
						auto c_ambient = ka.array() * ambient_light_intensity.array();
						auto c_diffuse = kd.array() * (light_intensity / r_2).array() * util_rd::clip(n.dot(l), 0.0, 1.0); //c_r.array() * (c_l * cosine).array();
						auto c_highlight = ks.array() * (light_intensity / r_2).array() * pow(std::max(h.dot(n), 0.0), PHONGEXP);
						Eigen::Vector3d c = (c_ambient + c_diffuse + c_highlight).min(Eigen::Array3d(1, 1, 1)).max(Eigen::Array3d(0, 0, 0));

						r.draw_pixel({ x, y, c * 255 * is_visible});

					}
				}

			}
		}
	}
	end_time = clock();
	double lasting_time = (static_cast<double>(end_time) - start_time) / CLOCKS_PER_SEC;
	std::cout << "time of shading: " << lasting_time << "s" << std::endl;

	
	for (auto iter: phane_triangles) {
		Eigen::Vector3d pixels[3];
		Eigen::Vector3d normals[3];
		Eigen::Vector3d points_in_view[3];
		Eigen::Vector2d texcoord[3];
		double w[3];
		for (unsigned int i = 0; i < 3; ++i) {
			Eigen::Vector3d position_i = plane_vertices[iter[i]];
			// (pixel.x, pixel.y) in [WIDTH, HEIGHT]
			auto homo = r.mvp * position_i.homogeneous();
			w[i] = homo.w();
			pixels[i] = util_rd::homo_to_v3(homo);
			// convert z depth value from [-1, 1] to [far, near];
			pixels[i].z() = f1 * pixels[i].z() + f2;
			// view/camera space
			//normals[i] = (m_n * mesh.vertices[iter->v[i]].get_normal()).normalized();
			auto s_m_n = (s.m_uvw).inverse().transpose();
			normals[i] = s_m_n * Eigen::Vector3d(0, 1, 0);
			points_in_view[i] = util_rd::homo_to_v3(r.m_cam * r.m_model * position_i.homogeneous());
			texcoord[i] = plane_texcoords[iter[i]];
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

						// depth test
						int is_visible = 1;
						Eigen::Vector3d p_world = util_rd::homo_to_v3(r.m_cam.inverse() * p.homogeneous());
						Eigen::Vector3d p_pixel = util_rd::homo_to_v3(s.mvp * r.m_cam.inverse() * p.homogeneous());

						size_t p_pixel_x = round(p_pixel.x());
						size_t p_pixel_y = round(p_pixel.y());
						double p_pixel_z = s_f1 * p_pixel.z() + s_f2;
						auto p_depth = s.to_z_buffer_value(p_pixel_z);
						
						if (p_depth <= s.z_buffer[p_pixel_y * s.n_x + p_pixel_x] + +static_cast<double>(UINT32_MAX) / 500) {
							is_visible = 1;
						}
						else {
							is_visible = 0;
						}

						double r_2 = (light_pos_view - p).squaredNorm();
						Eigen::Vector3d l = (light_pos_view - p).normalized();
						Eigen::Vector3d h = (e + l).normalized();

						double u = (alpha * texcoord[0][0] / w[0] + beta * texcoord[1][0] / w[1] + gamma * texcoord[2][0] / w[2]) * z_view;
						double v = (alpha * texcoord[0][1] / w[0] + beta * texcoord[1][1] / w[1] + gamma * texcoord[2][1] / w[2]) * z_view;
						u = tex::texcoord_wrap(u);
						v = tex::texcoord_wrap(v);
						auto kd = plane_tex.get_color(u, v) / 255;
						//auto kd = Eigen::Vector3d(0, 0, 1);
						Eigen::Vector3d n = ((alpha * normals[0] / pv0.z() + beta * normals[1] / pv1.z() + gamma * normals[2] / pv2.z()) * z_view).normalized();

						//double cosine = util_rd::clip(n.dot(l), 0.0, 1.0);
						//double phong_coe = pow(std::max(h.dot(n), 0.0), PHONGEXP);
						// Blinn Phong lighting model
						auto c_ambient = ka.array() * ambient_light_intensity.array();
						auto c_diffuse = kd.array() * (light_intensity / r_2).array() * util_rd::clip(n.dot(l), 0.0, 1.0); //c_r.array() * (c_l * cosine).array();
						auto c_highlight = ks.array() * (light_intensity / r_2).array() * pow(std::max(h.dot(n), 0.0), PHONGEXP);
						Eigen::Vector3d c = (c_ambient + c_diffuse + c_highlight).min(Eigen::Array3d(1, 1, 1)).max(Eigen::Array3d(0, 0, 0));

						r.draw_pixel({ x, y, c * 255 * is_visible});

					}
				}

			}
		}
	}
}


void phong_shading_shadow(mesh::TriangleMesh& mesh, rst::Rasterizer& r, tex::Texture& tex) {
	// light
	Eigen::Vector3d light_position = { 15, 15, 15 };
	Eigen::Vector3d light_intensity = { 500, 500, 500 };
	Eigen::Vector3d ambient_light_intensity = { 10, 10, 10 };
	Eigen::Vector3d ka = { 0.005, 0.005, 0.005 };
	Eigen::Vector3d ks = { 0.7937, 0.7937, 0.7937 };

	// plane
	tex::Texture plane_tex = tex::Texture("floor.jpg");
	std::vector<Eigen::Vector3d> plane_vertices = { {-8, -3, -8}, {-8, -3, 4}, {4, -3, 4}, {8, -3, -8} };
	std::vector<Eigen::Vector2d> plane_texcoords = { {-2, 2},     {-2, -1},    {1, -1},    {1, 2} };
	std::vector<Eigen::Vector3d> phane_triangles = { {0, 2, 3}, {0, 1, 2} };



	auto m_eye = r.m_cam * r.m_model;
	auto m_n = (r.m_uvw * r.m_model.block(0, 0, 3, 3)).inverse().transpose();
	auto light_pos_view = util_rd::homo_to_v3(r.m_cam * light_position.homogeneous());

	// shadow mapping begin
	/*std::vector<Eigen::Vector3d> aabb = { {mesh.x_min, mesh.y_min, mesh.z_min},
										  {mesh.x_max, mesh.y_min, mesh.z_min},
										  {mesh.x_max, mesh.y_max, mesh.z_min},
										  {mesh.x_min, mesh.y_max, mesh.z_min},
										  {mesh.x_min, mesh.y_min, mesh.z_max},
										  {mesh.x_max, mesh.y_min, mesh.z_max},
										  {mesh.x_max, mesh.y_max, mesh.z_max},
										  {mesh.x_min, mesh.y_max, mesh.z_max} };*/

	rst::Rasterizer s(480, 480);
	s.set_model_transformation(140, 2.5);
	s.set_view_volume(45, -0.1, -50);
	s.set_camera(light_position, -light_position.normalized(), Eigen::Vector3d(0, 1, 0));
	//s.set_camera({ 0, 10, 10 }, {0, -1, -1}, Eigen::Vector3d(0, 1, 0));
	s.calculate_matrix();
	auto s_f1 = (s.near - s.far) / 2;
	auto s_f2 = (s.near + s.far) / 2;


	for (auto iter : phane_triangles) {
		Eigen::Vector3d pixels[3];

		Eigen::Vector2d texcoord[3];
		double w[3];
		for (unsigned int i = 0; i < 3; ++i) {
			Eigen::Vector3d position_i = plane_vertices[iter[i]];
			// (pixel.x, pixel.y) in [WIDTH, HEIGHT]
			auto homo = s.mvp * position_i.homogeneous();
			w[i] = homo.w();
			pixels[i] = util_rd::homo_to_v3(homo);
			// convert z depth value from [-1, 1] to [far, near];
			pixels[i].z() = s_f1 * pixels[i].z() + s_f2;
			// view/camera space
			//normals[i] = (m_n * mesh.vertices[iter->v[i]].get_normal()).normalized();
			texcoord[i] = plane_texcoords[iter[i]];
		}
		auto p0 = pixels[0];
		auto p1 = pixels[1];
		auto p2 = pixels[2];
		auto x_range = util_rd::get_range_of_three(p0.x(), p1.x(), p2.x());
		auto y_range = util_rd::get_range_of_three(p0.y(), p1.y(), p2.y());

		auto x_min = util_rd::clip(static_cast<size_t>(x_range.first), static_cast<size_t>(0), s.n_x - 1);
		auto x_max = util_rd::clip(static_cast<size_t>(x_range.second + 1), static_cast<size_t>(0), s.n_x - 1);
		auto y_min = util_rd::clip(static_cast<size_t>(y_range.first), static_cast<size_t>(0), s.n_y - 1);
		auto y_max = util_rd::clip(static_cast<size_t>(y_range.second + 1), static_cast<size_t>(0), s.n_y - 1);
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
					auto _ = s.compare_pixel_in_z_buffer(x, y, s.to_z_buffer_value(z_depth_correct));
				}
			}
		}
	}

	int s_num_tri = 0;
	for (const auto& iter : mesh.triangles) {
		std::cout << "shadow mapping - number of triangle: " << ++s_num_tri << std::endl;
		Eigen::Vector3d pixels[3];
		double w[3];
		for (unsigned int i = 0; i < 3; ++i) {
			Eigen::Vector3d position_i = mesh.vertices[iter.v[i]].get_position();
			// (pixel.x, pixel.y) in [WIDTH, HEIGHT]
			Eigen::Vector4d test_homo = r.m_model * position_i.homogeneous();
			Eigen::Vector3d test_model = util_rd::homo_to_v3(r.m_model * position_i.homogeneous());
			auto homo = s.mvp * r.m_model * position_i.homogeneous();
			w[i] = homo.w();
			pixels[i] = util_rd::homo_to_v3(homo);
			// convert z depth value from [-1, 1] to [far, near];
			pixels[i].z() = s_f1 * pixels[i].z() + s_f2;
		}

		auto p0 = pixels[0];
		auto p1 = pixels[1];
		auto p2 = pixels[2];

		auto x_range = util_rd::get_range_of_three(p0.x(), p1.x(), p2.x());
		auto y_range = util_rd::get_range_of_three(p0.y(), p1.y(), p2.y());

		auto x_min = util_rd::clip(static_cast<size_t>(x_range.first), static_cast<size_t>(0), s.n_x - 1);
		auto x_max = util_rd::clip(static_cast<size_t>(x_range.second + 1), static_cast<size_t>(0), s.n_x - 1);
		auto y_min = util_rd::clip(static_cast<size_t>(y_range.first), static_cast<size_t>(0), s.n_y - 1);
		auto y_max = util_rd::clip(static_cast<size_t>(y_range.second + 1), static_cast<size_t>(0), s.n_y - 1);
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
					auto temp = s.compare_pixel_in_z_buffer(x, y, s.to_z_buffer_value(z_depth_correct));

				}
			}
		}
	}

	system("pause");
	std::vector<uint8_t> shadow_map(s.n_x * s.n_y, 255);
	int index = 0;
	for (auto iter : s.z_buffer) {
		shadow_map[index++] = (static_cast<double>(iter) / static_cast<double>(UINT32_MAX) * 255.0);
	}
	cv::Mat s_img(s.n_x, s.n_y, CV_8UC1, shadow_map.data());
	cv::flip(s_img, s_img, 0);
	//CV_WINDOW_NORMAL
	cv::namedWindow("test_shadow_map", CV_WINDOW_NORMAL);
	cv::imshow("test_shadow_map", s_img);
	cv::waitKey();
	cv::destroyWindow("test_shadow_map");

	// shadow map done

	auto f1 = (r.near - r.far) / 2;
	auto f2 = (r.near + r.far) / 2;
	int num_tri = 0;

	clock_t start_time, end_time;
	start_time = clock();
	for (const auto& iter : mesh.triangles) {
		std::cout << "number of triangle: " << ++num_tri << std::endl;
		Eigen::Vector3d pixels[3];
		Eigen::Vector3d normals[3];
		Eigen::Vector3d points_in_view[3];
		Eigen::Vector2d texcoord[3];
		double w[3];
		for (unsigned int i = 0; i < 3; ++i) {
			Eigen::Vector3d position_i = mesh.vertices[iter.v[i]].get_position();
			// (pixel.x, pixel.y) in [WIDTH, HEIGHT]
			auto homo = r.mvp * r.m_model * position_i.homogeneous();
			w[i] = homo.w();
			pixels[i] = util_rd::homo_to_v3(homo);
			// convert z depth value from [-1, 1] to [far, near];
			pixels[i].z() = f1 * pixels[i].z() + f2;
			// view/camera space
			//normals[i] = (m_n * mesh.vertices[iter->v[i]].get_normal()).normalized();
			normals[i] = (m_n * mesh.normals[iter.n[i]]).normalized();
			points_in_view[i] = util_rd::homo_to_v3(r.m_cam * r.m_model * position_i.homogeneous());
			texcoord[i] = mesh.texcoords[iter.t[i]];
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
						// depth test
						int is_visible = 1;
						Eigen::Vector3d p_world = util_rd::homo_to_v3(r.m_cam.inverse() * p.homogeneous());
						Eigen::Vector3d p_pixel = util_rd::homo_to_v3(s.mvp * r.m_cam.inverse() * p.homogeneous());

						size_t p_pixel_x = round(p_pixel.x());
						size_t p_pixel_y = round(p_pixel.y());
						double p_pixel_z = s_f1 * p_pixel.z() + s_f2;
						auto p_depth = s.to_z_buffer_value(p_pixel_z);

						if (p_depth <= s.z_buffer[p_pixel_y * s.n_x + p_pixel_x] + static_cast<double>(UINT32_MAX) / 5000) {
							is_visible = 1;
						}
						else {
							is_visible = 0;
						}
						Eigen::Vector3d e = (-p).normalized();
						double r_2 = (light_pos_view - p).squaredNorm();
						Eigen::Vector3d l = (light_pos_view - p).normalized();
						Eigen::Vector3d h = (e + l).normalized();

						//auto kd = Eigen::Vector3d(0, 0, 1);
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

						r.draw_pixel({ x, y, c * 255 * is_visible });

					}
				}

			}
		}
	}
	end_time = clock();
	double lasting_time = (static_cast<double>(end_time) - start_time) / CLOCKS_PER_SEC;
	std::cout << "time of shading: " << lasting_time << "s" << std::endl;


	for (auto iter : phane_triangles) {
		Eigen::Vector3d pixels[3];
		Eigen::Vector3d normals[3];
		Eigen::Vector3d points_in_view[3];
		Eigen::Vector2d texcoord[3];
		double w[3];
		for (unsigned int i = 0; i < 3; ++i) {
			Eigen::Vector3d position_i = plane_vertices[iter[i]];
			// (pixel.x, pixel.y) in [WIDTH, HEIGHT]
			Eigen::Vector4d test_vec = r.m_cam * position_i.homogeneous();
			auto homo = r.mvp * position_i.homogeneous();
			w[i] = homo.w();
			pixels[i] = util_rd::homo_to_v3(homo);
			// convert z depth value from [-1, 1] to [far, near];
			pixels[i].z() = f1 * pixels[i].z() + f2;
			// view/camera space
			auto s_m_n = (s.m_uvw).inverse().transpose();
			normals[i] = s_m_n * Eigen::Vector3d(0, 1, 0);
			points_in_view[i] = util_rd::homo_to_v3(r.m_cam * position_i.homogeneous());
			texcoord[i] = plane_texcoords[iter[i]];
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

						// depth test
						int is_visible = 1;
						Eigen::Vector3d p_world = util_rd::homo_to_v3(r.m_cam.inverse() * p.homogeneous());
						Eigen::Vector3d test_vec = util_rd::homo_to_v3(s.mvp * p_world.homogeneous());
						Eigen::Vector3d p_pixel = util_rd::homo_to_v3(s.mvp * r.m_cam.inverse() * p.homogeneous());

						size_t p_pixel_x = round(p_pixel.x());
						size_t p_pixel_y = round(p_pixel.y());
						double p_pixel_z = s_f1 * p_pixel.z() + s_f2;
						auto p_depth = s.to_z_buffer_value(p_pixel_z);

						if (p_depth <= s.z_buffer[p_pixel_y * s.n_x + p_pixel_x] + static_cast<double>(UINT32_MAX) / 5000) {
							is_visible = 1;
						}
						else {
							is_visible = 0;
						}

						double r_2 = (light_pos_view - p).squaredNorm();
						Eigen::Vector3d l = (light_pos_view - p).normalized();
						Eigen::Vector3d h = (e + l).normalized();

						double u = (alpha * texcoord[0][0] / w[0] + beta * texcoord[1][0] / w[1] + gamma * texcoord[2][0] / w[2]) * z_view;
						double v = (alpha * texcoord[0][1] / w[0] + beta * texcoord[1][1] / w[1] + gamma * texcoord[2][1] / w[2]) * z_view;
						u = tex::texcoord_wrap(u);
						v = tex::texcoord_wrap(v);
						auto kd = plane_tex.get_color(u, v) / 255;
						//auto kd = Eigen::Vector3d(0, 0, 1);
						Eigen::Vector3d n = ((alpha * normals[0] / pv0.z() + beta * normals[1] / pv1.z() + gamma * normals[2] / pv2.z()) * z_view).normalized();

						//double cosine = util_rd::clip(n.dot(l), 0.0, 1.0);
						//double phong_coe = pow(std::max(h.dot(n), 0.0), PHONGEXP);
						// Blinn Phong lighting model
						auto c_ambient = ka.array() * ambient_light_intensity.array();
						auto c_diffuse = kd.array() * (light_intensity / r_2).array() * util_rd::clip(n.dot(l), 0.0, 1.0); //c_r.array() * (c_l * cosine).array();
						auto c_highlight = ks.array() * (light_intensity / r_2).array() * pow(std::max(h.dot(n), 0.0), PHONGEXP);
						Eigen::Vector3d c = (c_ambient + c_diffuse + c_highlight).min(Eigen::Array3d(1, 1, 1)).max(Eigen::Array3d(0, 0, 0));

						r.draw_pixel({ x, y, c * 255 * is_visible});

					}
				}

			}
		}
	}
}


void bump_shading(mesh::TriangleMesh& mesh, rst::Rasterizer& r, tex::Texture& tex) {
	std::vector<Eigen::Vector3d> light_position = {{ -20, 20, 20 }, { 20, 20, 20 } };
	Eigen::Vector3d light_intensity = { 400, 400, 400 };
	Eigen::Vector3d ambient_light_intensity = { 10, 10, 10 };
	Eigen::Vector3d ka = { 0.005, 0.005, 0.005 };
	Eigen::Vector3d ks = { 0.7937, 0.7937, 0.7937 };

	double kh = 0.2, kn = 0.1;

	auto m_n = (r.m_uvw * r.m_model.block(0, 0, 3, 3)).inverse().transpose();
	//auto light_pos_view = util_rd::homo_to_v3(r.m_cam * light_position.homogeneous());
	//Eigen::Vector3d direc_light = -(r.m_uvw * light).normalized();
	//std::cout << "light direction: " << std::endl << direc_light << std::endl;
	auto f1 = (r.near - r.far) / 2;
	auto f2 = (r.near + r.far) / 2;
	int num_tri = 0;

	clock_t start_time, end_time;
	start_time = clock();
	for (const auto& iter : mesh.triangles) {
		std::cout << "number of triangle: " << ++num_tri << std::endl;
		Eigen::Vector3d pixels[3];
		Eigen::Vector3d normals[3];
		Eigen::Vector3d points_in_view[3];
		Eigen::Vector2d texcoord[3];
		double w[3];
		for (unsigned int i = 0; i < 3; ++i) {
			Eigen::Vector3d position_i = mesh.vertices[iter.v[i]].get_position();
			// (pixel.x, pixel.y) in [WIDTH, HEIGHT]
			auto homo = r.mvp * r.m_model * position_i.homogeneous();
			w[i] = homo.w();
			pixels[i] = util_rd::homo_to_v3(homo);
			// convert z depth value from [-1, 1] to [far, near];
			pixels[i].z() = f1 * pixels[i].z() + f2;
			// view/camera space
			//normals[i] = (m_n * mesh.vertices[iter->v[i]].get_normal()).normalized();
			normals[i] = (m_n * mesh.normals[iter.n[i]]).normalized();
			points_in_view[i] = util_rd::homo_to_v3(r.m_cam * r.m_model * position_i.homogeneous());
			texcoord[i] = mesh.texcoords[iter.t[i]];
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
						
						double u = (alpha * texcoord[0][0] / w[0] + beta * texcoord[1][0] / w[1] + gamma * texcoord[2][0] / w[2]) * z_view;
						double v = (alpha * texcoord[0][1] / w[0] + beta * texcoord[1][1] / w[1] + gamma * texcoord[2][1] / w[2]) * z_view;
						u = tex::texcoord_wrap(u);
						v = tex::texcoord_wrap(v);
						Eigen::Vector3d n = ((alpha * normals[0] / pv0.z() + beta * normals[1] / pv1.z() + gamma * normals[2] / pv2.z()) * z_view).normalized();
						double denominator = std::sqrt(n.x() * n.x() + n.z() * n.z());
						Eigen::Vector3d t = { (n.x() * n.y()) / denominator, denominator, (n.z() * n.y()) / denominator };
						Eigen::Vector3d b = n.cross(t);
						Eigen::Matrix3d TBN;
						TBN.col(0) = t, TBN.col(1) = b, TBN.col(2) = n;
						double dU = kh * kn * (tex.get_color(u + 1.0 / static_cast<double>(tex.width), v).norm() - tex.get_color(u, v).norm());
						double dV = kh * kn * (tex.get_color(u, v + 1.0 / static_cast<double>(tex.height)).norm() - tex.get_color(u, v).norm());
						Eigen::Vector3d ln = { -dU, -dV, 1.0 };
						p = p + kn * n * tex.get_color(u, v).norm();
						n = (TBN * ln).normalized();
						Eigen::Vector3d e = (-p).normalized();
						//light_pos_view
						// Blinn Phong lighting model
						Eigen::Array3d c;
						auto c_ambient = ka.array() * ambient_light_intensity.array();
						c = c_ambient;
						for (auto l_iter : light_position) {
							auto light_pos_view = util_rd::homo_to_v3(r.m_cam * l_iter.homogeneous());
							double r_2 = (light_pos_view - p).squaredNorm();
							Eigen::Vector3d l = (light_pos_view - p).normalized();
							Eigen::Vector3d h = (e + l).normalized();
							//auto kd = tex.get_color(u, v) / 255;
							Eigen::Vector3d kd = Eigen::Vector3d(148,121.0,92.0) / 255.0 ;
							//double cosine = util_rd::clip(n.dot(l), 0.0, 1.0);
							//double phong_coe = pow(std::max(h.dot(n), 0.0), PHONGEXP);

							auto c_diffuse = kd.array() * (light_intensity / r_2).array() * util_rd::clip(n.dot(l), 0.0, 1.0); //c_r.array() * (c_l * cosine).array();
							auto c_highlight = ks.array() * (light_intensity / r_2).array() * pow(std::max(h.dot(n), 0.0), PHONGEXP);
							c = c + c_diffuse + c_highlight;
						}
						c = c.min(Eigen::Array3d(1, 1, 1)).max(Eigen::Array3d(0, 0, 0));
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

//void bump_shading(mesh::TriangleMesh& mesh, rst::Rasterizer& r, tex::Texture& tex) {
//	std::vector<Eigen::Vector3d> light_position = { { -20, 20, 20 }, { 20, 20, 20 } };
//	Eigen::Vector3d light_intensity = { 500, 500, 500 };
//	Eigen::Vector3d ambient_light_intensity = { 10, 10, 10 };
//	Eigen::Vector3d ka = { 0.005, 0.005, 0.005 };
//	Eigen::Vector3d ks = { 0.7937, 0.7937, 0.7937 };
//
//	double kh = 0.2, kn = 0.1;
//
//	auto m_n = (r.m_uvw * r.m_model.block(0, 0, 3, 3)).inverse().transpose();
//	//auto light_pos_view = util_rd::homo_to_v3(r.m_cam * light_position.homogeneous());
//	//Eigen::Vector3d direc_light = -(r.m_uvw * light).normalized();
//	//std::cout << "light direction: " << std::endl << direc_light << std::endl;
//	auto f1 = (r.near - r.far) / 2;
//	auto f2 = (r.near + r.far) / 2;
//	int num_tri = 0;
//
//	clock_t start_time, end_time;
//	start_time = clock();
//	for (const auto& iter : mesh.triangles) {
//		std::cout << "number of triangle: " << ++num_tri << std::endl;
//		Eigen::Vector3d pixels[3];
//		Eigen::Vector3d normals[3];
//		Eigen::Vector3d points_in_view[3];
//		Eigen::Vector2d texcoord[3];
//		double w[3];
//		for (unsigned int i = 0; i < 3; ++i) {
//			texcoord[i] = mesh.texcoords[iter.t[i]];
//			Eigen::Vector3d n = (m_n * mesh.normals[iter.n[i]]).normalized();
//			double denominator = std::sqrt(n.x() * n.x() + n.z() * n.z());
//			Eigen::Vector3d t = { (n.x() * n.y()) / denominator, denominator, (n.z() * n.y()) / denominator };
//			Eigen::Vector3d b = n.cross(t);
//			Eigen::Matrix3d TBN;
//			TBN.col(0) = t, TBN.col(1) = b, TBN.col(2) = n;
//			auto u = texcoord[i][0];
//			auto v = texcoord[i][1];
//			u = tex::texcoord_wrap(u);
//			v = tex::texcoord_wrap(v);
//			double dU = kh * kn * (tex.get_color(u + 1.0 / static_cast<double>(tex.width), v).norm() - tex.get_color(u, v).norm());
//			double dV = kh * kn * (tex.get_color(u, v + 1.0 / static_cast<double>(tex.height)).norm() - tex.get_color(u, v).norm());
//			Eigen::Vector3d ln = { -dU, -dV, 1.0 };
//			//p = p + kn * n * tex.get_color(u, v).norm();
//			normals[i] = (TBN * ln).normalized();
//
//			Eigen::Vector3d position_i = mesh.vertices[iter.v[i]].get_position();
//			position_i = position_i + kn * n * (tex.get_color(u, v)/255.0).norm();
//			// (pixel.x, pixel.y) in [WIDTH, HEIGHT]
//			auto homo = r.mvp * r.m_model * position_i.homogeneous();
//			w[i] = homo.w();
//			pixels[i] = util_rd::homo_to_v3(homo);
//			// convert z depth value from [-1, 1] to [far, near];
//			pixels[i].z() = f1 * pixels[i].z() + f2;
//			// view/camera space
//			//normals[i] = (m_n * mesh.vertices[iter->v[i]].get_normal()).normalized();
//			points_in_view[i] = util_rd::homo_to_v3(r.m_cam * r.m_model * position_i.homogeneous());
//		}
//		auto p0 = pixels[0];
//		auto p1 = pixels[1];
//		auto p2 = pixels[2];
//		auto pv0 = points_in_view[0];
//		auto pv1 = points_in_view[1];
//		auto pv2 = points_in_view[2];
//		auto x_range = util_rd::get_range_of_three(p0.x(), p1.x(), p2.x());
//		auto y_range = util_rd::get_range_of_three(p0.y(), p1.y(), p2.y());
//
//		auto x_min = util_rd::clip(static_cast<size_t>(x_range.first), static_cast<size_t>(0), r.n_x - 1);
//		auto x_max = util_rd::clip(static_cast<size_t>(x_range.second + 1), static_cast<size_t>(0), r.n_x - 1);
//		auto y_min = util_rd::clip(static_cast<size_t>(y_range.first), static_cast<size_t>(0), r.n_y - 1);
//		auto y_max = util_rd::clip(static_cast<size_t>(y_range.second + 1), static_cast<size_t>(0), r.n_y - 1);
//		rst::Triangle2D tri({ p0.x(), p0.y() }, { p1.x(), p1.y() }, { p2.x(), p2.y() });
//		for (auto x = x_min; x <= x_max; ++x) {
//			for (auto y = y_min; y <= y_max; ++y) {
//				if (tri.is_inside({ static_cast<double>(x), static_cast<double>(y) })) {
//					std::cout << x << " " << y << std::endl;
//					auto barycentric_coordinates = util_rd::compute_barycentric_2D(x, y, { {p0.x(), p0.y()}, {p1.x(), p1.y()},{p2.x(), p2.y()} });
//					auto alpha = std::get<0>(barycentric_coordinates);
//					auto beta = std::get<1>(barycentric_coordinates);
//					auto gamma = std::get<2>(barycentric_coordinates);
//					double z_view = 1.0 / (alpha / w[0] + beta / w[1] + gamma / w[2]);
//					double z_depth_correct = (alpha * p0.z() / w[0] + beta * p1.z() / w[1] + gamma * p2.z() / w[2]) * z_view;
//					if (r.compare_pixel_in_z_buffer(x, y, r.to_z_buffer_value(z_depth_correct))) {
//						double x_view = (alpha * pv0.x() / w[0] + beta * pv1.x() / w[1] + gamma * pv2.x() / w[2]) * z_view;
//						double y_view = (alpha * pv0.y() / w[0] + beta * pv1.y() / w[1] + gamma * pv2.y() / w[2]) * z_view;
//						Eigen::Vector3d p = Eigen::Vector3d(x_view, y_view, z_view);
//
//						double u = (alpha * texcoord[0][0] / w[0] + beta * texcoord[1][0] / w[1] + gamma * texcoord[2][0] / w[2]) * z_view;
//						double v = (alpha * texcoord[0][1] / w[0] + beta * texcoord[1][1] / w[1] + gamma * texcoord[2][1] / w[2]) * z_view;
//						u = tex::texcoord_wrap(u);
//						v = tex::texcoord_wrap(v);
//						Eigen::Vector3d n = ((alpha * normals[0] / pv0.z() + beta * normals[1] / pv1.z() + gamma * normals[2] / pv2.z()) * z_view).normalized();
//						
//						Eigen::Vector3d e = (-p).normalized();
//						//light_pos_view
//						// Blinn Phong lighting model
//						Eigen::Array3d c;
//						auto c_ambient = ka.array() * ambient_light_intensity.array();
//						c = c_ambient;
//						for (auto l_iter : light_position) {
//							auto light_pos_view = util_rd::homo_to_v3(r.m_cam * l_iter.homogeneous());
//							double r_2 = (light_pos_view - p).squaredNorm();
//							Eigen::Vector3d l = (light_pos_view - p).normalized();
//							Eigen::Vector3d h = (e + l).normalized();
//							//auto kd = tex.get_color(u, v) / 255;
//							Eigen::Vector3d kd = Eigen::Vector3d(148, 121.0, 92.0) / 255.0;
//							//double cosine = util_rd::clip(n.dot(l), 0.0, 1.0);
//							//double phong_coe = pow(std::max(h.dot(n), 0.0), PHONGEXP);
//
//							auto c_diffuse = kd.array() * (light_intensity / r_2).array() * util_rd::clip(n.dot(l), 0.0, 1.0); //c_r.array() * (c_l * cosine).array();
//							auto c_highlight = ks.array() * (light_intensity / r_2).array() * pow(std::max(h.dot(n), 0.0), PHONGEXP);
//							c = c + c_diffuse + c_highlight;
//						}
//						c = c.min(Eigen::Array3d(1, 1, 1)).max(Eigen::Array3d(0, 0, 0));
//						r.draw_pixel({ x, y, c * 255 });
//					}
//				}
//
//			}
//		}
//	}
//	end_time = clock();
//	double lasting_time = (static_cast<double>(end_time) - start_time) / CLOCKS_PER_SEC;
//	std::cout << "time of shading: " << lasting_time << "s" << std::endl;
//}