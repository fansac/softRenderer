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

			cout << "vertex color: " << iter->v[i] << endl;
			cout << c << endl;
			rst::Pixel pixel_i(pos_3_i[0], pos_3_i[1], r.to_z_buffer_value(pos_3_i[2]), c * 255.0);
			pixels.push_back(pixel_i);
		}
		r.draw_triangle(pixels);
	}
}