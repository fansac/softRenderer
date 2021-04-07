#include <Eigen/Dense>
#include <vector>
#include "mesh.h"
#include "rasterizer.hpp"
#include "util.hpp"

//void open_image(void){
		//string file_path = "E:\\softRenderer\\SoftRenderer\\fire1.png";
	//cout << file_path << endl;
	//vector<Vector3d> canvas(WINDOW_WIDTH * WINDOW_HEIGHT, Vector3d(0, 0, 0));
	//draw_line(canvas, 20, 20, 49, 49, Vector3d(0, 255, 255));
	//set_pixel(canvas, 1, 1, Vector3d(0, 0, 255));
	//Mat img(WINDOW_HEIGHT, WINDOW_WIDTH, CV_64FC3, canvas.data());
	////cout << "m = " << endl << " " << img << endl;
	//namedWindow("Renderer", CV_WINDOW_KEEPRATIO);
	//imshow("Renderer", img);
	//waitKey();
	//Mat img = imread(file_path);
	//if (img.empty())
	//{
	//	fprintf(stderr, "Can not load image %s\n", file_path.c_str());
	//	return -1;
	//}

	//if (!img.data) // 检查是否正确载入图像
	//	return -1;

	//vector<cv::Vec<float, 3>> b = { {255., 0, 0}, {0, 255., 0}, {0, 0, 255.}, {255., 255., 255.} };
	////vector<vector3d> b = { vector3d(255, 0, 0), vector3d(255, 0, 0), vector3d(255, 0, 0), vector3d(255, 0, 0) };
	//cout << "test = " << endl << b.data() << endl << endl;
	//Mat img(2, 2, CV_32FC3, b.data());
	////memcpy(img.data, b.data(), b.size()*sizeof(uchar));
	//cout << "m = " << endl << " " << img << endl;
	//cv::namedWindow("Window", CV_WINDOW_NORMAL); //创建窗口g
	//cv::imshow("Window", img); //显示图像
	//cv::waitKey();
	//system("pause");
//}

//void vertex_shading() {
	/*string file_name = "cube.obj";
	TriangleMesh mesh;
	read_mesh_from_obj_file(mesh, file_name);
	std::cout << "obj: " << mesh.obj_name << endl;

	std::system("pause");

	rst::Rasterizer r(WINDOW_WIDTH, WINDOW_HEIGHT);
	double theta = 45, n = -0.1, f = -50;
	r.set_view_volume(theta, n, f);

	Eigen::Vector3d eye_point = { 3, 3, 2 };
	Eigen::Vector3d gaze = { -1, -1, -1 };
	Eigen::Vector3d view_up = { 0, 1, 0 };

	r.set_camera(eye_point, gaze, view_up);
	r.calculate_matrix();

	Eigen::Vector3d light = { -1, -1, -1 };
	Eigen::Vector3d c_l = { 1.0, 1.0, 1.0 };
	Eigen::Vector3d c_a = Eigen::Vector3d(10, 10, 10) / 255;
	Eigen::Vector3d c_p = { 0.7937, 0.7937, 0.7937 };
	auto direc_light = -(light.normalized());
	cout << "light: " << direc_light << endl;
	auto M = r.get_M();
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

		r.draw_triangle(pixels[0], pixels[1], pixels[2]);
	}*/
//}

void gouraud_shading(TriangleMesh& mesh, rst::Rasterizer& r, const Eigen::Vector3d eye_point, const Eigen::Matrix4d M);