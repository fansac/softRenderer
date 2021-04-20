#include <iostream>
#include <direct.h>
#include <string>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

#include "global.h"
#include "rasterizer.hpp"
#include "mesh.h"
#include "util.hpp"
#include "shading.hpp"

using namespace std;
using namespace cv;
using namespace Eigen;




int main(void) {
	cout << "main start" << endl;
	system("pause");

	// rasterizer setting
	rst::Rasterizer r(WINDOW_WIDTH, WINDOW_HEIGHT);

	//double angle = 140, scale = 2.5;
	double angle = 180, scale = 0.8;
	//Eigen::Vector3d trans = { 0, 0, 0 };
	r.set_model_transformation(angle, scale);

	double theta = 45, n = -0.1, f = -50;
	r.set_view_volume(theta, n, f);

	Eigen::Vector3d eye_point = { 0, 0, 10 };
	Eigen::Vector3d gaze = { 0, 0, -1 };
	Eigen::Vector3d view_up = { 0, 1, 0 };

	r.set_camera(eye_point, gaze, view_up);
	r.calculate_matrix();

	cout << endl << "read model file" << endl;
	system("pause");

	cout << "test: " << endl;
	Eigen::Vector3d a = { 1, 2, 3 }, b = { 4, 5, 6 }, c = { 7, 8, 9 };
	Eigen::Matrix3d mm;
	mm.col(0) = a; mm.col(1) = b; mm.col(2) = c;
	a.normalize();
	cout << a << endl;
	system("pause");
	
	// read obj spot_triangulated_good unit_sphere
	string file_name = "nb574_model.obj";
	mesh::TriangleMesh mesh;;
	mesh::read_mesh_from_obj_file(mesh, file_name);
	std::cout << "obj: " << mesh.obj_name << endl;
	cout << "AABB: " << "(" <<mesh.x_min << ", " << mesh.x_max << "), (" << mesh.y_min << ", " << mesh.y_max <<
		"), (" << mesh.z_min << ", " << mesh.z_max << ")" << endl;
	
	cout << endl << "read texture fiel" << endl;
	std::system("pause");

	string texture_path = "nb574.jpg";
	tex::Texture tex = tex::Texture(texture_path);
	cv::namedWindow("tex_show");
	cv::imshow("tex_show", tex.image_data);
	cv::waitKey();
	cv::destroyWindow("tex_show");
	

	string normal_texture_path = "nb574_normalmap.jpg";
	tex::Texture normal_tex = tex::Texture(normal_texture_path);
	cv::namedWindow("tex_show");
	cv::imshow("tex_show", normal_tex.image_data);
	cv::waitKey();
	cv::destroyWindow("tex_show");

	// shading
	std::cout << "shading" << std::endl;
	//gouraud_shading(mesh, r);
	//phong_shading(mesh, r, tex);
	bump_shading(mesh, r, tex, normal_tex);
	//phong_shading(mesh, r);
	//phong_shading_shadow(mesh, r);
	//phong_shading_shadow(mesh, r, tex);
	// show image
	std::cout << "show image" << endl;
	std::system("pause");
	auto screen = r.canvas_2_screen();
	Mat img(WINDOW_HEIGHT, WINDOW_WIDTH, CV_64FC3, screen.data());
	img.convertTo(img, CV_8UC3, 1.0f);
	cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
	// CV_WINDOW_NORMAL
	cv::namedWindow("test_rasterizing_" + mesh.obj_name);
	cv::imshow("test_rasterizing_" + mesh.obj_name, img);
	cv::waitKey();
	cv::imwrite( mesh.obj_name + ".jpg", img);
	cv::destroyWindow("test_rasterizing_" + mesh.obj_name);

	std::cout << "main end" << endl;
	std::system("pause");
	return 0;
}

