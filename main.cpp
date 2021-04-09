#include <iostream>
#include <direct.h>
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
	std::cout << "main start" << std::endl;
	char pwd[100];
	auto null_arg = _getcwd(pwd, 100);

	std::system("pause");
	// main test
	string file_name = "teapot.obj";
	TriangleMesh mesh;
	read_mesh_from_obj_file(mesh, file_name);
	std::cout << "obj: " << mesh.obj_name << endl;

	std::system("pause");

	rst::Rasterizer r(WINDOW_WIDTH, WINDOW_HEIGHT);
	double theta = 45, n = -0.1, f = -50;
	r.set_view_volume(theta, n, f);
	
	Eigen::Vector3d eye_point = { 0, 2, 10 };
	Eigen::Vector3d gaze = { 0, 0, -1 };
	Eigen::Vector3d view_up = { 0, 1, 0 };

	r.set_camera(eye_point, gaze, view_up);
	r.calculate_matrix();

	cout << "transformation test: " << endl;
	Eigen::Vector3d light = { 0, 0, -1 };
	Eigen::Vector3d c_l = { 1, 1, 1 };
	Eigen::Vector3d c_a = Eigen::Vector3d(0.1, 0.1, 0.1);
	Eigen::Vector3d c_p = { 0.7937, 0.7937, 0.7937 };
	auto m_n = r.m_uvw.inverse().transpose();

	Eigen::Vector3d c_r = { 0, 0, 1 };
	Eigen::Vector3d normal = { -0.05744, 0.97674, -0.214348 };
	auto normal_view = (m_n * normal).normalized();
	cout << "normal in view: " << normal_view << endl;
	//double cosine = util_rd::clip(normal.dot(direc_light), 0.0, 1.0);
	//auto c_diffuse = c_r.array() * (c_l * cosine).array();
	//Matrix4d m_n = m_eye.inverse().transpose();
	//cout << "cosine: " << endl << c_diffuse << endl;
	system("pause");

	// shading
	std::cout << "shading" << std::endl;
	//gouraud_shading(mesh, r, eye_point, M);
	phong_shading(mesh, r);


	// show image
	std::cout << "show image" << endl;
	std::system("pause");
	auto screen = r.canvas_2_screen();
	Mat img(WINDOW_HEIGHT, WINDOW_WIDTH, CV_64FC3, screen.data());
	img.convertTo(img, CV_8UC3, 1.0f);
	cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
	// CV_WINDOW_NORMAL
	cv::namedWindow("test_draw_line");
	cv::imshow("test_draw_line", img);
	cv::waitKey();
	cv::imwrite( mesh.obj_name + ".jpg", img);
	cv::destroyWindow("test_draw_line");

	std::cout << "main end" << endl;
	std::system("pause");
	return 0;
}

