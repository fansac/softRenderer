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
	string file_name = "cube.obj";
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

	auto M = r.get_M();

	// shading
	std::cout << "shading" << std::endl;
	gouraud_shading(mesh, r, eye_point, M);
	

	// show image
	std::cout << "show image" << endl;
	std::system("pause");
	auto screen = r.canvas_2_screen();
	Mat img(WINDOW_HEIGHT, WINDOW_WIDTH, CV_64FC3, screen.data());
	img.convertTo(img, CV_8UC3, 1.0f);
	cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
	// CV_WINDOW_NORMAL
	cv::namedWindow("test_draw_line", CV_WINDOW_NORMAL);
	cv::imshow("test_draw_line", img);
	cv::waitKey();
	cv::imwrite( mesh.obj_name + ".jpg", img);
	cv::destroyWindow("test_draw_line");

	std::cout << "main end" << endl;
	std::system("pause");
	return 0;
}

