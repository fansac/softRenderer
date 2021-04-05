#include <iostream>
#include <direct.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

#include "global.h"
#include "rasterizer.hpp"
#include "mesh.h"
#include "util.hpp"


using namespace std;
using namespace cv;
using namespace Eigen;



int main(void) {
	std::cout << "main start" << std::endl;
	char pwd[100];
	auto null_arg = _getcwd(pwd, 100);
	system("pause");
	
	// main test
	string file_name = "triangle.obj";
	TriangleMesh mesh;
	read_mesh_from_obj_file(mesh, file_name);
	cout << "obj: " << mesh.obj_name << endl;

	system("pause");

	rst::Rasterizer r(WINDOW_WIDTH, WINDOW_HEIGHT);
	double theta = 45, n = -0.1, f = -50;
	r.set_view_volume(theta, n, f);
	
	Eigen::Vector3d eye_point = { 0, 0, 5 };
	Eigen::Vector3d gaze = { 0, 0, -1 };
	Eigen::Vector3d view_up = { 0, 1, 0 };

	r.set_camera(eye_point, gaze, view_up);
	r.calculate_matrix();


	for (auto iter = mesh.triangles.begin(); iter != mesh.triangles.end(); ++iter) {
		auto a = mesh.vertices[iter->v[0]].get_position();
		auto b = mesh.vertices[iter->v[1]].get_position();
		auto c = mesh.vertices[iter->v[2]].get_position();

		auto c_a = mesh.colors[iter->c[0]];
		auto c_b = mesh.colors[iter->c[1]];
		auto c_c = mesh.colors[iter->c[2]];

		auto M = r.get_M();
		auto p_a = util_rd::homo_to_v2(M * a.homogeneous());
		auto p_b = util_rd::homo_to_v2(M * b.homogeneous());
		auto p_c = util_rd::homo_to_v2(M * c.homogeneous());

		rst::Pixel p0(p_a[0], p_a[1], c_a);
		rst::Pixel p1(p_b[0], p_b[1], c_b);
		rst::Pixel p2(p_c[0], p_c[1], c_c);
		
		r.draw_triangle(p0, p1, p2);
	}

	system("pause");

	// show image
	cout << "show image" << endl;
	auto screen = r.canvas_2_screen();
	Mat img(WINDOW_HEIGHT, WINDOW_WIDTH, CV_64FC3, screen.data());
	img.convertTo(img, CV_8UC3, 1.0f);
	cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
	// CV_WINDOW_NORMAL
	namedWindow("test_draw_line");
	imshow("test_draw_line", img);
	waitKey();
	destroyWindow("test_draw_line");

	cout << "main end" << endl;
	system("pause");
	return 0;
}

