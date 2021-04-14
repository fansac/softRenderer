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

	cout << "test: " << endl;
	vector<double> v = { 1.0 , 2.0, 3.0, 4.0 };
	double xmin = v[0], xmax = v[1];
	for (auto iter : v) {
		util_rd::update_min_max(iter, xmin, xmax);
	}
	cout << "min " << xmin << " " << "max " << xmax << endl;

	system("pause");

	// read obj spot_triangulated_good unit_sphere
	string file_name = "spot_triangulated_good.obj";
	mesh::TriangleMesh mesh;;
	mesh::read_mesh_from_obj_file(mesh, file_name);
	std::cout << "obj: " << mesh.obj_name << endl;
	cout << "AABB: " << "(" <<mesh.x_min << ", " << mesh.x_max << "), (" << mesh.y_min << ", " << mesh.y_max <<
		"), (" << mesh.z_min << ", " << mesh.z_max << ")" << endl;
	std::system("pause");

	string texture_path = "spot_texture.png";
	tex::Texture tex = tex::Texture(texture_path);
	cv::namedWindow("tex_show");
	cv::imshow("tex_show", tex.image_data);
	cv::waitKey();
	cv::destroyWindow("tex_show");

	rst::Rasterizer r(WINDOW_WIDTH, WINDOW_HEIGHT);

	double angle = 140, scale = 2.5;
	//double angle = 0, scale = 1.0;
	r.set_model_transformation(angle, scale);

	double theta = 45, n = -0.1, f = -50;
	r.set_view_volume(theta, n, f);
	
	Eigen::Vector3d eye_point = { 0, 0, 10 };
	Eigen::Vector3d gaze = { 0, 0, -1 };
	Eigen::Vector3d view_up = { 0, 1, 0 };

	r.set_camera(eye_point, gaze, view_up);
	r.calculate_matrix();

	

	// shading
	std::cout << "shading" << std::endl;
	//gouraud_shading(mesh, r);
	phong_shading(mesh, r, tex);
	//phong_shading(mesh, r);

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

