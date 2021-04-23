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

#define RENDER

using namespace std;
using namespace cv;
using namespace Eigen;




int main(void) {
	std::cout << "main start" << endl;
	system("pause");

#ifdef RENDER
	// rasterizer setting
	rst::Rasterizer r(WINDOW_WIDTH, WINDOW_HEIGHT);

	//double angle = 140, scale = 2.5;
	double angle = -30, scale = 1.2;
	//Eigen::Vector3d trans = { 401.5, 351, 359.56 };
	//Eigen::Vector3d trans = { 0, 0, 0 };
	r.set_model_transformation(angle, scale);

	double theta = 45, n = -0.1, f = -50;
	r.set_view_volume(theta, n, f);

	Eigen::Vector3d eye_point = { 0, 0, 5 };
	Eigen::Vector3d gaze = { 0, 0, -1 };
	Eigen::Vector3d view_up = { 0, 1, 0 };

	r.set_camera(eye_point, gaze, view_up);
	r.calculate_matrix();

	cout << endl << "read model file" << endl;
	// read obj spot_triangulated_good unit_sphere
	string file_name = "./model/sphere/sphere_highres.obj";

	cout << boolalpha << util_rd::is_file_exists_ifstream(file_name) << endl;
	system("pause");
	mesh::TriangleMesh mesh;;
	mesh::read_mesh_from_obj_file(mesh, file_name);
	std::cout << "obj: " << mesh.obj_name << endl;
	cout << "AABB: " << "(" <<mesh.x_min << ", " << mesh.x_max << "), (" << mesh.y_min << ", " << mesh.y_max <<
		"), (" << mesh.z_min << ", " << mesh.z_max << ")" << endl;
	cout << endl << "read texture file" << endl;
	std::system("pause");
	//earthmap1k.jpg ; Earth_Sphere_Material.png
	string texture_path = "./model/earth/Earth_Sphere_Material.png";
	tex::Texture tex = tex::Texture(texture_path);
	tex.show();
	

	//string normal_texture_path = "earthbump1k.jpg"; earth_normalmap.png
	//tex::Texture normal_tex = tex::Texture(normal_texture_path, tex::texType::bump);
	// tex::texType::bump
	string normal_texture_path = "./model/earth/earth_normalmap.png";
	tex::Texture normal_tex = tex::Texture(normal_texture_path, tex::texType::normal);
	normal_tex.show();

	// shading
	std::cout << "shading" << std::endl;
	//gouraud_shading(mesh, r);
	//phong_shading(mesh, r, tex);
	normal_map_shading(mesh, r, tex, normal_tex);
	//displacement_map_shading(mesh, r, tex, normal_tex);
	//bump_shading(mesh, r, tex, normal_tex);
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

#endif // RENDER
	std::cout << "main end" << endl;
	std::system("pause");
	return 0;
}

