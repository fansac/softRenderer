#include <Eigen/Dense>
#include <vector>
#include "mesh.h"
#include "rasterizer.hpp"
#include "util.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
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



void gouraud_shading(mesh::TriangleMesh& mesh, rst::Rasterizer& r);
void phong_shading(mesh::TriangleMesh& mesh, rst::Rasterizer& r);
void phong_shading(mesh::TriangleMesh& mesh, rst::Rasterizer& r, tex::Texture& tex);
//void bump_shading(mesh::TriangleMesh& mesh, rst::Rasterizer& r, tex::Texture& tex, tex::Texture& normal_map);

void phong_shading_shadow(mesh::TriangleMesh& mesh, rst::Rasterizer& r);
void phong_shading_shadow(mesh::TriangleMesh& mesh, rst::Rasterizer& r, tex::Texture& tex);
void normal_map_shading(mesh::TriangleMesh& mesh, rst::Rasterizer& r, tex::Texture& tex, tex::Texture& normal_map);
void displacement_map_shading(mesh::TriangleMesh& mesh, rst::Rasterizer& r, tex::Texture& tex, tex::Texture& normal_map);