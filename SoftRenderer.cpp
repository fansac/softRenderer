#include <iostream>
#include <string>
#include <vector>
#include <direct.h>
#include <Eigen\Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include "mesh.h"
#include "util.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const size_t WINDOW_WIDTH = 50;
const size_t WINDOW_HEIGHT = 50;



void set_pixel(vector<Vector3d> &canvas, int x, int y, Vector3d color) {
	assert(canvas.size() == WINDOW_WIDTH * WINDOW_HEIGHT && x >= 0 && x < WINDOW_WIDTH&& y >= 0 && y < WINDOW_HEIGHT);
	assert(y * WINDOW_WIDTH + x < WINDOW_WIDTH* WINDOW_HEIGHT);
	canvas[y * WINDOW_WIDTH + x] = color;
}

// (x - y) < 0 ?
int sign_Int(int x, int y) {
	if (x == y)     return 0;
	else if (x > y) return -1;
	else            return 1;

}


void draw_line(vector<Vector3d>& canvas, int x0, int y0, int x1, int y1, Vector3d color) {
	// Bresenham algorithm https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	assert(canvas.size() == WINDOW_WIDTH * WINDOW_HEIGHT && x1 >= 0 && x1 < WINDOW_WIDTH&& y1 >= 0 
		&& y1 < WINDOW_HEIGHT && x0 >= 0 && x0 < WINDOW_WIDTH&& y0 >= 0 && y0 < WINDOW_HEIGHT);
	int sign_x = sign_Int(x0, x1);
	int sign_y = sign_Int(y0, y1);
	if (x0 == x1) {
		for (int y = y0; y != y1; y += sign_y) set_pixel(canvas, x0, y, color);
		set_pixel(canvas, x1, y1, color);
		return;
	}
	if (y0 == y1) {
		for (int x = x0; x != x1; x += sign_x) set_pixel(canvas, x, y0, color);
		set_pixel(canvas, x1, y1, color);
		return;
	}

	int d_x = sign_x * (x1 - x0);
	int d_y = sign_y * (y0 - y1);
	int err = d_x + d_y;
	int x = x0, y = y0;
	do {
		set_pixel(canvas, x, y, color);
		int e2 = 2 * err;
		if (e2 >= d_y) {
			err += d_y;
			x += sign_x;
		}
		if (e2 <= d_x) {
			err += d_x;
			y += sign_y;
		}
	} while (!(x == x1 && y == y1));
}


int implicit_line_equation(int x, int y, int x0, int y0, int x1, int y1) {
	int result = (y0 - y1) * x + (x1 - x0) * y + x0 * y1 - x1 * y0;
	cout << "result: " << result << endl;
	return result;
}


void draw_line_mid_point(vector<Vector3d>& canvas, int x0, int y0, int x1, int y1, Vector3d color) {
	// mid point algorithm
	assert(canvas.size() == WINDOW_WIDTH * WINDOW_HEIGHT && x1 >= 0 && x1 < WINDOW_WIDTH&& y1 >= 0 && y1 < WINDOW_HEIGHT
	&& x0 >= 0 && x0 < WINDOW_WIDTH&& y0 >= 0 && y0 < WINDOW_HEIGHT);
	
	if (x1 == x0) {
		if (y0 < y1) for (int y = y0; y <= y1; ++y) set_pixel(canvas, x0, y, color);
		else for (int y = y0; y >= y1; --y) set_pixel(canvas, x0, y, color);
		return;
	}

	if (x1 < x0) {
		swap(x0, x1);
		swap(y0, y1);
	}
	int d_y = x1 - x0, d_x = y0 - y1;
	double m = (static_cast<double>(y1) - y0) / (static_cast<double>(x1) - x0);
	if (m <= 1 && m >= 0) {
		int y = y0;
		int d = implicit_line_equation(x0 + 1, y0 + 0.5, x0, y0, x1, y1);
		for (int x = x0; x <= x1 && y <= y1; ++x) {
			set_pixel(canvas, x, y, color);
			if (d < 0) {
				++y;
				d += d_x + d_y;
			}
			else d += d_x;
		}
	}
	else if (m >= -1 && m < 0) {
		int y = y0;
		int d = implicit_line_equation(x0 + 1, y0 - 0.5, x0, y0, x1, y1);
		for (int x = x0; x <= x1 && y >= y1; ++x) {
			set_pixel(canvas, x, y, color);
			if (d > 0) {
				--y;
				d += d_x - d_y;
			}
			else d += d_x;
		}
	}
	else if (m > 1) {
		int x = x0;
		int d = implicit_line_equation(x0 + 0.5, y0 + 1, x0, y0, x1, y1);
		for (int y = y0; y <= y1; ++y) {
			set_pixel(canvas, x, y, color);
			if (d > 0) {
				++x;
				d += d_x + d_y;
			}
			else d += d_y;
			
		}
	}
	else {
		int x = x0;
		int d = implicit_line_equation(x0 + 0.5, y0 - 1, x0, y0, x1, y1);
		for (int y = y0; y >= y1; --y) {
			set_pixel(canvas, x, y, color);
			if (d < 0) {
				++x;
				d += d_x - d_y;
			}
			else d -= d_y;
		}
	}
	
}

void test_draw_line() {
	vector<Vector3d> canvas(WINDOW_WIDTH * WINDOW_HEIGHT, Vector3d(0, 0, 0));
	// horizontal line
	//draw_line(canvas, 0, 0, 49, 0, Vector3d(0, 0, 255));
	//draw_line(canvas, 0, 49, 49, 49, Vector3d(0, 0, 255));
	//draw_line(canvas, 0, 25, 49, 25, Vector3d(0, 0, 255));
	//draw_line(canvas, 20, 20, 20, 20, Vector3d(0, 0, 255));
	//draw_line(canvas, 30, 30, 40, 30, Vector3d(0, 0, 255));

	//draw_line(canvas, 49, 0, 0, 0, Vector3d(0, 0, 255));
	//draw_line(canvas, 49, 49, 0, 49, Vector3d(0, 0, 255));
	//draw_line(canvas, 0, 25, 49, 25, Vector3d(0, 0, 255));
	//draw_line(canvas, 20, 20, 20, 20, Vector3d(0, 0, 255));
	//draw_line(canvas, 40, 30, 30, 30, Vector3d(0, 0, 255));

	// vertical line
	/*draw_line(canvas, 0, 0, 0, 49, Vector3d(255, 0, 0));
	draw_line(canvas, 49, 0, 49, 49, Vector3d(255, 0, 0));
	draw_line(canvas, 25, 0, 25, 49, Vector3d(255, 0, 0));
	draw_line(canvas, 20, 20, 20, 20, Vector3d(255, 0, 0));
	draw_line(canvas, 30, 30, 30, 40, Vector3d(255, 0, 0));*/

	//draw_line(canvas, 0, 49, 0, 0, Vector3d(255, 0, 0));
	//draw_line(canvas, 49, 49, 49, 0, Vector3d(255, 0, 0));
	//draw_line(canvas, 25, 49, 25, 0, Vector3d(255, 0, 0));
	//draw_line(canvas, 20, 20, 20, 20, Vector3d(255, 0, 0));
	//draw_line(canvas, 30, 40, 30, 30, Vector3d(255, 0, 0));
	
	// arbitrary
	//draw_line(canvas, 20, 20, 40, 1, Vector3d(0, 0, 255));
	//draw_line(canvas, 20, 20, 40, 20, Vector3d(0, 0, 255));
	//draw_line(canvas, 20, 20, 40, 40, Vector3d(0, 0, 255));
	//draw_line(canvas, 20, 20, 40, 10, Vector3d(0, 0, 255));
	//draw_line(canvas, 20, 20, 40, 49, Vector3d(0, 0, 255));
	//draw_line(canvas, 20, 20, 40, 30, Vector3d(0, 0, 255));

	//draw_line(canvas, 20, 20, 1, 1, Vector3d(255, 0, 0));
	//draw_line(canvas, 20, 20, 1, 20, Vector3d(255, 0, 0));
	//draw_line(canvas, 20, 20, 1, 40, Vector3d(255, 0, 0));
	//draw_line(canvas, 20, 20, 1, 10, Vector3d(255, 0, 0));
	//draw_line(canvas, 20, 20, 1, 49, Vector3d(255, 0, 0));
	//draw_line(canvas, 20, 20, 1, 30, Vector3d(255, 0, 0));


	//draw_line(canvas, 20, 20, 1, 1, Vector3d(0, 255, 0));
	//draw_line(canvas, 20, 20, 10, 1, Vector3d(0, 255, 0));
	//draw_line(canvas, 20, 20, 20, 1, Vector3d(0, 255, 0));
	//draw_line(canvas, 20, 20, 30, 1, Vector3d(0, 255, 0));
	//draw_line(canvas, 20, 20, 40, 1, Vector3d(0, 255, 0));
	//draw_line(canvas, 20, 20, 49, 1, Vector3d(0, 255, 0));

	draw_line(canvas, 20, 20, 1, 49, Vector3d(0, 255, 255));
	draw_line(canvas, 20, 20, 10, 49, Vector3d(0, 255, 255));
	draw_line(canvas, 20, 20, 20, 49, Vector3d(0, 255, 255));
	draw_line(canvas, 20, 20, 30, 49, Vector3d(0, 255, 255));
	draw_line(canvas, 20, 20, 40, 49, Vector3d(0, 255, 255));
	draw_line(canvas, 20, 20, 49, 49, Vector3d(0, 255, 255));
	Mat img(WINDOW_HEIGHT, WINDOW_WIDTH, CV_64FC3, canvas.data());
	//cout << "m = " << endl << " " << img << endl;
	namedWindow("test_draw_line", CV_WINDOW_KEEPRATIO);
	imshow("test_draw_line", img);
	waitKey();
}

int main(int argc, char** argv){
	assert(WINDOW_WIDTH > 0 && WINDOW_WIDTH <= 1920 && WINDOW_HEIGHT > 0 && WINDOW_HEIGHT <= 1080);
	cout << "main start!" << endl;
	char pwd[100];
	auto null_arg = _getcwd(pwd, 100);
	system("pause");

	TriangleMesh mesh;
	string file_name = "cube.obj";
	read_mesh_from_obj_file(mesh, file_name);
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
	cout << "main end!" << endl;
	return 0;
}

