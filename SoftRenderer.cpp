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

const size_t WINDOW_WIDTH = 640;
const size_t WINDOW_HEIGHT = 480;
const double EPSON = 1E-5;


void set_pixel(vector<Vector3d>& canvas, int x, int y, Vector3d color);
int sign_Int(int x, int y);
void draw_line(vector<Vector3d>& canvas, int x0, int y0, int x1, int y1, Vector3d color);

void simple_shade(TriangleMesh& mesh) {
	assert(mesh.obj_name == "triangle");
	mesh.vertices[0].set_color({ 255, 0, 0 });
	mesh.vertices[1].set_color({ 0, 255, 0 });
	mesh.vertices[2].set_color({ 0, 0, 255 });
}

void render_obj(TriangleMesh& mesh) {
	vector<Vector3d> canvas(WINDOW_WIDTH * WINDOW_HEIGHT, Vector3d(0, 0, 0));
	for (auto iter = mesh.triangles.begin(); iter != mesh.triangles.end(); ++iter) {
		auto a = mesh.vertices[iter->v[0]].get_position();
		auto b = mesh.vertices[iter->v[1]].get_position();
		auto c = mesh.vertices[iter->v[2]].get_position();

		auto c_a = mesh.vertices[iter->v[0]].get_color();
		auto c_b = mesh.vertices[iter->v[1]].get_color();
		auto c_c = mesh.vertices[iter->v[2]].get_color();

		auto n = (b - a).cross(c - a);
		auto n_2 = n.squaredNorm();

		int z = 0;
		for (unsigned int i = 90; i < 410; ++i) {
			for (unsigned int j = 90; j < 410; ++j) {
			
				unsigned int k = 0;

				Vector3d p(i, j, k);
				
				auto n_a = (c - b).cross(p - b);
				auto n_b = (a - c).cross(p - c);
				
				double alpha = n.dot(n_a) / n_2;
				double beta = n.dot(n_b) / n_2;
				double gamma = 1.0 - alpha - beta;
				if (alpha >= -EPSON && alpha <= 1+ EPSON && beta >= -EPSON && beta <= 1 + EPSON && gamma >= -EPSON && gamma <= 1 + EPSON) {
					auto color = alpha * c_a + beta * c_b + gamma * c_c;
					//cout << z++ << " " << color(0) << " " << color(1) << " " << color(2) << endl;
					set_pixel(canvas, p(0), p(1), color);
				}
				
			}
		}
		
	}

	Mat img(WINDOW_HEIGHT, WINDOW_WIDTH, CV_64FC3, canvas.data());
	img.convertTo(img, CV_8UC3, 1.0f);
	cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
	// CV_WINDOW_NORMAL
	namedWindow("test_draw_line", CV_WINDOW_NORMAL);
	imshow("test_draw_line", img);
	waitKey();
	destroyWindow("test_draw_line");
}

int main(int argc, char** argv){
	assert(WINDOW_WIDTH > 0 && WINDOW_WIDTH <= 1920 && WINDOW_HEIGHT > 0 && WINDOW_HEIGHT <= 1080);
	cout << "main start!" << endl;
	char pwd[100];
	auto null_arg = _getcwd(pwd, 100);
	system("pause");

	TriangleMesh mesh;
	string file_name = "triangle.obj";
	read_mesh_from_obj_file(mesh, file_name);
	Triangle t = mesh.triangles[0];
	simple_shade(mesh);
	render_obj(mesh);
	cout << "main end!" << endl;
	system("pause");

	return 0;
}


void set_pixel(vector<Vector3d>& canvas, int x, int y, Vector3d color) {
	assert(canvas.size() == WINDOW_WIDTH * WINDOW_HEIGHT && x >= 0 && x < WINDOW_WIDTH&& y >= 0 && y < WINDOW_HEIGHT);
	assert(y * WINDOW_WIDTH + x < WINDOW_WIDTH* WINDOW_HEIGHT);
	canvas[y * WINDOW_WIDTH + x] = color;
}


int sign_Int(int x, int y) { // (x - y) < 0 ?
	if (x == y)     return 0;
	else if (x > y) return -1;
	else            return 1;
}


void draw_line(vector<Vector3d>& canvas, int x0, int y0, int x1, int y1, Vector3d color) {
	// Bresenham algorithm https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	assert(canvas.size() == WINDOW_WIDTH * WINDOW_HEIGHT && x1 >= 0 && x1 < WINDOW_WIDTH&& y1 >= 0
		&& y1 < WINDOW_HEIGHT&& x0 >= 0 && x0 < WINDOW_WIDTH&& y0 >= 0 && y0 < WINDOW_HEIGHT);
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