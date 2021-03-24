#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <iostream>
#include <string>
#include <vector>
#include <Eigen\Dense>
#include "util.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int WINDOW_WIDTH = 50;
const int WINDOW_HEIGHT = 50;



void set_pixel(vector<Vector3d> &canvas, int x, int y, Vector3d color) {
	assert(canvas.size() == WINDOW_WIDTH * WINDOW_HEIGHT && x >= 0 && x < WINDOW_WIDTH&& y >= 0 && y < WINDOW_HEIGHT);
	canvas[y * WINDOW_WIDTH + x] = color;
}

void draw_line(vector<Vector3d>& canvas, int x1, int y1, int x2, int y2, Vector3d color) {
	assert(canvas.size() == WINDOW_WIDTH * WINDOW_HEIGHT && x1 >= 0 && x1 < WINDOW_WIDTH&& y1 >= 0 && y1 < WINDOW_HEIGHT
	&& x2 >= 0 && x2 < WINDOW_WIDTH&& y2 >= 0 && y2 < WINDOW_HEIGHT);
	if (x1 == x2) {
		set_pixel(canvas, x1, x2, color);
	}
	if (x1 > x2) {
		swap(x1, x2);
		swap(y1, y2);
	}
	for (int x = x1; x <= x2; ++x) {
		int y = int(1.0 * (y2 - y1) * (x - x1) / (x2 - x1)) + y1;
		set_pixel(canvas, x, y, color);
	}
}


int main(int argc, char** argv){
	assert(WINDOW_WIDTH > 0 && WINDOW_WIDTH <= 1920 && WINDOW_HEIGHT > 0 && WINDOW_HEIGHT <= 1080);

	//string file_path = "E:\\softRenderer\\SoftRenderer\\fire1.png";
	//cout << file_path << endl;
	vector<Vector3d> canvas(WINDOW_WIDTH * WINDOW_HEIGHT, Vector3d(0, 0, 0));
	draw_line(canvas, 14, 48, 1, 1, Vector3d(0, 0, 255));
	draw_line(canvas, 48, 14, 1, 1, Vector3d(0, 255, 0));
	draw_line(canvas, 48, 48, 1, 1, Vector3d(255, 0, 0));
	//set_pixel(canvas, 1, 1, Vector3d(0, 0, 255));
	Mat img(WINDOW_HEIGHT, WINDOW_WIDTH, CV_64FC3, canvas.data());
	//cout << "m = " << endl << " " << img << endl;
	namedWindow("Renderer", CV_WINDOW_KEEPRATIO);
	imshow("Renderer", img);
	waitKey();
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
	
	return 0;
}