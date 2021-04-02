#include <iostream>
#include <direct.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

#include "global.h"
#include "rasterizer.hpp"



using namespace std;
using namespace cv;
using namespace Eigen;



int main(void) {
	std::cout << "main start" << std::endl;
	char pwd[100];
	auto null_arg = _getcwd(pwd, 100);
	system("pause");
	
	// main test

	rst::Rasterizer r(WINDOW_WIDTH, WINDOW_HEIGHT);

	rst::Pixel p0(10, 10, { 255, 0, 0 });
	rst::Pixel p1(640, 460, { 0, 0, 255 });
	r.draw_line(p0, p1);

	auto screen = r.canvas_2_screen();
	cout << "show image" << endl;
	system("pause");
	// show image
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

