#include "util.h"

#include <string>
#include <fstream>

bool is_file_exists_ifstream(const std::string name) {
	std::ifstream f(name.c_str());
	return f.good();
}

std::string find_file_name(const std::string file_path) {
	size_t start = file_path.find_last_of('/') + 1;
	size_t end = file_path.find_last_of('.');
	return file_path.substr(start, end);
}


void test_draw_line() {
	//vector<Vector3d> canvas(WINDOW_WIDTH * WINDOW_HEIGHT, Vector3d(0, 0, 0));
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

	//draw_line(canvas, 20, 20, 1, 49, Vector3d(0, 255, 255));
	//draw_line(canvas, 20, 20, 10, 49, Vector3d(0, 255, 255));
	//draw_line(canvas, 20, 20, 20, 49, Vector3d(0, 255, 255));
	//draw_line(canvas, 20, 20, 30, 49, Vector3d(0, 255, 255));
	//draw_line(canvas, 20, 20, 40, 49, Vector3d(0, 255, 255));
	//draw_line(canvas, 20, 20, 49, 49, Vector3d(0, 255, 255));
	//Mat img(WINDOW_HEIGHT, WINDOW_WIDTH, CV_64FC3, canvas.data());
	//namedWindow("test_draw_line", CV_WINDOW_KEEPRATIO);
	//imshow("test_draw_line", img);
	//waitKey();
}

void open_image(void){
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
}

