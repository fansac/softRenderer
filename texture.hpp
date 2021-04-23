#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace tex {
    enum texType {
        texture = 0,
        normal = 1,
        bump = 2
    };
    inline double texcoord_wrap(double d) {
        double result = fmod(d, 1.0);
        if (result < 0) {
            result += 1;
        }
        return result;
    }

    class Texture {
    public:
        cv::Mat image_data;
        texType type = texType::texture;
        Texture(const std::string& file_path) {
            image_data = cv::imread(file_path);
            cv::flip(image_data, image_data, 0);
            cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);

            width = image_data.cols;
            height = image_data.rows;
        }

        Texture(const std::string& file_path, texType t): type(t) {
            
            
            switch (type)
            {
            case tex::normal:
                image_data = cv::imread(file_path);
                cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
                break;
            case tex::bump:
                image_data = cv::imread(file_path, cv::IMREAD_GRAYSCALE);
                break;
            default:
                image_data = cv::imread(file_path);
                cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
                break;
            }
            
            cv::flip(image_data, image_data, 0);
            width = image_data.cols;
            height = image_data.rows;
        }

        size_t width, height;

        Eigen::Vector3d get_color(double u, double v) {
            auto u_img = u * width;
            auto v_img = v * height;
            auto color = image_data.at<cv::Vec3b>(v_img, u_img);
            return Eigen::Vector3d(color[0], color[1], color[2]);
        }

        uchar get_height(double u, double v) {
            assert(type == texType::bump);
            auto u_img = u * width;
            auto v_img = v * height;
            auto color = image_data.at<uchar>(v_img, u_img);
            return color;
        }

        void bump2normal(double scale) {
            assert(type == texType::bump);
            int h = static_cast<int>(height);
            int w = static_cast<int>(width);
            std::vector<Eigen::Vector3d>normal_map(height * width, {0, 0, 0});
            // Foundations of Game Engine Development Volumes 2 Rendering Listing7.5

            double s = scale;
            for (int y = 1; y < h - 1; ++y) {
                int ym1 = (y - 1), yp1 = (y + 1);
                uchar *center_row_ptr = image_data.ptr<uchar>(y);              
                uchar *upper_row_ptr = image_data.ptr<uchar>(ym1);
                uchar *lower_row_ptr = image_data.ptr<uchar>(yp1);

                for (int x = 1; x < w - 1; ++x) {
                    int xm1 = (x - 1), xp1 = (x + 1);
                    // Calculate slopes;
                    double dx = s * (center_row_ptr[xp1] - center_row_ptr[xm1]) * 0.5;
                    double dy = s * (lower_row_ptr[x] - upper_row_ptr[x]) * 0.5;

                    // Normalize and clamp
                    double nz = 1.0 / sqrt(dx * dx + dy * dy + 1.0);
                    double nx = std::min(std::max(-dx * nz, -1.0), 1.0);
                    double ny = std::min(std::max(-dy * nz, -1.0), 1.0);
                    normal_map[y * width + x] << ((nx + 1.0) * 127.5), ((ny + 1.0) * 127.5), ((nz + 1.0) * 127.5);
                }
            }

            for (int x = 0; x < w - 1; ++x) {
                double dx = s * (image_data.at<uchar>(x + 1) - image_data.at<uchar>(x));
                double dy = s * (image_data.at<uchar>(w + x) - image_data.at<uchar>(x));
                double nz = 1.0 / sqrt(dx * dx + dy * dy + 1.0);
                double nx = std::min(std::max(-dx * nz, -1.0), 1.0);
                double ny = std::min(std::max(-dy * nz, -1.0), 1.0);
                normal_map[x] << ((nx + 1.0) * 127.5), ((ny + 1.0) * 127.5), ((nz + 1.0) * 127.5);
            }

            for (int y = 0; y < h - 1; ++y) {
                double dx = s * (image_data.at<uchar>((y + 1) * w - 1) - image_data.at<uchar>((y + 1) * w - 2));
                double dy = s * (image_data.at<uchar>((y + 2) * w - 1) - image_data.at<uchar>((y + 1) * w - 1));
                double nz = 1.0 / sqrt(dx * dx + dy * dy + 1.0);
                double nx = std::min(std::max(-dx * nz, -1.0), 1.0);
                double ny = std::min(std::max(-dy * nz, -1.0), 1.0);
                normal_map[y * width + width - 1] << ((nx + 1.0) * 127.5), ((ny + 1.0) * 127.5), ((nz + 1.0) * 127.5);
            }

            for (int x = 1; x < w; ++x) {
                double dx = s * (image_data.at<uchar>((h - 1) * w + x) - image_data.at<uchar>((h - 1) * w + x - 1));
                double dy = s * (image_data.at<uchar>((h - 1) * w + x) - image_data.at<uchar>((h - 2) * w + x));
                double nz = 1.0 / sqrt(dx * dx + dy * dy + 1.0);
                double nx = std::min(std::max(-dx * nz, -1.0), 1.0);
                double ny = std::min(std::max(-dy * nz, -1.0), 1.0);
                normal_map[(height  - 1) * width + x] << ((nx + 1.0) * 127.5), ((ny + 1.0) * 127.5), ((nz + 1.0) * 127.5);
            }

            for (int y = 1; y < h; ++y) {
                double dx = s * (image_data.at<uchar>(y * w + 1) - image_data.at<uchar>(y * w));
                double dy = s * (image_data.at<uchar>(y * w) - image_data.at<uchar>((y - 1) * w));
                double nz = 1.0 / sqrt(dx * dx + dy * dy + 1.0);
                double nx = std::min(std::max(-dx * nz, -1.0), 1.0);
                double ny = std::min(std::max(-dy * nz, -1.0), 1.0);
                normal_map[y * width] << ((nx + 1.0) * 127.5), ((ny + 1.0) * 127.5), ((nz + 1.0) * 127.5);
            }
            image_data = cv::Mat(height, width, CV_64FC3, normal_map.data());
            image_data.convertTo(image_data, CV_8UC3, 1.0f);
            //cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
            type = texType::normal;
            
        }

        void show(void) {
            cv::namedWindow("tex_show");
            cv::imshow("tex_show", this->image_data);
            cv::waitKey();
            cv::destroyWindow("tex_show");
        }
    };
}





#endif //RASTERIZER_TEXTURE_H