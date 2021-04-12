#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace tex {
    double texcoord_wrap(double d) {
        double result = fmod(d, 1.0);
        if (result < 0) {
            result += 1;
        }
        return result;
    }

    class Texture {
    public:
        cv::Mat image_data;
        Texture(const std::string& file_path)
        {
            image_data = cv::imread(file_path);
            cv::flip(image_data, image_data, 0);
            cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);

            width = image_data.cols;
            height = image_data.rows;
        }

        size_t width, height;

        Eigen::Vector3d get_color(double u, double v)
        {
            auto u_img = u * width;
            auto v_img = v * height;
            auto color = image_data.at<cv::Vec3b>(v_img, u_img);
            return Eigen::Vector3d(color[0], color[1], color[2]);
        }
    };
}





#endif //RASTERIZER_TEXTURE_H