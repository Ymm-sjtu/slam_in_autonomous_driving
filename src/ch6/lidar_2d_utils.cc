//
// Created by xiang on 2022/3/15.
//

#include "ch6/lidar_2d_utils.h"
#include <opencv2/imgproc.hpp>

namespace sad {

void Visualize2DScan(Scan2d::Ptr scan, const SE2& pose, cv::Mat& image, const Vec3b& color, int image_size,
                     float resolution, const SE2& pose_submap) {
    // 如果图像为空，创建白色背景的正方形图像
    if (image.data == nullptr) {
        image = cv::Mat(image_size, image_size, CV_8UC3, cv::Vec3b(255, 255, 255));
    }

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        // 1. 有效性检查
        if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max) {
            continue;
        }

        // 2. 计算真实角度和笛卡尔坐标
        double real_angle = scan->angle_min + i * scan->angle_increment;
        double x = scan->ranges[i] * std::cos(real_angle);
        double y = scan->ranges[i] * std::sin(real_angle);

        // 3. 角度范围过滤（去除前后30度）
        if (real_angle < scan->angle_min + 30 * M_PI / 180.0 || real_angle > scan->angle_max - 30 * M_PI / 180.0) {
            continue;
        }

        // 4. 坐标变换
        Vec2d psubmap = pose_submap.inverse() * (pose * Vec2d(x, y));

        // 5. 转换到图像坐标
        int image_x = int(psubmap[0] * resolution + image_size / 2.0);
        int image_y = int(psubmap[1] * resolution + image_size / 2.0);

        // 6. 边界检查并绘制
        if (image_x >= 0 && image_x < image.cols && image_y >= 0 && image_y < image.rows) {
            image.at<cv::Vec3b>(image_y, image_x) = cv::Vec3b(color[0], color[1], color[2]);
        }
    }

    // 同时画出pose自身所在位置
    Vec2d pose_in_image =
        pose_submap.inverse() * (pose.translation()) * double(resolution) + Vec2d(image_size / 2, image_size / 2);
    cv::circle(image, cv::Point2f(pose_in_image[0], pose_in_image[1]), 5, cv::Scalar(color[0], color[1], color[2]), 2);
}

}  // namespace sad