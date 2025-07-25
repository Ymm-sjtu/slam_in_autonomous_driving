//
// Created by xiang on 2022/3/15.
//
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "ch6/lidar_2d_utils.h"
#include "common/io_utils.h"

DEFINE_string(bag_path, "./dataset/sad/2dmapping/floor_1.bag", "数据包路径");

/// 测试从rosbag中读取2d scan并plot的结果

int main(int argc, char** argv) {
    // 初始化日志系统
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    LOG(INFO) << "开始读取bag文件: " << FLAGS_bag_path;

    // 创建ROS bag读取器
    sad::RosbagIO rosbag_io(FLAGS_bag_path);  // 移除fLS::

    int frame_count = 0;
    rosbag_io
        .AddScan2DHandle("/pavo_scan_bottom",                // 订阅的话题名
                         [&frame_count](Scan2d::Ptr scan) {  // Lambda回调函数
                             frame_count++;
                             LOG(INFO) << "处理第 " << frame_count << " 帧扫描数据，点数: " << scan->ranges.size();

                             cv::Mat image;
                             // 提供所有必需的参数
                             sad::Visualize2DScan(scan, SE2(), image, Vec3b(255, 0, 0), 800, 20.0f, SE2());

                             if (!image.empty()) {
                                 LOG(INFO) << "显示图像，尺寸: " << image.cols << "x" << image.rows;
                                 cv::imshow("scan", image);
                                 int key = cv::waitKey(20);
                                 if (key == 27) {  // ESC键退出
                                     LOG(INFO) << "用户按ESC退出";
                                     return false;
                                 }
                             } else {
                                 LOG(WARNING) << "图像为空！";
                             }
                             return true;
                         })
        .Go();  // 开始处理

    LOG(INFO) << "处理完成，总共处理了 " << frame_count << " 帧数据";
    cv::destroyAllWindows();
    return 0;
}