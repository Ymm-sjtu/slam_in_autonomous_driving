//
// Created by xiang on 2022/3/23.
//

#ifndef SLAM_IN_AUTO_DRIVING_MAPPING_2D_H
#define SLAM_IN_AUTO_DRIVING_MAPPING_2D_H

#include "ch6/frame.h"
#include "common/eigen_types.h"
#include "common/lidar_utils.h"

#include <memory>
#include <opencv2/core.hpp>

namespace sad {

class Submap;
class LoopClosing;

/**
 * 2D 激光建图的主要类
 */
class Mapping2D {
   public:
    bool Init(bool with_loop_closing = true);

    /// 单回波的scan
    bool ProcessScan(Scan2d::Ptr scan);

    /// 多回波的scan
    /// 暂时没用到
    bool ProcessScan(MultiScan2d::Ptr scan);

    /**
     * 显示全局地图
     * @param max_size 全局地图最大长宽
     * @return 全局地图图像
     */
    cv::Mat ShowGlobalMap(int max_size = 500);

   private:
    /// 判定当前帧是否为关键帧
    bool IsKeyFrame();

    /// 增加一个关键帧
    void AddKeyFrame();

    /// 扩展新的submap
    void ExpandSubmap();

    /// 数据成员
    size_t frame_id_ = 0;     // 帧ID计数器
    size_t keyframe_id_ = 0;  // 关键帧ID计数器
    size_t submap_id_ = 0;    // 子地图ID计数器

    bool first_scan_ = true;                            // 首帧标志
    std::shared_ptr<Frame> current_frame_ = nullptr;    // 当前帧
    std::shared_ptr<Frame> last_frame_ = nullptr;       // 上一帧
    SE2 motion_guess_;                                  // 运动预测
    std::shared_ptr<Frame> last_keyframe_ = nullptr;    // 上一关键帧
    std::shared_ptr<Submap> current_submap_ = nullptr;  // 当前子地图

    std::vector<std::shared_ptr<Submap>> all_submaps_;     // 所有子地图
    std::shared_ptr<LoopClosing> loop_closing_ = nullptr;  // 回环检测器

    // 参数
    inline static constexpr double keyframe_pos_th_ = 0.3;              // 关键帧位移阈值：0.3米
    inline static constexpr double keyframe_ang_th_ = 15 * M_PI / 180;  // 关键帧角度阈值：15度
};

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_MAPPING_2D_H
