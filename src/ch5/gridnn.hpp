//
// Created by xiang on 2021/8/25.
//

#ifndef SLAM_IN_AUTO_DRIVING_GRID2D_HPP
#define SLAM_IN_AUTO_DRIVING_GRID2D_HPP

#include "common/eigen_types.h"
#include "common/math_utils.h"
#include "common/point_types.h"

#include <glog/logging.h>
#include <execution>
#include <unordered_map>
#include "bfnn.h"

namespace sad {

/**
 * 栅格法最近邻搜索器
 * @tparam dim 模板参数，支持2D或3D栅格
 *
 * 功能：
 * - 将点云数据划分到规则栅格中，减少最近邻搜索的计算范围。
 * - 支持单点最近邻搜索和点云到点云的最近邻匹配。
 * - 提供串行和并行两种匹配方式。
 */
template <int dim>
class GridNN {
   public:
    using KeyType = Eigen::Matrix<int, dim, 1>;   // 栅格索引类型（整数坐标）
    using PtType = Eigen::Matrix<float, dim, 1>;  // 点坐标类型（浮点数）

    /**
     * 邻域类型
     * 定义在搜索最近邻时，考虑的栅格邻域范围。
     */
    enum class NearbyType {
        CENTER,  // 只考虑中心栅格
        // 2D 栅格邻域类型
        NEARBY4,  // 上下左右四个邻居
        NEARBY8,  // 上下左右+四角八个邻居
        // 3D 栅格邻域类型
        NEARBY6,  // 上下左右前后六个邻居
    };

    /**
     * 构造函数
     * @param resolution 栅格分辨率，决定每个栅格的大小
     * @param nearby_type 邻域类型，决定搜索时考虑的邻域范围
     */
    explicit GridNN(float resolution = 0.1, NearbyType nearby_type = NearbyType::NEARBY4)
        : resolution_(resolution), nearby_type_(nearby_type) {
        inv_resolution_ = 1.0 / resolution_;

        // 检查维度与邻域类型的兼容性
        if (dim == 2 && nearby_type_ == NearbyType::NEARBY6) {
            LOG(INFO) << "2D grid does not support nearby6, using nearby4 instead.";
            nearby_type_ = NearbyType::NEARBY4;
        } else if (dim == 3 && (nearby_type_ != NearbyType::NEARBY6 && nearby_type_ != NearbyType::CENTER)) {
            LOG(INFO) << "3D grid does not support nearby4/8, using nearby6 instead.";
            nearby_type_ = NearbyType::NEARBY6;
        }

        // 生成邻域偏移
        GenerateNearbyGrids();
    }

    /**
     * 设置点云，建立栅格索引
     * @param cloud 输入点云
     * @return 是否成功
     *
     * 功能：
     * - 将点云中的每个点映射到对应的栅格坐标。
     * - 在哈希表中存储每个栅格内的点索引。
     */
    bool SetPointCloud(CloudPtr cloud);

    /**
     * 获取单点的最近邻
     * @param pt 查询点
     * @param closest_pt 输出最近邻点的坐标
     * @param idx 输出最近邻点在点云中的索引
     * @return 是否找到最近邻
     *
     * 功能：
     * - 将查询点映射到栅格坐标。
     * - 在查询点所在栅格及其邻域栅格中查找最近邻。
     */
    bool GetClosestPoint(const PointType& pt, PointType& closest_pt, size_t& idx);

    /**
     * 对比两个点云，找到最近邻匹配（串行版本）
     * @param ref 参考点云
     * @param query 查询点云
     * @param matches 输出匹配对，格式为 {参考点索引, 查询点索引}
     * @return 是否成功
     */
    bool GetClosestPointForCloud(CloudPtr ref, CloudPtr query, std::vector<std::pair<size_t, size_t>>& matches);

    /**
     * 对比两个点云，找到最近邻匹配（并行版本）
     * @param ref 参考点云
     * @param query 查询点云
     * @param matches 输出匹配对，格式为 {参考点索引, 查询点索引}
     * @return 是否成功
     */
    bool GetClosestPointForCloudMT(CloudPtr ref, CloudPtr query, std::vector<std::pair<size_t, size_t>>& matches);

   private:
    /**
     * 根据邻域类型，生成邻域偏移
     * 功能：
     * - 根据 `nearby_type_`，生成邻域栅格的偏移向量列表。
     * - 例如，对于 NEARBY4，生成 {(-1, 0), (1, 0), (0, -1), (0, 1)}。
     */
    void GenerateNearbyGrids();

    /**
     * 将空间坐标转换为栅格坐标
     * @param pt 输入点的坐标
     * @return 栅格坐标
     *
     * 功能：
     * - 根据分辨率，将点的浮点坐标映射到整数栅格坐标。
     */
    KeyType Pos2Grid(const PtType& pt);

    float resolution_ = 0.1;       // 栅格分辨率
    float inv_resolution_ = 10.0;  // 分辨率倒数（优化计算）

    NearbyType nearby_type_ = NearbyType::NEARBY4;                           // 邻域类型
    std::unordered_map<KeyType, std::vector<size_t>, hash_vec<dim>> grids_;  // 栅格数据，存储每个栅格内的点索引
    CloudPtr cloud_;                                                         // 输入点云

    std::vector<KeyType> nearby_grids_;  // 邻域栅格的偏移向量
};

// 实现部分
template <int dim>
bool GridNN<dim>::SetPointCloud(CloudPtr cloud) {
    grids_.clear();
    for (size_t idx = 0; idx < cloud->size(); ++idx) {
        auto key = Pos2Grid(ToEigen<float, dim>(cloud->points[idx]));
        grids_[key].emplace_back(idx);
    }
    cloud_ = cloud;
    LOG(INFO) << "grids: " << grids_.size();
    return true;
}

template <int dim>
Eigen::Matrix<int, dim, 1> GridNN<dim>::Pos2Grid(const Eigen::Matrix<float, dim, 1>& pt) {
    return (pt.array() * inv_resolution_).round().template cast<int>();
}

template <>
inline void GridNN<2>::GenerateNearbyGrids() {
    if (nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (nearby_type_ == NearbyType::NEARBY4) {
        nearby_grids_ = {Vec2i(0, 0), Vec2i(-1, 0), Vec2i(1, 0), Vec2i(0, 1), Vec2i(0, -1)};
    } else if (nearby_type_ == NearbyType::NEARBY8) {
        nearby_grids_ = {
            Vec2i(0, 0),   Vec2i(-1, 0), Vec2i(1, 0),  Vec2i(0, 1), Vec2i(0, -1),
            Vec2i(-1, -1), Vec2i(-1, 1), Vec2i(1, -1), Vec2i(1, 1),
        };
    }
}

template <>
inline void GridNN<3>::GenerateNearbyGrids() {
    if (nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (nearby_type_ == NearbyType::NEARBY6) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    }
}

template <int dim>
bool GridNN<dim>::GetClosestPoint(const PointType& pt, PointType& closest_pt, size_t& idx) {
    std::vector<size_t> idx_to_check;
    auto key = Pos2Grid(ToEigen<float, dim>(pt));

    std::for_each(nearby_grids_.begin(), nearby_grids_.end(), [&key, &idx_to_check, this](const KeyType& delta) {
        auto dkey = key + delta;
        auto iter = grids_.find(dkey);
        if (iter != grids_.end()) {
            idx_to_check.insert(idx_to_check.end(), iter->second.begin(), iter->second.end());
        }
    });

    if (idx_to_check.empty()) {
        return false;
    }

    CloudPtr nearby_cloud(new PointCloudType);
    std::vector<size_t> nearby_idx;
    for (auto& idx : idx_to_check) {
        nearby_cloud->points.emplace_back(cloud_->points[idx]);
        nearby_idx.emplace_back(idx);
    }

    size_t closest_point_idx = bfnn_point(nearby_cloud, ToVec3f(pt));
    idx = nearby_idx.at(closest_point_idx);
    closest_pt = cloud_->points[idx];

    return true;
}

template <int dim>
bool GridNN<dim>::GetClosestPointForCloud(CloudPtr ref, CloudPtr query,
                                          std::vector<std::pair<size_t, size_t>>& matches) {
    matches.clear();
    for (size_t idx = 0; idx < query->size(); ++idx) {
        PointType cp;
        size_t cp_idx;
        if (GetClosestPoint(query->points[idx], cp, cp_idx)) {
            matches.emplace_back(cp_idx, idx);
        }
    }
    return true;
}

template <int dim>
bool GridNN<dim>::GetClosestPointForCloudMT(CloudPtr ref, CloudPtr query,
                                            std::vector<std::pair<size_t, size_t>>& matches) {
    matches.clear();
    matches.resize(query->size());

    std::for_each(std::execution::par_unseq, matches.begin(), matches.end(), [this, &matches, &query](auto& match) {
        size_t idx = &match - &matches[0];
        PointType cp;
        size_t cp_idx;
        if (GetClosestPoint(query->points[idx], cp, cp_idx)) {
            match = {cp_idx, idx};
        } else {
            match = {math::kINVALID_ID, math::kINVALID_ID};
        }
    });

    return true;
}
}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_GRID2D_HPP
