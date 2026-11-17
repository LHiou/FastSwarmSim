#pragma once

#include <filesystem>
#include <iostream>
#include <system_error>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace marsim
{
/**
 * @brief PCD 文件超过门槛时，直接在内存中对已加载的点云做体素降采样。
 * @param path 原始 PCD 路径，仅查询文件大小，不修改原文件。
 * @param threshold_mb 触发门槛，单位 MiB；0 表示始终降采样。
 * @param leaf_size 体素边长，单位米。
 * @param cloud 已加载的 XYZI 点云，成功后替换为滤波结果。
 * @return 配置及文件有效时为 true。
 */
inline bool FilterLargeMap(const std::filesystem::path& path, double threshold_mb, float leaf_size,
    pcl::PointCloud<pcl::PointXYZI>& cloud)
{
    // 只对超过大小门槛的地图启用滤波，避免小地图丢失不必要的细节。
    std::error_code error;
    const auto bytes = std::filesystem::file_size(path, error);
    if (error || threshold_mb < 0.0 || leaf_size <= 0.0f || cloud.empty())
    {
        std::cerr << "地图路径、滤波参数无效，或点云为空: " << path << '\n';
        return false;
    }
    if (static_cast<double>(bytes) <= threshold_mb * 1024.0 * 1024.0)
    {
        return true;
    }
    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setInputCloud(cloud.makeShared());
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    pcl::PointCloud<pcl::PointXYZI> filtered;
    filter.filter(filtered);
    std::cout << "地图体素降采样: " << cloud.size() << " -> " << filtered.size() << "，体素边长 "
              << leaf_size << " m\n";
    cloud.swap(filtered);
    return !cloud.empty();
}
}  // namespace marsim
