#pragma once

#include <memory>
#include <mutex>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/angles.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// Forward declaration of Strategy interface
class PointCloudFilterStrategy;

class PointCloudProcessor : public rclcpp::Node
{
public:
    PointCloudProcessor();
    ~PointCloudProcessor() override = default;

    // 알고리즘 전략 동적 교체 메서드
    void set_algorithm(std::shared_ptr<PointCloudFilterStrategy> new_algo);

private:
    // ROS 2 Interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data Sync & Strategy
    sensor_msgs::msg::PointCloud2::SharedPtr latest_msg_;
    std::shared_ptr<PointCloudFilterStrategy> current_algorithm_;
    
    std::mutex data_mutex_;
    std::mutex algo_mutex_;

    // Callbacks
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void timer_callback();
};