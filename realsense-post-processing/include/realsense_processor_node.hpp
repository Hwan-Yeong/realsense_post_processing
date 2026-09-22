#pragma once

#include <memory>
#include <mutex>
#include <chrono>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/angles.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include "floor_ransac.hpp"

// build_pointcloud가 xyz만 채우므로 전 전략이 PointXYZ로 통일
using CloudT = pcl::PointCloud<pcl::PointXYZ>;

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
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_cam_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data Sync & Strategy
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_depth_img_;
    std::shared_ptr<PointCloudFilterStrategy> current_algorithm_;

    std::mutex img_mutex_;
    std::mutex algo_mutex_;

    // Callbacks
    void depth_img_callback(sensor_msgs::msg::Image::ConstSharedPtr msg);
    void depth_cam_info_callback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    void timer_callback();

    // depth 이미지 -> organized 광학 좌표계 점군. 실패 시 false.
    //
    // organized(행/열 구조 유지)로 내보내는 이유: 바닥 RANSAC이 '화면 하단 ROI'를
    // 후보로 쓰는데, 유효 점만 모아 압축해 버리면 어느 점이 몇 번째 행이었는지 잃는다.
    // 무효 픽셀은 NaN으로 채우고 is_dense = false로 표시한다.
    bool build_pointcloud(
        const sensor_msgs::msg::Image & img,
        const sensor_msgs::msg::CameraInfo & cam_info,
        sensor_msgs::msg::PointCloud2 & cloud_out);

    // 파라미터 문자열로 전략 조립 ('+'로 이으면 Composite)
    std::shared_ptr<PointCloudFilterStrategy> make_strategy(const std::string & spec);

    bool b_cam_info_updated_ = false;

    // 점군 생성 파라미터
    uint32_t row_step_ = 1;      // 세로 서브샘플 간격 [px]
    uint32_t col_step_ = 1;      // 가로 서브샘플 간격 [px]
    double depth_scale_ = 0.001; // 16UC1 raw -> m (D435i 기본 1 mm)
    double range_min_ = 0.2;     // [m]
    double range_max_ = 4.0;     // [m]

    // 바닥 RANSAC 파라미터 (튜너가 찾은 값을 그대로 넣는 자리)
    floor_ransac::Params ransac_params_;
    double obs_height_ = 0.02;   // 이 높이를 넘으면 장애물로 본다 [m]
};
