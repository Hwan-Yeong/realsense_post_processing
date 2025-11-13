#ifndef __POINTCLOUD_PROCESSOR__
#define __POINTCLOUD_PROCESSOR__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <sensor_msgs/msg/camera_info.hpp>
// #include <realsense2_camera_msgs/msg/metadata.hpp>
// #include <realsense2_camera_msgs/msg/extrinsics.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

class PointCloudProcessor : public rclcpp::Node
{
public:
    PointCloudProcessor();

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

#endif //__POINTCLOUD_PROCESSOR__