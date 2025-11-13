#include "realsense_processor_node.hpp"


PointCloudProcessor::PointCloudProcessor() : Node("pointcloud_processor")
{
    // Subscribe to original PointCloud2
    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/camera/depth/color/points", 10,
        std::bind(&PointCloudProcessor::pc_callback, this, std::placeholders::_1));

    // Publish filtered PointCloud2
    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/camera/points_filtered", 10);
}

void PointCloudProcessor::pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Convert ROS2 PointCloud2 -> PCL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);

    // --- 1. ROI 필터 (X:0~2, Y:-1~1, Z:0.2~3.0)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 2.0);
    pass.filter(*cloud_roi);

    pass.setInputCloud(cloud_roi);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-1.0, 1.0);
    pass.filter(*cloud_roi);

    pass.setInputCloud(cloud_roi);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.2, 3.0);
    pass.filter(*cloud_roi);

    // --- 2. Voxel Grid Downsampling (leaf size 0.01m)
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    vg.setInputCloud(cloud_roi);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);

    // --- 3. Convert back to ROS2 PointCloud2
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    output.header = msg->header;  // frame_id와 timestamp 유지

    // Publish
    pc_pub_->publish(output);

    RCLCPP_INFO(this->get_logger(), "Filtered PointCloud published: %zu points", cloud_filtered->size());
}

