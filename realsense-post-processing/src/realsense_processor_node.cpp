#include "realsense_processor_node.hpp"
#include <mutex>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class PointCloudFilterStrategy
{
public:
    virtual ~PointCloudFilterStrategy() = default;
    
    virtual void process(
        const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output) = 0;
};

// PassThrough + VoxelGrid
class PassThroughVoxelFilter : public PointCloudFilterStrategy
{
public:
    void process(
        const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output) override
    {
        if (!input || input->empty()) return;

        // ROI PassThrough Filter
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        
        pass.setInputCloud(input);
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

        // Voxel Grid Downsampling
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(cloud_roi);
        vg.setLeafSize(0.01f, 0.01f, 0.01f);
        vg.filter(*output);
    }
};

// 바닥 검출 RANSAC
class FloorDetectionRansacFilter : public PointCloudFilterStrategy
{
public:
    void process(
        const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output) override
    {
        if (!input || input->empty()) return;

        // 1. RANSAC 평면 세그멘테이션 설정
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); // 지정 축 기반 평면
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.05); // 평면과의 거리가 5cm 이내인 점들을 Inlier로 판별
        seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0)); // Z축(수직축) 방향의 평면 검출
        seg.setEpsAngle(pcl::deg2rad(15.0f)); // 허용 오차 각도 (15도)

        seg.setInputCloud(input);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            *output = *input; // 평면 미검출 시 원본 유지
            return;
        }

        // 2. 바닥 포인트 제거 (setNegative: true = 바닥 제거 후 장애물만 남김)
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(input);
        extract.setIndices(inliers);
        extract.setNegative(true); 
        extract.filter(*output);
    }
};


PointCloudProcessor::PointCloudProcessor() : Node("pointcloud_processor")
{
    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/camera/depth/color/points", 10,
        std::bind(&PointCloudProcessor::pc_callback, this, std::placeholders::_1));

    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/camera/points_filtered", 10);

    current_algorithm_ = std::make_shared<FloorDetectionRansacFilter>();

    timer_ = this->create_wall_timer(
        33ms, std::bind(&PointCloudProcessor::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "PointCloudProcessor initialized with Timer-based Strategy Pattern.");
}

void PointCloudProcessor::pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_msg_ = msg;
}

void PointCloudProcessor::timer_callback()
{
    sensor_msgs::msg::PointCloud2::SharedPtr msg_to_process;
    std::shared_ptr<PointCloudFilterStrategy> algo_to_use;

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!latest_msg_) {
            return;
        }
        msg_to_process = latest_msg_;
    }

    {
        std::lock_guard<std::mutex> lock(algo_mutex_);
        algo_to_use = current_algorithm_;
    }

    if (!algo_to_use) return;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg_to_process, *cloud_in);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    algo_to_use->process(cloud_in, cloud_out);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_out, output);
    output.header = msg_to_process->header; // Timestamp 및 frame_id 동기화

    pc_pub_->publish(output);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
        "Filtered PointCloud published: %zu points", cloud_out->size());
}

void PointCloudProcessor::set_algorithm(std::shared_ptr<PointCloudFilterStrategy> new_algo)
{
    std::lock_guard<std::mutex> lock(algo_mutex_);
    current_algorithm_ = new_algo;
    RCLCPP_INFO(this->get_logger(), "Algorithm strategy successfully switched.");
}