#include "realsense_processor_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <vector>

#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace std::chrono_literals;

class PointCloudFilterStrategy
{
public:
    virtual ~PointCloudFilterStrategy() = default;

    virtual void process(
        const CloudT::ConstPtr& input,
        CloudT::Ptr& output) = 0;

    virtual std::string name() const = 0;
};

// 여러 전략을 순서대로 연결한다.
//
// Strategy 패턴은 "하나를 고르는" 구조라 필터 체인을 만들 수 없다.
// 체인이 필요하면 이 Composite가 따로 있어야 한다 (예: passthrough_voxel + floor_ransac).
class CompositeFilter : public PointCloudFilterStrategy
{
public:
    void add(std::shared_ptr<PointCloudFilterStrategy> s)
    {
        if (s) {
            stages_.push_back(std::move(s));
        }
    }

    bool empty() const { return stages_.empty(); }

    void process(const CloudT::ConstPtr& input, CloudT::Ptr& output) override
    {
        if (!input || input->empty()) {
            return;
        }
        CloudT::ConstPtr cur = input;
        for (std::size_t i = 0; i < stages_.size(); ++i) {
            CloudT::Ptr next(new CloudT);
            stages_[i]->process(cur, next);
            cur = next;
            if (cur->empty()) {
                break;  // 중간에 비면 더 돌릴 것이 없다
            }
        }
        *output = *cur;
    }

    std::string name() const override
    {
        std::ostringstream os;
        for (std::size_t i = 0; i < stages_.size(); ++i) {
            os << (i ? " + " : "") << stages_[i]->name();
        }
        return os.str();
    }

private:
    std::vector<std::shared_ptr<PointCloudFilterStrategy>> stages_;
};

// PassThrough + VoxelGrid
class PassThroughVoxelFilter : public PointCloudFilterStrategy
{
public:
    void process(const CloudT::ConstPtr& input, CloudT::Ptr& output) override
    {
        if (!input || input->empty()) return;

        // ROI PassThrough Filter
        CloudT::Ptr cloud_roi(new CloudT);
        pcl::PassThrough<pcl::PointXYZ> pass;

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
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud_roi);
        vg.setLeafSize(0.01f, 0.01f, 0.01f);
        vg.filter(*output);
    }

    std::string name() const override { return "passthrough_voxel"; }
};

// 바닥 검출 RANSAC — floor_ransac 코어 호출.
//
// pcl::SACSegmentation을 쓰지 않는 이유 (셋 다 PCL API로는 불가능):
//   1. 법선 방향 규약 강제 (n[1] < 0). PCL 계수는 부호가 임의라
//      마운트 높이 15 cm처럼 평면이 원점 근처면 법선이 180° 튄다.
//   2. 재추정 후 tilt 재검사. setOptimizeCoefficients(true)는 재추정만 하고
//      eps-angle을 다시 보지 않는다.
//   3. 가설별 디버그 정보 (튜너가 파라미터 안정성을 판단하는 근거).
//
// 그리고 무엇보다, 이 전략이 부르는 코드가 Python 튜너가 부르는 코드와 '같은 코드'다.
class FloorRansacCoreFilter : public PointCloudFilterStrategy
{
public:
    FloorRansacCoreFilter(const floor_ransac::Params & params, double obs_height,
                          rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
    : params_(params), obs_height_(obs_height), logger_(logger), clock_(std::move(clock)) {}

    void process(const CloudT::ConstPtr& input, CloudT::Ptr& output) override
    {
        if (!input || input->empty()) return;

        const int height = static_cast<int>(input->height);
        const int width = static_cast<int>(input->width);
        const Eigen::Index n_pts = static_cast<Eigen::Index>(input->size());

        // PCL(float, NaN 포함) -> 코어가 받는 (N,3) double.
        // 무효 픽셀은 (0,0,0)으로 둔다: 코어는 z > z_min 조건으로 걸러낸다.
        floor_ransac::PointsRowMajor pts(n_pts, 3);
        for (Eigen::Index i = 0; i < n_pts; ++i) {
            const auto & p = (*input)[static_cast<std::size_t>(i)];
            if (std::isfinite(p.z)) {
                pts(i, 0) = static_cast<double>(p.x);
                pts(i, 1) = static_cast<double>(p.y);
                pts(i, 2) = static_cast<double>(p.z);
            } else {
                pts.row(i).setZero();
            }
        }

        // organized면 화면 하단 ROI를 쓰고, 아니면 z 범위로만 거른다.
        //
        // 어느 쪽이든 무효 픽셀((0,0,0)으로 채운 자리)은 반드시 빼야 한다.
        // 원점에 수십만 점이 몰려 있으면 '원점을 지나는 평면'이 압도적 다수결로
        // 뽑혀서 카메라 높이가 0으로 나온다.
        floor_ransac::Result res;
        if (height > 1 && static_cast<Eigen::Index>(height) * width == n_pts) {
            const floor_ransac::PointsRowMajor cand =
                floor_ransac::select_candidates(pts, height, width, params_);
            res = floor_ransac::fit_plane(cand, params_);
        } else {
            floor_ransac::PointsRowMajor cand(n_pts, 3);
            Eigen::Index k = 0;
            for (Eigen::Index i = 0; i < n_pts; ++i) {
                const double z = pts(i, 2);
                if (z > params_.z_min && z < params_.z_max) {
                    cand.row(k++) = pts.row(i);
                }
            }
            cand.conservativeResize(k, 3);
            res = floor_ransac::fit_plane(cand, params_);
        }

        if (!res.ok) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 3000,
                "[floor_ransac] 바닥 추출 실패 (%s, 후보 %d점) - 원본 유지",
                floor_ransac::status_string(res.status), res.debug.candidates);
            *output = *input;  // 평면 미검출 시 원본 유지 (장애물을 지우지 않는다)
            return;
        }

        const floor_ransac::Angles ang = floor_ransac::plane_angles(res.plane.n);
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 5000,
            "[floor_ransac] cam h %.3f m, pitch %.2f deg, roll %.2f deg, inlier %d/%d",
            res.plane.d, ang.pitch_down_deg, ang.roll_deg, res.inliers, res.debug.candidates);

        // 바닥 위 obs_height 를 넘는 점만 남긴다 (= 바닥 제거, 장애물만).
        const Eigen::VectorXd h = floor_ransac::point_heights(pts, res.plane);
        output->clear();
        output->reserve(static_cast<std::size_t>(n_pts));
        for (Eigen::Index i = 0; i < n_pts; ++i) {
            const auto & p = (*input)[static_cast<std::size_t>(i)];
            if (std::isfinite(p.z) && h[i] > obs_height_) {
                output->push_back(p);
            }
        }
        output->width = static_cast<uint32_t>(output->size());
        output->height = 1;
        output->is_dense = true;
    }

    std::string name() const override { return "floor_ransac"; }

private:
    floor_ransac::Params params_;
    double obs_height_;
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;
};


PointCloudProcessor::PointCloudProcessor() : Node("pointcloud_processor")
{
    // ---- 점군 생성 파라미터 ----
    row_step_ = static_cast<uint32_t>(this->declare_parameter<int>("row_step", 1));
    col_step_ = static_cast<uint32_t>(this->declare_parameter<int>("col_step", 1));
    depth_scale_ = this->declare_parameter<double>("depth_scale", 0.001);
    range_min_ = this->declare_parameter<double>("range_min", 0.2);
    range_max_ = this->declare_parameter<double>("range_max", 4.0);
    if (row_step_ == 0) row_step_ = 1;
    if (col_step_ == 0) col_step_ = 1;

    // ---- 바닥 RANSAC 파라미터 ----
    // 기본값은 rs_ransac_tuner.py로 bag/20260922_174035.db3를 튜닝해 얻은 값이다.
    // 튜너가 부르는 코드와 여기가 부르는 코드가 같으므로 값이 그대로 재현된다.
    ransac_params_.thresh = this->declare_parameter<double>("ransac.thresh", 0.006);
    ransac_params_.iters = this->declare_parameter<int>("ransac.iters", 1000);
    ransac_params_.max_tilt_deg = this->declare_parameter<double>("ransac.max_tilt_deg", 10.0);
    ransac_params_.roi_bottom = this->declare_parameter<double>("ransac.roi_bottom", 0.5);
    ransac_params_.z_min = this->declare_parameter<double>("ransac.z_min", 0.2);
    ransac_params_.z_max = this->declare_parameter<double>("ransac.z_max", 1.5);
    ransac_params_.eval_n = this->declare_parameter<int>("ransac.eval_n", 20000);
    ransac_params_.seed =
        static_cast<uint64_t>(this->declare_parameter<int>("ransac.seed", 0));
    ransac_params_.min_candidates =
        this->declare_parameter<int>("ransac.min_candidates", 200);
    obs_height_ = this->declare_parameter<double>("floor.obs_height", 0.02);

    const std::string pipeline =
        this->declare_parameter<std::string>("pipeline", "floor_ransac");

    depth_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/depth/image_rect_raw", 10,
        std::bind(&PointCloudProcessor::depth_img_callback, this, std::placeholders::_1));

    depth_cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera/depth/camera_info", 10,
        std::bind(&PointCloudProcessor::depth_cam_info_callback, this, std::placeholders::_1));

    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/camera/points_filtered", 10);

    current_algorithm_ = make_strategy(pipeline);

    timer_ = this->create_wall_timer(
        33ms, std::bind(&PointCloudProcessor::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
        "PointCloudProcessor 시작. pipeline='%s', RANSAC thresh %.3f m, iters %d, "
        "max_tilt %.1f deg, roi_bottom %.2f, z %.2f~%.2f m, eval_n %d",
        current_algorithm_ ? current_algorithm_->name().c_str() : "(none)",
        ransac_params_.thresh, ransac_params_.iters, ransac_params_.max_tilt_deg,
        ransac_params_.roi_bottom, ransac_params_.z_min, ransac_params_.z_max,
        ransac_params_.eval_n);
}

std::shared_ptr<PointCloudFilterStrategy>
PointCloudProcessor::make_strategy(const std::string & spec)
{
    auto composite = std::make_shared<CompositeFilter>();
    std::istringstream ss(spec);
    std::string token;
    int stage = 0;
    while (std::getline(ss, token, '+')) {
        // 공백 제거
        token.erase(0, token.find_first_not_of(" \t"));
        const auto last = token.find_last_not_of(" \t");
        if (last != std::string::npos) {
            token.erase(last + 1);
        }
        if (token.empty()) {
            continue;
        }
        if (token == "floor_ransac") {
            if (stage > 0) {
                // 앞 단계가 점군을 압축(unorganized)하면 행/열 구조가 사라져
                // 화면 하단 ROI를 쓸 수 없고, z 범위로만 거른 후보로 적합하게 된다.
                // 후보 수가 급감해 평면이 불안정해지므로 floor_ransac을 먼저 둘 것.
                RCLCPP_WARN(this->get_logger(),
                    "floor_ransac이 %d번째 단계다. 앞 단계가 organized 구조를 깨면 "
                    "하단 ROI를 쓸 수 없어 바닥 추정이 불안정해진다 "
                    "(권장: 'floor_ransac+...' 처럼 먼저 배치)", stage + 1);
            }
            composite->add(std::make_shared<FloorRansacCoreFilter>(
                ransac_params_, obs_height_, this->get_logger(), this->get_clock()));
            ++stage;
        } else if (token == "passthrough_voxel") {
            composite->add(std::make_shared<PassThroughVoxelFilter>());
            ++stage;
        } else {
            RCLCPP_WARN(this->get_logger(), "알 수 없는 pipeline 단계 '%s' - 건너뜀",
                        token.c_str());
        }
    }
    if (composite->empty()) {
        RCLCPP_WARN(this->get_logger(),
            "pipeline '%s'에서 유효한 단계를 찾지 못해 floor_ransac으로 대체합니다",
            spec.c_str());
        composite->add(std::make_shared<FloorRansacCoreFilter>(
            ransac_params_, obs_height_, this->get_logger(), this->get_clock()));
    }
    return composite;
}

void PointCloudProcessor::depth_img_callback(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(img_mutex_);
    latest_depth_img_ = msg;
}

void PointCloudProcessor::depth_cam_info_callback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
    if (!b_cam_info_updated_) {
        RCLCPP_INFO(this->get_logger(), "Camera info received and stored.");
        camera_info_ = msg;
        b_cam_info_updated_ = true;
    }
}

bool PointCloudProcessor::build_pointcloud(
    const sensor_msgs::msg::Image & image,
    const sensor_msgs::msg::CameraInfo & info,
    sensor_msgs::msg::PointCloud2 & cloud_out)
{
    if (image.encoding != "16UC1") {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                            "[depth_camera] Unexpected depth encoding '%s' "
                            "(expected 16UC1) - skipping",
                            image.encoding.c_str());
        return false;
    }

    const uint32_t width = image.width;
    const uint32_t height = image.height;
    if (width == 0 || height == 0 || image.data.empty()) {
        return false;
    }

    // 카메라 내부 파라미터 (camera_info K 에서 추출)
    const float fx = static_cast<float>(info.k[0]);
    const float fy = static_cast<float>(info.k[4]);
    const float cx = static_cast<float>(info.k[2]);
    const float cy = static_cast<float>(info.k[5]);
    if (fx == 0.0f || fy == 0.0f) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                            "[depth_camera] Invalid camera intrinsics "
                            "(fx/fy = 0). Waiting for valid camera_info.");
        return false;
    }

    const uint32_t out_h = (height + row_step_ - 1) / row_step_;
    const uint32_t out_w = (width + col_step_ - 1) / col_step_;

    cloud_out.header = image.header;
    // 카메라 드라이버의 image stamp 는 ROS time 과 다른 클럭이라
    // stamp 기준 TF lookup 이 항상 timeout(50ms/frame 블로킹)된다.
    // 수신 시각(ROS time)으로 재스탬프하여 TF 를 즉시 조회하게 한다.
    cloud_out.header.stamp = this->now();
    cloud_out.height = out_h;   // organized 유지 (바닥 ROI가 행 구조를 필요로 함)
    cloud_out.width = out_w;
    cloud_out.is_dense = false; // 무효 픽셀은 NaN
    cloud_out.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_out);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(static_cast<size_t>(out_h) * out_w);

    // PointCloud2Modifier::resize()는 height/width가 둘 다 1이 아니면
    // height를 1로 눌러 버린다(= organized 구조 파괴). resize 뒤에 되돌려 놓아야
    // 바닥 RANSAC이 화면 하단 ROI를 쓸 수 있다.
    cloud_out.height = out_h;
    cloud_out.width = out_w;
    cloud_out.row_step = out_w * cloud_out.point_step;

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud_out, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud_out, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud_out, "z");

    const float scale = static_cast<float>(depth_scale_);
    const float rmin = static_cast<float>(range_min_);
    const float rmax = static_cast<float>(range_max_);
    const float nan = std::numeric_limits<float>::quiet_NaN();

    for (uint32_t v = 0; v < height; v += row_step_) {
        const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(
            image.data.data() + static_cast<size_t>(v) * image.step);
        for (uint32_t u = 0; u < width; u += col_step_, ++it_x, ++it_y, ++it_z) {
            const uint16_t raw = depth_row[u];
            const float z = static_cast<float>(raw) * scale;
            if (raw == 0 || z < rmin || z > rmax) {
                *it_x = *it_y = *it_z = nan;  // 무효 픽셀 (행/열 구조는 유지)
                continue;
            }
            // 광학 좌표계: x=오른쪽, y=아래, z=정면
            *it_x = (static_cast<float>(u) - cx) * z / fx;
            *it_y = (static_cast<float>(v) - cy) * z / fy;
            *it_z = z;
        }
    }
    return true;
}

void PointCloudProcessor::timer_callback()
{
    if (!b_cam_info_updated_) return;

    sensor_msgs::msg::PointCloud2 msg_to_process;
    std::shared_ptr<PointCloudFilterStrategy> algo_to_use;

    {
        std::lock_guard<std::mutex> lock(img_mutex_);
        if (!latest_depth_img_) {
            return;
        }
        if (!build_pointcloud(*latest_depth_img_, *camera_info_, msg_to_process)) {
            return;
        }
    }

    {
        std::lock_guard<std::mutex> lock(algo_mutex_);
        algo_to_use = current_algorithm_;
    }

    if (!algo_to_use) return;

    CloudT::Ptr cloud_in(new CloudT);
    pcl::fromROSMsg(msg_to_process, *cloud_in);

    CloudT::Ptr cloud_out(new CloudT);
    const auto t0 = std::chrono::steady_clock::now();
    algo_to_use->process(cloud_in, cloud_out);
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_out, output);
    output.header = msg_to_process.header; // Timestamp 및 frame_id 동기화

    pc_pub_->publish(output);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
        "Filtered PointCloud published: %zu points (%.1f ms)", cloud_out->size(), ms);
}

void PointCloudProcessor::set_algorithm(std::shared_ptr<PointCloudFilterStrategy> new_algo)
{
    std::lock_guard<std::mutex> lock(algo_mutex_);
    current_algorithm_ = new_algo;
    RCLCPP_INFO(this->get_logger(), "Algorithm strategy successfully switched.");
}
