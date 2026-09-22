"""realsense_post_processing 노드 실행.

RANSAC 기본값은 rs_ransac_tuner.py로 bag/20260922_174035.db3를 튜닝해 얻은 값이다.
튜너와 노드가 같은 C++ 코어(floor_ransac)를 호출하므로 값이 그대로 재현된다.

.db3 녹화로 검증하려면 브리지를 함께 띄운다 (ros2 bag play로는 재생 불가):
  ros2 launch realsense_post_processing realsense_post_processing.launch.py
  python3 scripts/rs_db3_bridge.py --bag bag/20260922_174035.db3
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 'floor_ransac' | 'passthrough_voxel' | '+'로 이어 붙인 Composite
#   예: 'passthrough_voxel+floor_ransac'
DEFAULT_PIPELINE = "floor_ransac"

RANSAC_DEFAULTS = {
    # 3 cm 턱을 바닥에 삼키지 않으려면 inlier 밴드를 좁게 — 5 cm는 턱을 먹어버린다
    "ransac.thresh": 0.006,
    "ransac.iters": 1000,
    # 바닥 법선 허용 기울기. 좁힐수록 벽/경사면 오채택이 줄어든다
    "ransac.max_tilt_deg": 10.0,
    "ransac.roi_bottom": 0.5,
    "ransac.z_min": 0.2,
    # 마운트가 15 cm로 낮아 먼 바닥은 스침각이라 노이즈뿐 -> 1.5 m에서 자른다
    "ransac.z_max": 1.5,
    # 추정 안정성에 가장 크게 기여한 값 (5000 -> 20000에서 pitch std가 절반)
    "ransac.eval_n": 20000,
    "ransac.seed": 0,
    "ransac.min_candidates": 200,
    "floor.obs_height": 0.02,
}

POINTCLOUD_DEFAULTS = {
    "row_step": 1,
    "col_step": 1,
    "depth_scale": 0.001,
    "range_min": 0.2,
    "range_max": 4.0,
}


def generate_launch_description():
    pipeline_arg = DeclareLaunchArgument(
        "pipeline", default_value=DEFAULT_PIPELINE,
        description="필터 전략. '+'로 이으면 Composite (예: passthrough_voxel+floor_ransac)")

    node = Node(
        package="realsense_post_processing",
        executable="realsense_post_processing",
        name="pointcloud_processor",
        output="screen",
        parameters=[
            {"pipeline": LaunchConfiguration("pipeline")},
            RANSAC_DEFAULTS,
            POINTCLOUD_DEFAULTS,
        ],
    )

    return LaunchDescription([pipeline_arg, node])
