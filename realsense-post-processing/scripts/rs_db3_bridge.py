#!/usr/bin/env python3
"""녹화 .db3 -> ROS 2 depth Image + CameraInfo 브리지.

`ros2 bag play`로는 이 파일을 쓸 수 없다. bag/*.db3는 SQLite/rosbag2 컨테이너를
쓰지만 내용이 librealsense 내부 포맷이라, 토픽이 전부 /device_0/sensor_0/... 형태의
std_msgs/String이고 sensor_msgs/Image가 아예 없다.
읽는 방법은 pyrealsense2의 enable_device_from_file 경로뿐이므로,
C++ 노드에 먹이려면 이렇게 Python이 읽어서 다시 발행해 주어야 한다.

(튜닝 경로에는 이 브리지가 필요 없다 — 튜너는 pybind11로 코어를 직접 부른다.
 이건 ROS 노드 실데이터 검증/성능평가 때만 쓴다.)

예:
  ros2 run realsense_post_processing rs_db3_bridge.py --ros-args -p bag:=bag/x.db3
  python3 rs_db3_bridge.py --bag bag/20260922_174035.db3 --fps 15 --loop
"""
import argparse
import sys

import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class Db3Bridge(Node):
    def __init__(self, args):
        super().__init__("rs_db3_bridge")
        self.frame_id = args.frame_id
        self.loop = args.loop

        self.img_pub = self.create_publisher(Image, args.image_topic, 10)
        self.info_pub = self.create_publisher(CameraInfo, args.info_topic, 10)

        self.pipe, self.cfg = rs.pipeline(), rs.config()
        rs.config.enable_device_from_file(self.cfg, args.bag, repeat_playback=args.loop)
        self.cfg.enable_stream(rs.stream.depth)
        prof = self.pipe.start(self.cfg)
        prof.get_device().as_playback().set_real_time(False)

        self.scale = prof.get_device().first_depth_sensor().get_depth_scale()
        intr = prof.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        self.info = self._camera_info(intr)
        self.get_logger().info(
            f"{args.bag}: {intr.width}x{intr.height} "
            f"fx {intr.fx:.1f} fy {intr.fy:.1f} cx {intr.ppx:.1f} cy {intr.ppy:.1f}, "
            f"depth_scale {self.scale}")
        if abs(self.scale - 0.001) > 1e-6:
            self.get_logger().warn(
                f"depth_scale이 {self.scale}이다. 노드의 depth_scale 파라미터를 맞출 것")

        self.count = 0
        self.timer = self.create_timer(1.0 / max(args.fps, 1e-3), self.tick)

    def _camera_info(self, intr):
        info = CameraInfo()
        info.width, info.height = intr.width, intr.height
        info.distortion_model = "plumb_bob"
        info.d = list(intr.coeffs)
        info.k = [intr.fx, 0.0, intr.ppx, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [intr.fx, 0.0, intr.ppx, 0.0, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def tick(self):
        try:
            frames = self.pipe.wait_for_frames(2000)
        except RuntimeError:
            self.get_logger().info(f"재생 끝 ({self.count} 프레임 발행)")
            if not self.loop:
                raise SystemExit(0)
            return
        depth = frames.get_depth_frame()
        if not depth:
            return

        a = np.asanyarray(depth.get_data())  # uint16, (H, W)
        stamp = self.get_clock().now().to_msg()

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = a.shape
        msg.encoding = "16UC1"
        msg.is_bigendian = 0
        msg.step = a.shape[1] * 2
        msg.data = a.tobytes()
        self.img_pub.publish(msg)

        self.info.header.stamp = stamp
        self.info.header.frame_id = self.frame_id
        self.info_pub.publish(self.info)
        self.count += 1


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--bag", required=True)
    p.add_argument("--fps", type=float, default=15.0)
    p.add_argument("--loop", action="store_true")
    p.add_argument("--frame_id", default="camera_depth_optical_frame")
    p.add_argument("--image_topic", default="/camera/camera/depth/image_rect_raw")
    p.add_argument("--info_topic", default="/camera/camera/depth/camera_info")
    args, ros_args = p.parse_known_args()

    rclpy.init(args=ros_args)
    node = Db3Bridge(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.pipe.stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
