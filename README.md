# realsense_post_processing
For post-processing RealSense pointcloud data

 - platform : Orange Pi 5 Max
 - realsense driver : clone & build

   ```
   $ cd ~/realsense
   $ git clone -b ros2-development https://github.com/IntelRealSense/realsense-ros.git

   $ source ~/.venv/bin/activate
   $ source /opt/ros/humble/setup.bash

   $ rosdep update
   $ rosdep install --from-paths . --ignore-src -y -r
   $ colcon build --symlink-install --parallel-workers 2
   ```
