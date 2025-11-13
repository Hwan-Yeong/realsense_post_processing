1. WSL2에서 USB 접근 허용 설정

	WSL2는 기본적으로 USB 장치를 바로 인식하지 않습니다.
	이를 위해 Windows용 USBIPD 도구를 이용합니다.

	(1) USBIPD 설치 (Windows 쪽)

	PowerShell(관리자 권한)에서 다음 명령 실행:

	```
	$ winget install usbipd
	```


2. RealSense D435 연결 및 공유

	카메라 연결

	D435를 USB 3.0 포트에 연결합니다.

	Windows PowerShell에서 장치 목록 확인

	```
	$ usbipd list
	```

	출력 예시:

	```
	BUSID  VID:PID    DEVICE
	7-4    8086:0b07  Intel(R) RealSense(TM) Depth Camera D435
	```

	WSL로 장치 연결
	(Ubuntu 22.04가 WSL2에 설치되어 있다고 가정)
	
	wsl 확인
	
	```
	$ wsl -l -v
	```
	
	출력 예시:

	```
	  NAME                   STATE           VERSION
	* Ubuntu-22.04           Running         2
	  docker-desktop         Stopped         2
	  docker-desktop-data    Stopped         2
	```

	22.04 가 running 상태이면 됨.
	
	장치 공유가 필요함 (realsense 의 state 가 "Not shared" 일 것임 최초에는...)
	
	```
	$ usbipd bind --busid 7-4
	```
	
	이후 busid 를 usbipd list 에서 확인한 RealSense의 BUSID 입력

	```
	$ usbipd attach --wsl --busid 7-4
	```

	1-2 부분은 실제 BUSID에 맞게 바꿔주세요.



3. Ubuntu(WSL2) 안에서 장치 인식 확인

	WSL2 터미널(Ubuntu)에서:

	```
	$ lsusb
	```

	출력 예시:

	```
	Bus 001 Device 002: ID 8086:0b07 Intel Corp. Intel(R) RealSense(TM) Depth Camera D435
	```

	이게 보이면 WSL2에서 인식된 겁니다.


4. RealSense SDK 설치
	```
	$ sudo apt install software-properties-common apt-transport-https curl -y
	$ sudo curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo gpg --dearmor -o /usr/share/keyrings/librealsense-archive-keyring.gpg
	$ echo "deb [signed-by=/usr/share/keyrings/librealsense-archive-keyring.gpg] https://librealsense.intel.com/Debian/apt-repo jammy main" | sudo tee /etc/apt/sources.list.d/librealsense.list
	$ sudo apt update
	$ sudo apt install librealsense2-utils librealsense2-dev librealsense2-dbg -y
	```

	테스트:
	```
	$ realsense-viewer
	```

	GUI는 WSLg (Windows Subsystem for Linux GUI) 가 활성화되어 있으면 Windows 창으로 뜹니다.
	(Windows 11 기본 설정이라면 GUI도 됩니다.)


5. ROS2용 RealSense 드라이버 설치
	```
	$ sudo apt install ros-humble-realsense2-camera
	```


6. 노드 실행 및 토픽 확인

	카메라 실행:
	```
	$ ros2 launch realsense2_camera rs_launch.py
	```
	
	(rviz2 같이 보려면)
	$ ros2 launch realsense2_camera rs_launch.py rviz:=true

	토픽 목록 확인:
	```
	$ ros2 topic list
	```

	출력 예시:
	```
	/camera/color/image_raw
	/camera/depth/image_rect_raw
	/camera/aligned_depth_to_color/image_raw
	/camera/color/camera_info
	```

	👉 이렇게 나오면 ROS2 노드가 D435를 인식하고 토픽을 발행 중입니다.



==========================================================================

실행 시

powershell (관리자권한 실행)

(연결된 디바이스 확인)
$ usbipd list

(타겟 디바이스 wsl 연결)
$ usbipd attach --wsl --busid 7-4



wsl (ubuntu 22.04)

(디바이스 연결됐는지 확인)
$ lsusb

(ros2 실행)
$ ros2 launch realsense2_camera rs_launch.py  pointcloud.enable:=true

