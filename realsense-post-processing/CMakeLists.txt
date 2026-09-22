cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0077 NEW)
project(realsense_post_processing)

set(CMAKE_BUILD_TYPE Release) #set(CMAKE_BUILD_TYPE Debug)

set(PROJECT_LIB ${PROJECT_NAME}_lib)
set(CORE_LIB floor_ransac)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 (ROS 2 Humble 최소 요구)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)  # 코어를 pybind11 모듈에 정적 링크하므로 필요

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # PYTHON_INSTALL_DIR (pybind11 모듈 설치 위치)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------------------
# 알고리즘 코어: ROS / PCL 비의존 (Eigen + 표준 라이브러리만)
#
# 여기가 바닥 RANSAC의 단일 구현이다. ROS 노드 / pybind11 모듈 / gtest가
# 전부 이 라이브러리를 링크한다 — 구현이 갈라지면 튜닝 결과가 재현되지 않는다.
# 의도적으로 rclcpp / PCL 을 링크하지 않는다.
# ---------------------------------------------------------------------------
add_library(${CORE_LIB} STATIC src/floor_ransac.cpp)
target_include_directories(${CORE_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${CORE_LIB} PUBLIC Eigen3::Eigen)

# ---------------------------------------------------------------------------
# ROS 2 노드
# ---------------------------------------------------------------------------
include_directories(
	include
	${CMAKE_CURRENT_SOURCE_DIR}
	${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Source files
set(src_files
	src/realsense_processor_node.cpp
)

set(dependencies
	rclcpp
	std_msgs
	sensor_msgs
    PCL
	pcl_conversions
    pcl_ros
)

# Add library
add_library(${PROJECT_LIB} STATIC ${src_files})
target_link_libraries(${PROJECT_LIB} ${CORE_LIB} ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_LIB} ${dependencies})

install(TARGETS
  ${PROJECT_LIB}
  ${CORE_LIB}
  DESTINATION lib/${PROJECT_NAME})

# Add executable
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_LIB} ${CORE_LIB} ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Installation
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION
  lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY
	launch #params
	DESTINATION
	share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# pybind11 모듈 — Python 튜너가 컴파일된 코어를 그대로 호출한다
# ---------------------------------------------------------------------------
# pybind11의 새 툴체인은 Python3::Module 타깃을 먼저 요구한다
find_package(Python3 COMPONENTS Interpreter Development QUIET)
find_package(pybind11 QUIET)
if(NOT pybind11_FOUND)
  find_package(pybind11_vendor QUIET)
  find_package(pybind11 QUIET)
endif()

if(pybind11_FOUND AND Python3_Development_FOUND)
  pybind11_add_module(floor_ransac_py src/floor_ransac_py.cpp)
  target_link_libraries(floor_ransac_py PRIVATE ${CORE_LIB})
  if(NOT PYTHON_INSTALL_DIR)
    set(PYTHON_INSTALL_DIR
      "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
  endif()
  # 튜너는 `source install/setup.bash` 후 그냥 import 하면 된다
  install(TARGETS floor_ransac_py LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}")
else()
  message(WARNING
    "pybind11을 찾지 못해 floor_ransac_py 모듈을 건너뜁니다. "
    "Python 튜너(scripts/rs_ransac_tuner.py)는 이 모듈이 있어야 동작합니다. "
    "설치: sudo apt install python3-pybind11  또는  pip install pybind11")
endif()

# Python 공용 모듈. site-packages에 'common'이라는 이름을 올리면 충돌 위험이 커서
# 스크립트 옆에 그대로 설치하고, 각 스크립트가 자기 디렉터리를 sys.path에 넣어 import 한다.
install(DIRECTORY scripts/common
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

install(PROGRAMS
  scripts/rs_floor_ransac_test.py
  scripts/rs_bag_filter_test.py
  scripts/rs_ransac_tuner.py
  scripts/rs_db3_bridge.py
  DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# 테스트
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_floor_ransac test/test_floor_ransac.cpp)
  if(TARGET test_floor_ransac)
    target_link_libraries(test_floor_ransac ${CORE_LIB})
  endif()

  # C++ 기준값 덤프. test/test_floor_ransac_py.py가 이걸 돌려
  # pybind11 모듈 결과와 비트 단위로 비교한다 (Phase 3 게이트).
  add_executable(dump_reference test/dump_reference.cpp)
  target_link_libraries(dump_reference ${CORE_LIB})

  # ament_add_pytest_test 대신 직접 등록한다.
  # ament의 pytest 탐지는 `python3 -m pytest --version`으로 하는데, 이 환경에는
  # ~/.local의 anyio 4.x가 시스템 pytest 6.2.5보다 최신 pytest를 요구해서
  # 플러그인 자동 로드가 깨져 있다 -> ament가 "pytest 없음"으로 판단해 테스트를 건너뛴다.
  # PYTEST_DISABLE_PLUGIN_AUTOLOAD로 그 플러그인을 빼면 정상 동작한다.
  # (근본 해결은 `pip install -U pytest` 또는 ~/.local의 anyio 제거)
  if(TARGET floor_ransac_py AND Python3_Interpreter_FOUND)
    add_test(NAME test_floor_ransac_py
      COMMAND ${CMAKE_COMMAND} -E env
              PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
              PYTHONPATH=$<TARGET_FILE_DIR:floor_ransac_py>
              ${Python3_EXECUTABLE} -m pytest -q
              ${CMAKE_CURRENT_SOURCE_DIR}/test/test_floor_ransac_py.py
              --junit-xml=${CMAKE_BINARY_DIR}/test_results/${PROJECT_NAME}/test_floor_ransac_py.xunit.xml
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    set_tests_properties(test_floor_ransac_py PROPERTIES TIMEOUT 180)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${CORE_LIB} ${PROJECT_LIB})
ament_package()
