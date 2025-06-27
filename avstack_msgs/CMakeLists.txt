cmake_minimum_required(VERSION 3.8)
project(avstack_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AssignmentMetrics.msg"
  "msg/AgentArray.msg"
  "msg/BoxTrack2D.msg"
  "msg/BoxTrack2DArray.msg"
  "msg/BoxTrack3D.msg"
  "msg/BoxTrack3DArray.msg"
  "msg/BoxTrack3DArrayWithSender.msg"
  "msg/BoxTrack3DArrayWithSenderArray.msg"
  "msg/IdList.msg"
  "msg/ObjectState.msg"
  "msg/ObjectStateArray.msg"
  "msg/ObjectStateArrayWithSender.msg"
  "msg/ObjectStateArrayWithSenderArray.msg"
  "msg/XyzTrack.msg"
  "msg/XyzTrackArray.msg"
  DEPENDENCIES std_msgs geometry_msgs vision_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
