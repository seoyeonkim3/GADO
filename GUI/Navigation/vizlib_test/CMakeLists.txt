cmake_minimum_required(VERSION 3.0.2)
project(vizlib_test)

# 필수 catkin 및 Qt 패키지 찾기
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  geometry_msgs
)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# Meta-Object Compiler (MOC) 활성화
set(CMAKE_AUTOMOC ON)

# catkin 패키지 설정
catkin_package(
  CATKIN_DEPENDS roscpp rviz geometry_msgs
)

# catkin과 Qt의 include 디렉토리 추가
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

# 실행 파일 지정
add_executable(vizlib_test_node src/vizlib_test.cpp src/main.cpp)

# 타겟 라이브러리 링크
target_link_libraries(vizlib_test_node
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

# 설치 규칙 (필요 시)
install(TARGETS vizlib_test_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
