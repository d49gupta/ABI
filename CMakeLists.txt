cmake_minimum_required(VERSION 3.8)
project(ABI)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-reorder -Wno-unused-but-set-variable -Wno-class-memaccess)
endif()


add_definitions(-DTHREE_POINT)
add_definitions(-DOLD_TAGS)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

pkg_check_modules(APRILTAG REQUIRED apriltag)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)

include_directories(
  inc
  ${OpenCV_INCLUDE_DIRS}
  ${APRILTAG_INCLUDE_DIRS}
  ${MOSQUITTO_INCLUDE_DIRS}
)

file(GLOB SRCS "src/*.cpp")
add_executable(tag_detector ${SRCS})

target_link_libraries(tag_detector
  ${OpenCV_LIBS}
  ${APRILTAG_LIBRARIES}
  ${MOSQUITTO_LIBRARIES}
  pthread 
)

install(TARGETS tag_detector
  DESTINATION lib/${PROJECT_NAME}
)
