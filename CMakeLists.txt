cmake_minimum_required(VERSION 3.0.2)
project(min_udp)
set(CMAKE_CXX_STANDARD 17)

# 添加Xenomai支持
set(XENOMAI_DIR /usr/xenomai)
set(XENOMAI_INCLUDE_DIRS 
  ${XENOMAI_DIR}/include/cobalt
  ${XENOMAI_DIR}/include
  ${XENOMAI_DIR}/include/alchemy
)
set(XENOMAI_LIBRARY_DIRS ${XENOMAI_DIR}/lib)
set(XENOMAI_CFLAGS "-D_GNU_SOURCE -D_REENTRANT -fasynchronous-unwind-tables -D__COBALT__")
set(XENOMAI_LDFLAGS "-Wl,--no-as-needed -Wl,@/usr/xenomai/lib/modechk.wrappers -lalchemy -lcopperplate -L/usr/xenomai/lib -lcobalt -lmodechk -lpthread -lrt")

# 应用Xenomai编译标志
add_definitions(${XENOMAI_CFLAGS})

# 查找其他依赖
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${XENOMAI_INCLUDE_DIRS}
)

set(SRC_FILES
  src/basic_math.cpp
  src/udp_comm.cpp
  src/udp_basic.cpp
  src/motor_init.cpp
  src/rt_control.cpp
)

# 添加实时库
add_library(min_udp STATIC ${SRC_FILES})

# 链接Xenomai库，但不包含bootstrap.o
target_link_libraries(min_udp
  -L${XENOMAI_LIBRARY_DIRS} -lalchemy -lcopperplate -lcobalt -lmodechk -lpthread -lrt
)

add_executable(protect_motor test/protect_motor.cpp)
add_executable(test_init test/test_init.cpp)
add_executable(test_rt_control test/test_rt_control.cpp)

target_link_libraries(test_rt_control min_udp
  pthread ${catkin_LIBRARIES}
)


target_link_libraries(test_init min_udp
  pthread ${catkin_LIBRARIES}
)

target_link_libraries(protect_motor min_udp
  pthread ${catkin_LIBRARIES}
)


