cmake_minimum_required(VERSION 3.10)
project(test)

# 设置c++版本
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")
set(CMAKE_CXX_STANDARD 20)

include_directories(
      ${PROJECT_SOURCE_DIR}/include
      /home/czq/QuadProgpp/src
      /usr/local/include/OsqpEigen
)

# 设置静态库文件目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# 动态库文件目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# 可执行文件目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

############################################################################
# matplotlib-cpp
include(GNUInstallDirs)
set(PACKAGE_NAME matplotlib_cpp)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/${PACKAGE_NAME}/cmake)

# Library target
add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp
                           INTERFACE
                           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
                           $<INSTALL_INTERFACE:include>
                           )
target_compile_features(matplotlib_cpp INTERFACE
                        cxx_std_20
                        )
# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(matplotlib_cpp INTERFACE
                      Python3::Python
                      Python3::Module
                      )
find_package(Python3 COMPONENTS NumPy)
if (Python3_NumPy_FOUND)
	target_link_libraries(matplotlib_cpp INTERFACE
	                      Python3::NumPy
	                      )
else ()
	target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif ()
install(
		TARGETS matplotlib_cpp
		EXPORT install_targets
)
###########################################################################
add_executable(test src/main.cpp 
               src/refercenceline_smooth.cpp 
               src/read_data.cpp 
               src/reference_line.cpp 
               src/matching_line.cpp
               src/extract_point.cpp
               # src/lqr_control.cpp
               # src/lqr_controller_node.cpp
               src/LQR_control.cpp)

target_link_libraries(test
   matplotlib_cpp
#    /home/czq/local/quad/lib/libquadprog.a
   osqp::osqp
   OsqpEigen::OsqpEigen  # library name
)
#############################################################################