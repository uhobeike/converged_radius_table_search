cmake_minimum_required(VERSION 3.8)
project(converged_radius_table_search)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(eigen3_cmake_module)
find_package(Eigen3)

ament_auto_find_build_dependencies()
find_package(PCL REQUIRED COMPONENTS common io registration)
include_directories(${PCL_INCLUDE_DIRS})

ament_auto_add_executable(converged_radius_table_search
  src/converged_radius_table_search.cpp
)

link_directories(${PCL_LIBRARY_DIRS})
target_link_libraries(converged_radius_table_search ${PCL_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)
