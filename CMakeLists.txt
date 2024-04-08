cmake_minimum_required(VERSION 3.8)
project(demo_launch_mixin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# install mixins
ament_python_install_package(launch_packages)

ament_package()
