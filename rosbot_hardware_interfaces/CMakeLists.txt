cmake_minimum_required(VERSION 3.11)

# Handle superbuild first
option(USE_SUPERBUILD "Whether or not a superbuild should be invoked" ON)

if(USE_SUPERBUILD)
  project(SUPERBUILD NONE)
  include(cmake/SuperBuild.cmake)
  return()
else()
  project(husarion_ugv_hardware_interfaces)
endif()

project(rosbot_hardware_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    ament_index_cpp
    controller_interface
    hardware_interface
    rosbot_utils
    PkgConfig
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_srvs)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

set(ENV{PKG_CONFIG_PATH}
    "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")

# pkg_check_modules(LIBLELY_COAPP REQUIRED IMPORTED_TARGET liblely-coapp)
pkg_check_modules(LIBGPIOD REQUIRED IMPORTED_TARGET libgpiodcxx)

add_library(
  ${PROJECT_NAME} SHARED 
  src/rosbot_system.cpp
  src/rosbot_imu_sensor.cpp
  src/gpio_controller.cpp
  src/gpio_driver.cpp
  src/system_e_stop.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)

ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})

target_compile_definitions(${PROJECT_NAME}
                           PRIVATE "ROSBOT_HARDWARE_INTERFACES_BUILDING_DLL")

# target_compile_definitions(${PROJECT_NAME} PUBLIC
# "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"s)
pluginlib_export_plugin_description_file(hardware_interface
                                         rosbot_hardware_interfaces.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
  
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${PACKAGE_DEPENDENCIES})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Tell compiler where gpiod.hpp is installed
include_directories(${CMAKE_INSTALL_PREFIX}/include)
    
# Or more properly, per target:
target_include_directories(rosbot_hardware_interfaces PUBLIC
    ${CMAKE_INSTALL_PREFIX}/include)
    
# Also find and link the library
find_library(GPIOD_LIBRARY 
    NAMES gpiodcxx libgpiodcxx
    HINTS ${CMAKE_INSTALL_PREFIX}/lib
    )

target_link_libraries(rosbot_hardware_interfaces
    ${GPIOD_LIBRARY}
)

ament_package()
