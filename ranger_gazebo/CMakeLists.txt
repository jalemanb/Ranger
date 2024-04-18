cmake_minimum_required(VERSION 3.5)

# Following directive defines the project name.
project(ranger_gazebo)


# Following directives find required packages and load their configuration.
# The 'set' directive defines a variable (e.g. 'GZ_PLUGIN_VER').
# Such variables can be used lateron in the CMakeLists.txt file.
find_package(ament_cmake REQUIRED)
find_package(ranger_description REQUIRED)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

# Harmonic
if("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Harmonic")
# Default to Garden
else()
  find_package(gz-sim7 REQUIRED)
  set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Garden")
endif()

# Following 'add_library' directive defines a library target named 'BasicSystem'.
# The 'SHARED' keyword indicates that a shared library should be compiled, and
# is followed by the list of source files for the target.
add_library(Synchro
  SHARED
  src/Synchro.cc
)

# Following 'target_include_directories' directive specifies 'include' as the
# include directory to use when compiling the 'Synchro' target.
# The 'PRIVATE' keyword specifies that this directive will populate the
# INCLUDE_DIRECTORIES property for the 'Synchro' target.
target_include_directories(
  Synchro PRIVATE include
)

# Following 'target_link_libraries' directive specifies to use the 
# gz-sim library when linking the 'Synchro' target.
# The 'PRIVATE' keyword stipulates that the gz-sim library will not
# automatically be included if the 'Synchro' target were to
# be linked to anoter target. 
# ${GZ_SIM_VER} is substituted by the value that is was set to above.
target_link_libraries(Synchro PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER})

# Following 'install' directive ensures that the compiled libraries
# of the 'Synchro' targets will be copied
# to the subfolder 'lib/ros_gz_example_gazebo' of the install directory.
install(
  TARGETS Synchro
  DESTINATION lib/${PROJECT_NAME}
)

# Following 'install' directive ensures that the 'worlds' subfolder
# will be copied to the 'share/ros_gz_example_gazebo/worlds'
# subfolder of the installation directory.
install(DIRECTORY
  launch
  worlds
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Following directives are used when testing.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Following hooks are used to ensure that the correct environment variables
# will be set by exectuting 'sourece install/setup.bash' after compilation.
# When using this template for your project, change the filenames of the
# files in the 'hooks' folder, to correspond to your project name.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

# Following directive configures ament based on the previous directives, and should
# typically be the last directive in the CMakeLists.txt file.
ament_package()
