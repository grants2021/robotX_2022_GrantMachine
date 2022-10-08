cmake_minimum_required(VERSION 2.8.3)
project(robotx_2022)

# We need erb to process the .world.erb files.
find_program(ERB_EXE_PATH erb)
if(NOT ERB_EXE_PATH)
  message(FATAL_ERROR "Could not find the `erb` tool.  Try `sudo apt-get install ruby`")
endif()

add_compile_options(-std=c++17)

# For Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# Set policy for CMake 3.1+. Use OLD policy to let FindBoost.cmake, dependency
# of gazebo, use quoted variables in if()
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 OLD)
endif()
# Use OLD policy to be able to use *_generate_services_cpp before they exist,
# without error.
if(POLICY CMP0046)
  cmake_policy(SET CMP0046 OLD)
endif()

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)
#find_package(ardupilot_gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  geographic_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  wamv_gazebo
  wave_gazebo
  xacro
  vrx_gazebo
)

########################
## Message generation ##
########################


# Python scripts setup
catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS wamv_gazebo wave_gazebo xacro gazebo_dev geographic_msgs std_msgs message_runtime vrx_gazebo
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}") 

include_directories(${PROJECT_SOURCE_DIR}
  include 
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${QT_USE_FILE}
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  /msgs
)

link_libraries(
        ${GAZEBO_LIBRARIES}
        )

link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)

add_definitions(${QT_DEFINITIONS})

set (plugins_single_header
        ArduPilotPlugin
        ArduCopterIRLockPlugin
        GimbalSmall2dPlugin
        )

add_library(ArduCopterIRLockPlugin SHARED src/ArduCopterIRLockPlugin.cc)
target_link_libraries(ArduCopterIRLockPlugin ${GAZEBO_LIBRARIES})

add_library(ArduPilotPlugin SHARED src/ArduPilotPlugin.cc)
target_link_libraries(ArduPilotPlugin ${GAZEBO_LIBRARIES})

if(${GAZEBO_MAJOR_VERSION} GREATER 7)
  # Plugin for Task Info GUI Overlay
  add_library(gui_task_widget SHARED ${headers_MOC}
         	src/gui_task_widget.cc
  )

  target_link_libraries(gui_task_widget 
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${QT_LIBRARIES}
  )
  qt5_use_modules(gui_task_widget Widgets)
  add_dependencies(gui_task_widget
    ${catkin_EXPORTED_TARGETS}
    vrx_gazebo_generate_messages_cpp)
  install(TARGETS gui_task_widget
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

install(TARGETS ArduCopterIRLockPlugin DESTINATION ${GAZEBO_PLUGIN_PATH})
install(TARGETS ArduPilotPlugin DESTINATION ${GAZEBO_PLUGIN_PATH})

install(DIRECTORY models DESTINATION ${GAZEBO_MODEL_PATH}/..)
install(DIRECTORY worlds DESTINATION ${GAZEBO_MODEL_PATH}/..)

# Install all the config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

# Install all the model files
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh")
  
# Python Scripts
catkin_install_python(PROGRAMS
    nodes/boat_twist.py
    nodes/wamv_dk_functions.py
    nodes/spare_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

