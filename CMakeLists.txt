cmake_minimum_required(VERSION 3.5)

project(hscfsm_bridge)

add_compile_options(-std=c++11 -D_LINUX_)

add_definitions(-D__LINUX__)

#set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
FIND_PACKAGE(HPlugin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  hirop_msgs
  message_generation
  tf
)


#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)
#如果编译不过 则把startTaskCmd.srv 放在hirop_msgs/srv ,catkin_make 重新编译一次
add_service_files(
  FILES
  startTaskCmd.srv
)
generate_messages(DEPENDENCIES std_msgs)


catkin_package(
 CATKIN_DEPENDS 
 roscpp 
 hirop_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  src
  framework
  framework/loader  
  ${hirop_msgs_INCLUDE_DIRS}
)


file(GLOB ALL_FRAMEWORK  framework/*.cpp framework/loader/*.cpp)
file(GLOB ALL_BRIDGE_SOURCE src/*.cpp)

##### HsTaskFramework 主要框架的设计
add_library(HsTASKFramework ${ALL_FRAMEWORK})

target_link_libraries(HsTASKFramework
  ${catkin_LIBRARIES}
  ${HPlugin_LIBRARIES}
  dl
)
install(TARGETS HsTASKFramework DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


##### PickPlaceTask 子状态机APP
##TASK 源文件
file(GLOB PickPlaceTask_SOURCE App/pickplacetask/*.cpp)
##TASK 头文件
include_directories(PickPlaceTask App/pickplacetask/)

add_library(PickPlaceTask ${PickPlaceTask_SOURCE})
##TASK 输出路径
#add_dependencies(PickPlaceTask ${hirop_msgs_EXPORTED_TARGETS}  ${PickPlaceTask_EXPORTED_TARGETS} )

target_link_libraries(PickPlaceTask
  ${catkin_LIBRARIES}
  ${HPlugin_LIBRARIES}
  dl
  HsTASKFramework
)
install(TARGETS  PickPlaceTask  DESTINATION ${CMAKE_BINARY_DIR}/lib)



##### VoiceCtlRobRosTask 单机握手 子状态机APP
##TASK 源文件
file(GLOB VoiceCtlRobRosTask_SOURCE App/VoiceCtlRobRosTask/*.cpp)
##TASK 头文件
include_directories(VoiceCtlRobRosTask App/VoiceCtlRobRosTask/)

add_library(VoiceCtlRobRosTask ${VoiceCtlRobRosTask_SOURCE})
##TASK 输出路径
#add_dependencies(PickPlaceTask ${hirop_msgs_EXPORTED_TARGETS}  ${PickPlaceTask_EXPORTED_TARGETS} )

target_link_libraries(VoiceCtlRobRosTask
  ${catkin_LIBRARIES}
  ${HPlugin_LIBRARIES}
  dl
  HsTASKFramework
)
install(TARGETS  VoiceCtlRobRosTask  DESTINATION ${CMAKE_BINARY_DIR}/lib)



##### RubikCubeSolve 子状态机APP
##TASK 源文件
#file(GLOB RubikCubeSolve_SOURCE App/RubikCubeSolve/*.cpp)
##TASK 头文件
#include_directories(RubikCubeSolve App/RubikCubeSolve/)

#add_library(RubikCubeSolve ${RubikCubeSolve_SOURCE})
##TASK 输出路径
#add_dependencies(PickPlaceTask ${hirop_msgs_EXPORTED_TARGETS}  ${PickPlaceTask_EXPORTED_TARGETS} )

#target_link_libraries(RubikCubeSolve
#  ${catkin_LIBRARIES}
#  ${HPlugin_LIBRARIES}
#  dl
#  HsTASKFramework
#)
#install(TARGETS  RubikCubeSolve  DESTINATION ${CMAKE_BINARY_DIR}/lib)




##### hscfsm_bridge 主要桥
##TASK 源文件
add_executable(${PROJECT_NAME} ${ALL_BRIDGE_SOURCE} ${PickPlaceTask_SOURCE})
INCLUDE_DIRECTORIES(${PROJECT_NAME}
     ${catkin_INCLUDE_DIRS}
     )
add_dependencies(${PROJECT_NAME} ${hirop_msgs_EXPORTED_TARGETS} ${hscfsm_bridge_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${HPlugin_LIBRARIES}
  dl
  HsTASKFramework
)


