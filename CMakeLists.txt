cmake_minimum_required(VERSION 2.8.3)
project(rbe3002)


find_package(catkin REQUIRED COMPONENTS
   message_generation
   rospy
)


catkin_python_setup()



catkin_package(
    CATKIN_DEPENDS message_runtime
#   INCLUDE_DIRS src/lab01
    INCLUDE_DIRS src/lab02
    INCLUDE_DIRS src/lab03
    INCLUDE_DIRS src/lab04
#   INCLUDE_DIRS src/lab05
)

add_message_files(
  FILES
  Num.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rbe3002.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()



## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
