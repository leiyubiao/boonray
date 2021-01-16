cmake_minimum_required(VERSION 2.8.3)
project(gnss_pos)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  pos_xy.msg
)

generate_messages(DEPENDENCIES std_msgs)


catkin_package(
    DEPENDS
    CATKIN_DEPENDS message_runtime
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


