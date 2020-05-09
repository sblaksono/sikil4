cmake_minimum_required(VERSION 2.8.3)
project(sikil4)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs message_generation)
add_service_files(FILES Task.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS rospy std_msgs)

