cmake_minimum_required(VERSION 3.5)
project(action_server_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    genmsg
    actionlib_msgs
    actionlib
)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

add_service_files(
   DIRECTORY
   srv
   FILES
   GetActions.srv
)

add_action_files(
   DIRECTORY
   action
   FILES
   Task.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES actionlib_msgs
)

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package()
