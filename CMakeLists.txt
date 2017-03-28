cmake_minimum_required(VERSION 2.8.3)
project(action_server)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    genmsg
    actionlib_msgs
    actionlib
)

catkin_python_setup()

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate services
add_service_files(
   FILES
   AddAction.srv
   GetActionStatus.srv
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
