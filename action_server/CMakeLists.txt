cmake_minimum_required(VERSION 2.8.3)
project(action_server)

find_package(catkin REQUIRED COMPONENTS
    action_server_msgs
)

catkin_python_setup()

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package()
