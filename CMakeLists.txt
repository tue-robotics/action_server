cmake_minimum_required(VERSION 2.8.3)
project(action_server)

find_package(catkin REQUIRED COMPONENTS
    message_generation
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

# Generate added messages and services with any dependencies listed here
generate_messages(
)

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package()
