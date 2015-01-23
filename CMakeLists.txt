cmake_minimum_required(VERSION 2.8.3)
project(action_server)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    tue_config
    cb_planner_msgs_srvs
    ed
)

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
# add_message_files(
#    FILES
#    message1.msg
#    ...
# )

# Generate services
add_service_files(
   FILES
   AddAction.srv
   GetActionStatus.srv
   RegisterActionServer.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
#   DEPENDENCIES
#    geometry_msgs
#    ...
)

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
	components
    ${catkin_INCLUDE_DIRS}
)

add_library(action_server
    src/uuid.cpp
    src/server.cpp
    src/action.cpp
    src/action_factory.cpp
)
target_link_libraries(action_server ${catkin_LIBRARIES})

add_executable(action_server_main
    src/main.cpp

    # Navigation
    components/navigate_to.cpp

    # Manipulation
    components/pick_up.cpp
    components/manipulation/joint_trajectory.cpp
    components/manipulation/amigo_joint_trajectory_client.cpp
)

target_link_libraries(action_server_main action_server ${catkin_LIBRARIES})
add_dependencies(action_server_main ${PROJECT_NAME}_generate_messages_cpp)

# ------------------------------------------------------------------------------------------------
#                                             EXAMPLES
# ------------------------------------------------------------------------------------------------

add_executable(act_example
    examples/example.cpp
	examples/simple_action_factory.cpp
	examples/simple_action.cpp
)
target_link_libraries(act_example action_server)
