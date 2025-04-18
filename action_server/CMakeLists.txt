cmake_minimum_required(VERSION 3.5)
project(action_server)

find_package(catkin REQUIRED COMPONENTS
    action_server_msgs
)

catkin_python_setup()

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
