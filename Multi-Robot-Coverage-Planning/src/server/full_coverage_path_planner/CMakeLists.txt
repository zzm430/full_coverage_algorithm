cmake_minimum_required(VERSION 3.0.2)
project(full_coverage_path_planner)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        base_local_planner
        costmap_2d
        nav_core
        geometry_msgs
        nav_msgs
        std_msgs
        std_srvs
        pluginlib
        roscpp
        roslint
        tf
        tf2
        )
include_directories(
        include
        test/include
        ${catkin_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        base_local_planner
        geometry_msgs
        nav_msgs
        std_msgs
        std_srvs
        tf2
        tf
        costmap_2d
        nav_core
        pluginlib
        roscpp
)

roslint_cpp()
roslint_python()

add_library(${PROJECT_NAME}
        src/common.cpp
        src/${PROJECT_NAME}.cpp
        src/boustrophedon_stc.cpp
        )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

add_executable(path_planner src/main.cpp)
target_link_libraries(path_planner ${catkin_LIBRARIES} ${PROJECT_NAME} ${OpenCV_LIBRARIES})
add_dependencies(path_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(coverage_progress_visualizer_node src/coverage_progress_visualizer_node.cpp)
target_link_libraries(coverage_progress_visualizer_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${OpenCV_LIBRARIES})
add_dependencies(coverage_progress_visualizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(TARGETS path_planner coverage_progress_visualizer_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

install(FILES fcpp_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

catkin_install_python(
        PROGRAMS
        scripts/optimizer.py
        scripts/multi_robot_bringup.py
        scripts/map_merge.py
        scripts/eval_progress.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch maps rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_common test/src/test_common.cpp test/src/util.cpp src/common.cpp)

    catkin_add_gtest(test_boustrophedon_stc test/src/test_boustrophedon_stc.cpp test/src/util.cpp src/boustrophedon_stc.cpp src/common.cpp src/${PROJECT_NAME}.cpp)
    add_dependencies(test_boustrophedon_stc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test_boustrophedon_stc ${catkin_LIBRARIES})

    find_package(OpenCV)
    include_directories(${OpenCV_INCLUDE_DIRS})
    target_link_libraries(test_boustrophedon_stc ${OpenCV_LIBRARIES})

    find_package(roslaunch REQUIRED)
    roslaunch_add_file_check(launch)

    roslint_add_test()
endif ()
