cmake_minimum_required(VERSION 3.0)
project(planning_scene_manager)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
			actionlib
			roscpp
			moveit_msgs
			grasping_msgs
			planning_scene_manager_msgs
		)


find_package(Boost REQUIRED COMPONENTS system thread) #?
include_directories(${Boost_INCLUDE_DIRS}) #?


include_directories(${catkin_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS
		actionlib
		roscpp
		moveit_msgs
		grasping_msgs
		planning_scene_manager_msgs
		)

set(SRC_FILES
		src/main.cpp
		src/PlanningSceneManager.h
		src/PlanningSceneManager.cpp
		)


add_executable(	planning_scene_manager
	${SRC_FILES}
)

target_link_libraries(planning_scene_manager ${catkin_LIBRARIES})