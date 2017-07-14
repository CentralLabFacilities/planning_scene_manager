

#include "PlanningSceneManager.h"
#include <ros/ros.h>
#include <stdio.h>

/*
 * 
 */

int main(int argc, char** argv) {

//    ROS_INFO("running local version");
    ros::init(argc, argv, "planning_scene_manager");
    ROS_INFO("starting planning_scene_manager");
    PlanningSceneManager psm("planning_scene_manager","object_fitter");

    ros::spin();

    return 0;
    
}
