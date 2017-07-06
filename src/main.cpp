

#include "PlanningSceneManager.h"
#include <ros/ros.h>
#include <stdio.h>

/*
 * 
 */

int main(int argc, char** argv) {


    ros::init(argc, argv, "planning_scene_manager");

    PlanningSceneManager psm("planning_scene_manager","object_fitter");

    ros::spin();

    return 0;
    
}
