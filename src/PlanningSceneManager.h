/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PlanningSceneManager.h
 * Author: bing
 *
 */

#ifndef PLANNINGSCENEMANAGER_H
#define PLANNINGSCENEMANAGER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <moveit_msgs/PlanningSceneWorld.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>

#include <grasping_msgs/Object.h>

#include <grasping_msgs/FitPrimitivesAction.h>
#include <grasping_msgs/FitPrimitivesGoal.h>
#include <grasping_msgs/FitPrimitivesActionGoal.h>

#include <planning_scene_manager_msgs/Segmentation.h>
#include <planning_scene_manager_msgs/SegmentationResponse.h>
#include <planning_scene_manager_msgs/PlanningSceneManagerRequestAction.h>
#include <planning_scene_manager_msgs/PlanningSceneManagerRequestActionGoal.h>
#include <planning_scene_manager_msgs/PlanningSceneManagerRequestGoal.h>


#include <string>


class PlanningSceneManager
{
private:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<planning_scene_manager_msgs::PlanningSceneManagerRequestAction> psm_server;
    actionlib::SimpleActionClient<grasping_msgs::FitPrimitivesAction> object_fitter_client;
    std::string action_name;
    std::string object_fitter_scope;

    ros::Publisher scene_publisher;
    ros::ServiceClient object_tracker_client;

    std::vector<moveit_msgs::CollisionObject> prev_objects;

public:
    PlanningSceneManager(std::string name, std::string fitter_server);
    ~PlanningSceneManager();
    void execute(const planning_scene_manager_msgs::PlanningSceneManagerRequestGoalConstPtr &goal);

};


#endif /* PLANNINGSCENEMANAGER_H */

