/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PlanningSceneManager.cpp
 * Author: semueller
 * 
 */

#include "PlanningSceneManager.h"


PlanningSceneManager::PlanningSceneManager(std::string name, std::string fitter_server):
        psm_server(nh, name, boost::bind(&PlanningSceneManager::execute, this, _1), false),
        object_fitter_client(fitter_server, true)
{

    //start this action server
    psm_server.start();


    //wait for objectfitter to be running
    object_fitter_client.waitForServer();

    //
    object_tracker_client = nh.serviceClient<planning_scene_manager_msgs::Segmentation>("segmented_objects");

    // planning_scene publisher/ subscriber
    scene_subscriber = nh.subscribe("planning_scene", 10, &PlanningSceneManager::sceneCallback, this);
    scene_publisher =  nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    
};


PlanningSceneManager::~PlanningSceneManager(void){};

void PlanningSceneManager::execute(const planning_scene_manager_msgs::PlanningSceneManagerRequestGoalConstPtr &goal) {
    //process call from bonsai or sth
    ROS_INFO("PlanningSceneManager start processing request");
    //send service request to object rec
    planning_scene_manager_msgs::Segmentation seg;
    if(!object_tracker_client.call(seg)){
        ROS_ERROR("failed to connect to object segmentation service");
    }

    //translate answer Segmentation to grasping_msgs::FitPrimitivesAction
    //seg -> fp
    grasping_msgs::FitPrimitivesGoal fp_goal;

    fp_goal.objects = seg.response.objects;
    fp_goal.support_surfaces = seg.response.support_surfaces;
    fp_goal.config_names = seg.response.config_names;

    //send action to ObjectFitter
    object_fitter_client.sendGoal(fp_goal);
        //check every 0.1 seconds whether object fitter ist done
    while(!object_fitter_client.waitForResult(ros::Duration(0.1)));
    ROS_INFO("ObjectFitter finished with state: %s", object_fitter_client.getState().toString().c_str());
    //translate changes to mvoeit planning scene update
    moveit_msgs::PlanningScene ps_update;
    ps_update.is_diff = true;


        //build collision objects
    for (grasping_msgs::Object object : fp_goal.objects){
        moveit_msgs::CollisionObject fit_obj;

        fit_obj.header = object.header;
        fit_obj.id = object.name;
        object_recognition_msgs::ObjectType ot;
        fit_obj.primitives = object.primitives;
        fit_obj.primitive_poses = object.primitive_poses;
        //fit_obj.meshes =  object.meshes; currently unused
        //fit_obj.mesh_poses = object.mesh_poses;
        fit_obj.operation = fit_obj.ADD;

        current_planning_scene.world.collision_objects.push_back(fit_obj);
    }

    //publish changes to planning_scene
    scene_publisher.publish(ps_update);
    ros::spinOnce();
}

void PlanningSceneManager::sceneCallback(const moveit_msgs::PlanningScene& new_ps){
    this->current_planning_scene = new_ps;
}