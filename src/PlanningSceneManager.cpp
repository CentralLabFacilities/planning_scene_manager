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

    ROS_INFO("starting psm action server");

    //start this action server
    psm_server.start();

    object_tracker_client = nh.serviceClient<planning_scene_manager_msgs::Segmentation>("/segmentation");
    ROS_INFO("started segmentation client");

    // planning_scene publisher/ subscriber
    scene_publisher =  nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ROS_INFO("started nodes for planning_scene");
    
    ROS_INFO("waiting for object_fitter");

    //wait for objectfitter to be running
    object_fitter_client.waitForServer();
    ROS_INFO("found object_fitter server");
}


PlanningSceneManager::~PlanningSceneManager(void){psm_server.shutdown();}

void PlanningSceneManager::execute(const planning_scene_manager_msgs::PlanningSceneManagerRequestGoalConstPtr &goal) {
    //process call from bonsai or sth
    ROS_INFO("PlanningSceneManager start processing request");
    //send service request to object rec
    planning_scene_manager_msgs::Segmentation seg;
    if(!object_tracker_client.call(seg)){
        ROS_ERROR("failed to connect to object segmentation service");
    }

    ROS_INFO_STREAM("got " << seg.response.objects.size() << " objects and " << seg.response.support_surfaces.size() << " planes");

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

    // clear old planning scene
    if(prev_objects.size() > 0){
        ROS_INFO("We have old objects, clearing them.");
        moveit_msgs::PlanningScene planning_scene_clear;
        planning_scene_clear.robot_state.attached_collision_objects.clear();
        planning_scene_clear.world.collision_objects.clear();
        planning_scene_clear.is_diff = true;
        planning_scene_clear.robot_state.is_diff = true;

        for(moveit_msgs::CollisionObject o : prev_objects){
            ROS_INFO_STREAM("Removing " << o.id);
            o.operation = o.REMOVE;
            planning_scene_clear.world.collision_objects.push_back(o);
        }

        scene_publisher.publish(planning_scene_clear);

        prev_objects.clear();

    } else {
        ROS_INFO("No old objects, skipping clearing.");
    }

    // get fitter result
    grasping_msgs::FitPrimitivesResultConstPtr fitter_result_ptr;
    fitter_result_ptr = object_fitter_client.getResult();
    ROS_INFO_STREAM("ObjectFitter finished with state: " << object_fitter_client.getState().toString() << " and gave back " << fitter_result_ptr.get()->objects.size() << " objects");

    moveit_msgs::PlanningScene planning_scene;

        //build collision objects
    for (grasping_msgs::Object object : fitter_result_ptr.get()->objects){
        moveit_msgs::CollisionObject fit_obj;

        fit_obj.header = object.header;
        fit_obj.id = object.name;
        fit_obj.primitives = object.primitives;
        fit_obj.primitive_poses = object.primitive_poses;
        fit_obj.operation = fit_obj.ADD;

        ROS_INFO_STREAM("adding object");

        planning_scene.world.collision_objects.push_back(fit_obj);
        prev_objects.push_back(fit_obj);
    }

    // copy planes
    for (grasping_msgs::Object object : seg.response.support_surfaces){
        moveit_msgs::CollisionObject plane;

        plane.header = object.header;
        plane.id = object.name;
        plane.primitives = object.primitives;
        plane.primitive_poses = object.primitive_poses;
        plane.operation = plane.ADD;

        ROS_INFO_STREAM("adding plane");

        planning_scene.world.collision_objects.push_back(plane);
        prev_objects.push_back(plane);
    }


    planning_scene_manager_msgs::PlanningSceneManagerRequestResult result;

    //publish changes to planning_scene
    planning_scene.is_diff = true;
    planning_scene.robot_state.is_diff = true;
    scene_publisher.publish(planning_scene);

    ROS_INFO_STREAM("sending result, storing " << prev_objects.size() << " obstacle(s)");
    psm_server.setSucceeded(result);

}
