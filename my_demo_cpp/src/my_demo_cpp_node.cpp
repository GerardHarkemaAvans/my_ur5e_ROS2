#include <cstdio>
//#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("my_demo_cpp");

int main(int argc, char ** argv)
{
    (void) argc;
    (void) argv;

    printf("hello world my_demo_cpp package\n");

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("my_demo_cpp", node_options);
 
    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();
 
 
    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
    // are used interchangeably.
    static const std::string PLANNING_GROUP = "arm";

    // The
    // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
    // class can be easily set up using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // old !! move_group = new moveit::planning_interface::MoveGroupInterface("arm");//group);

    std::map<std::string, double> target = move_group.getNamedTargetValues("home");
    
    move_group.setJointValueTarget(target);


  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
  // class to add and remove collision objects in our "virtual world" scene
//  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

#if 1
//  move_group->move();

  bool succes = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (succes)
    {
    move_group->execute(my_plan);
    //move_group->asyncExecute(my_plan);
    }
#endif




    rclcpp::shutdown();
    return 0;
}

#if 0
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#if 0
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#endif

moveit::planning_interface::MoveGroupInterface* move_group;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  move_group = new moveit::planning_interface::MoveGroupInterface("arm");//group);

  std::map<std::string, double> target = move_group->getNamedTargetValues("home");

  moveit_msgs::Constraints constraints;
  std::map<std::string, double>::iterator it = target.begin();
  while(it != target.end())
  {
      moveit_msgs::JointConstraint joint_constraint;

      std::cout<<it->first<<" = "<<it->second<<std::endl;
      // Constrain the position of a joint to be within a certain bound
      joint_constraint.joint_name = it->first;

      // the bound to be achieved is [position - tolerance_below, position + tolerance_above]
      joint_constraint.position = it->second;
      joint_constraint.tolerance_above = 0.1;
      joint_constraint.tolerance_below = 0.1;

      // A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
      joint_constraint.weight = 1.0;

      constraints.joint_constraints.push_back(joint_constraint);

      it++;
  }
  move_group->setJointValueTarget(target);

  //move_group->setPathConstraints(constraints);

  //move_group->setMaxVelocityScalingFactor(0.5);
  //move_group->setMaxAccelerationScalingFactor(0.5);
  //move_group->setPlanningTime(10.0);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

#if 1
//  move_group->move();

  bool succes = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (succes)
    {
    move_group->execute(my_plan);
    //move_group->asyncExecute(my_plan);
    }
#endif



  ros::shutdown();
  return 0;
}
#endif
