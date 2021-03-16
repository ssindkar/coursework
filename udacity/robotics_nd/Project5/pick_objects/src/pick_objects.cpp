#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 1.6;
  goal.target_pose.pose.position.y = -4.6;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal1");
  ac.sendGoal(goal);
  ac.waitForResult();

  // Check if the robot reached its goal
  bool goal1_reached = false;
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
  {
    goal1_reached = true;
    ROS_INFO("Hooray, the base moved to (1.6, -4.6, 0.0, 1.0)");
  } else
    ROS_INFO("The base failed to move to (1.6, -4.6, 0.0, 1.0) for some reason");

  if (goal1_reached)
  {
    ROS_INFO("Waiting for 5 seconds after reaching first goal");
    ros::Duration(5.0).sleep();
    goal.target_pose.pose.position.x = -2.75;
    goal.target_pose.pose.position.y = -9.70;
    goal.target_pose.pose.orientation.w = 1.0;
    
    ROS_INFO("Sending goal2");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray again, the base moved to (-2.75, -9.70, 0.0, 1.0)");
    else 
      ROS_INFO("The base failed to move to (-2.75, -9.70, 0.0, 1.0) for some reason");
  }
  return 0;
}
