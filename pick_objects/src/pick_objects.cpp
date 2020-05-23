#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  float pickUp_pos[3] = {4.5, -1.5, 0.010};
  float dropOff_pos[3] = {3, 4, 0.010};
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  // initialize nodehandle 
  ros::NodeHandle n;
  ros::Publisher position_pub = n.advertise<std_msgs::String>("robot_delivery", 1000);
  ros::Rate loop_rate(10);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  //message object to be published 
  std_msgs::String msg;
  // object with the goal data
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = pickUp_pos[0];
  goal.target_pose.pose.position.y = pickUp_pos[1];
  goal.target_pose.pose.orientation.w = pickUp_pos[2];

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  // moving to pickup location 
  msg.data = "moving to pickup";
  position_pub.publish(msg);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
{
    ROS_INFO("Hooray, Robot reached pickup location!");
    msg.data = "pickup";
    position_pub.publish(msg);

  	// second goal
  	// Wait 5 sec for move_base action server to come up
    ros::Duration(5.0).sleep();

  	// Define a (2nd) position and orientation for the robot to reach
  	goal.target_pose.pose.position.x = dropOff_pos[0];
  	goal.target_pose.pose.position.y = dropOff_pos[1];
  	goal.target_pose.pose.orientation.w = 0.010;

   	// Send the goal position and orientation for the robot to reach
  	ROS_INFO("Sending goal");
  	ac.sendGoal(goal);
        msg.data = "moving to dropoff";
        position_pub.publish(msg);

  	// Wait an infinite time for the results
 	  ac.waitForResult();
    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, Robot reached dropoff location!");
      msg.data = "dropoff";
      position_pub.publish(msg);
      }
    else
      ROS_INFO("The robot failed to reach dropoff location for some reason");
} else 
		ROS_INFO("The robot failed to reach the pickup location for some reason");
  return 0;
}
