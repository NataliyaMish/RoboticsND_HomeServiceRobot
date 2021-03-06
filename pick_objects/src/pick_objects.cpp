#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  
  // Get node handle
  ros::NodeHandle nh;

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  
  // Define node parameters that indicate whether the robot reached
  // pick-up/drop-off locations
  nh.setParam("pick_up_location_reached" , false);
  nh.setParam("drop_off_location_reached", false);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Move base goal messages - pick-up and drop-off locations
  move_base_msgs::MoveBaseGoal pick_up, drop_off;

  // Define pick-up location
  //----------------------------
  // Set up the frame parameters
  pick_up.target_pose.header.frame_id = "map";
  pick_up.target_pose.header.stamp    = ros::Time::now();

  // Define a position and orientation for the robot to reach
  pick_up.target_pose.pose.position.x    = -2.3;
  pick_up.target_pose.pose.position.y    = 6.5;
  pick_up.target_pose.pose.orientation.w = 1.0;

  // Define drop-off location
  //----------------------------
  // Set up the frame parameters
  drop_off.target_pose.header.frame_id = "map";
  drop_off.target_pose.header.stamp    = ros::Time::now();

  // Define a position and orientation for the robot to reach
  drop_off.target_pose.pose.position.x    = 3.5;
  drop_off.target_pose.pose.position.y    = 0.5;
  drop_off.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot
  // to reach for pick-up
  //--------------------------------------------
  ROS_INFO("Sending pick-up location");
  ac.sendGoal(pick_up);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, reached pick-up location");
    nh.setParam("pick_up_location_reached", true);
  }
  else
    ROS_INFO("Failed to reach pick-up location for some reason");

  // Pause for 5 seconds
  ros::Duration(5).sleep();

  // Send the goal position and orientation for the robot
  // to reach for drop-off
  ROS_INFO("Sending drop-off location");
  ac.sendGoal(drop_off);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, reached drop-ff location");
    nh.setParam("drop_off_location_reached", true);
  }
  else
    ROS_INFO("Failed to reach drop-off location for some reason");

  // Pause for 2 seconds
  ros::Duration(2).sleep();


  return 0;
}
