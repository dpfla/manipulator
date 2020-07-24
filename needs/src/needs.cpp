#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>

#include <eigen3/Eigen/Eigen>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>
#include <moveit_msgs/MoveGroupActionGoal.h>

bool setJointSpacePath(std::vector<double> joint_angle, double path_time);
bool setToolControl(std::vector<double> joint_angle);
void updateRobotState();
void demoSequence();

  // Move group arm
std::string planning_group_name = "arm";
moveit::planning_interface::MoveGroupInterface* move_group_;

// Move group gripper
std::string planning_group_name2 = "gripper";
moveit::planning_interface::MoveGroupInterface* move_group2_;

int main(int argc, char **argv){
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;

	// Moveit 
	ros::init(argc, argv,"needs");
	ros::AsyncSpinner spinner(1); 
	spinner.start();

	joint_name.push_back("joint1");
	joint_name.push_back("joint2"); 
	joint_name.push_back("joint3"); 
	joint_name.push_back("joint4"); 



	if ( ! ros::master::check() )
		return false;
	
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n("");
	move_group_ = new moveit::planning_interface::MoveGroupInterface(planning_group_name);
	move_group2_ = new moveit::planning_interface::MoveGroupInterface(planning_group_name2);

	//ros::start();
	while (ros::ok())
	{
		ros::spinOnce();
		demoSequence();
		ros::spinOnce();
		updateRobotState();
	}
	
}

bool setJointSpacePath(std::vector<double> joint_angle, double path_time)
{
	ros::AsyncSpinner spinner(1); 
	spinner.start();
	// Next get the current set of joint values for the group.
	//JointModelGroup - moveit_fun ,Get a joint group from this model (by name) 
	//getCurrentState() - Get the current state of the robot. move_Group.cpp 2091
	const robot_state::JointModelGroup* joint_model_group =
	move_group_->getCurrentState()->getJointModelGroup("arm");
	  
	moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();

	//copyJointValueTargetPositions -For a given group, copy the position values of the variables
	// that make up the group into another location, in the order that the variables are found in the group. 
	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	// Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
	joint_group_positions[0] = joint_angle.at(0);  // radians
	joint_group_positions[1] = joint_angle.at(1);  // radians
	joint_group_positions[2] = joint_angle.at(2);  // radians
	joint_group_positions[3] = joint_angle.at(3);  // radians
	
	//setJointValueTarget - capture
	move_group_->setJointValueTarget(joint_group_positions);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (success == false)
	return false;

	move_group_->move();

	spinner.stop();
	
	updateRobotState();

	return true;
}

bool setToolControl(std::vector<double> joint_angle)
{
	ros::AsyncSpinner spinner(1); 
	spinner.start();
	// Next get the current set of joint values for the group.
	const robot_state::JointModelGroup* joint_model_group =
	move_group2_->getCurrentState()->getJointModelGroup("gripper");
	  
	moveit::core::RobotStatePtr current_state = move_group2_->getCurrentState();

	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	// Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
	joint_group_positions[0] = joint_angle.at(0);  // radians
	move_group2_->setJointValueTarget(joint_group_positions);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (move_group2_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (success == false)
	return false;

	move_group2_->move();

	spinner.stop();
	return true;  
}

void updateRobotState()
{
	 ros::AsyncSpinner spinner(1); 
	 spinner.start();

	  std::vector<double> jointValues = move_group_->getCurrentJointValues();
	  std::vector<double> jointValues2 = move_group2_->getCurrentJointValues();
	  std::vector<double> temp_angle;
	  temp_angle.push_back(jointValues.at(0));
	  temp_angle.push_back(jointValues.at(1));
	  temp_angle.push_back(jointValues.at(2));
	  temp_angle.push_back(jointValues.at(3));
	  temp_angle.push_back(jointValues2.at(0));

	  geometry_msgs::Pose current_pose = move_group_->getCurrentPose().pose;  
	  std::vector<double> temp_position;
	  temp_position.push_back(current_pose.position.x);
	  temp_position.push_back(current_pose.position.y);
	  temp_position.push_back(current_pose.position.z);

	  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n  / Gripper Angle: %.3lf",
         	temp_angle.at(0),
         	temp_angle.at(1),
         	temp_angle.at(2),
         	temp_angle.at(3), 
         	temp_angle.at(4));

      printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
             temp_position.at(0),
             temp_position.at(1),
             temp_position.at(2));
}

void demoSequence()
{
	int count = 0;
	std::vector<double> joint_angle;
	std::vector<double> kinematics_position;
	std::vector<double> kinematics_orientation;
	std::vector<double> gripper_value;

	switch(count){
	case 0: // home pose
		joint_angle.push_back( 0.00);
		joint_angle.push_back(-1.05);
		joint_angle.push_back( 0.35);
		joint_angle.push_back( 0.70);
		setJointSpacePath(joint_angle, 1.5);
		count ++;
		break;
	case 1: // initial pose
		joint_angle.push_back( 0.01);
		joint_angle.push_back(-0.80);
		joint_angle.push_back( 0.00);
		joint_angle.push_back( 1.90);
		setJointSpacePath(joint_angle, 1.0);
		count ++;
		break;
	case 2: // wait & open the gripper
		setJointSpacePath(joint_angle, 3.0);
		gripper_value.push_back(0.010);
		setToolControl(gripper_value);
		count ++;
		break;	
	case 3: // wait & open the gripper
		setJointSpacePath(joint_angle, 3.0);
		gripper_value.push_back(-0.010);
		setToolControl(gripper_value);
		break;
	
	} // end of switch-case

}
