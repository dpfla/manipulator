#include "test_turtle_mani.hpp"

OpenMani::OpenMani()
:n(""),
 count(0),
 mode(2)
{
	joint_name.push_back("joint1");
	joint_name.push_back("joint2"); 
	joint_name.push_back("joint3"); 
	joint_name.push_back("joint4"); 
	
	  // Move group arm
	planning_group_name = "arm";
	
	// Move group gripper
	planning_group_name2 = "gripper";
	
	move_group_ = new moveit::planning_interface::MoveGroupInterface(planning_group_name);
	move_group2_ = new moveit::planning_interface::MoveGroupInterface(planning_group_name2);
}


OpenMani::~OpenMani()
{
	if (ros::isStarted()) 
	{
		ros::shutdown();
		ros::waitForShutdown();
	}
}

bool OpenMani::setTaskSpacePath(std::vector<double> kinematics_pose, double path_time)
{
  ros::AsyncSpinner spinner(1); 
  spinner.start();

  geometry_msgs::Pose target_pose;
  target_pose.position.x = kinematics_pose.at(0);
  target_pose.position.y = kinematics_pose.at(1);
  target_pose.position.z = kinematics_pose.at(2);

  move_group_->setPositionTarget(
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success == false)
    return false;

  move_group_->move();

  spinner.stop();
  return true;
}


bool OpenMani::setToolControl(std::vector<double> joint_angle)
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


void OpenMani::updateRobotState()
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

	printf("Present Joint J1: %.2lf J2: %.2lf J3: %.2lf J4: %.2lf\n  / Gripper: %.2lf",
		temp_angle.at(0),
		temp_angle.at(1),
		temp_angle.at(2),
		temp_angle.at(3), 
		temp_angle.at(4));

	printf("Present Kinematics Position X: %.2lf Y: %.2lf Z: %.2lf\n",
		temp_position.at(0),
		temp_position.at(1),
		temp_position.at(2));
}

void OpenMani::demoSequence()
{
	std::vector<double> kinematics_position;
	std::vector<double> kinematics_orientation;
	std::vector<double> gripper_value;

	switch(count){
	case 0: // home pose
		kinematics_position.push_back( 0.27);
		kinematics_position.push_back( 0.00);
		kinematics_position.push_back( 0.085);
		setTaskSpacePath(kinematics_position, 2.0);
		count ++;
		ROS_INFO("case 0");
		break;
		
	case 1:
		kinematics_orientation.push_back(-0.01);
		setToolControl(kinematics_orientation);
		count++;
		ROS_INFO("case 1");
		break;

	case 2: // initial pose
		kinematics_position.push_back( 0.20);
		kinematics_position.push_back( -0.00);
		kinematics_position.push_back( 0.287);
		setTaskSpacePath(kinematics_position, 2.0);
		count ++;
		ROS_INFO("case 2");
		break;

	case 3:
		kinematics_orientation.push_back(0.01);
		setToolControl(kinematics_orientation);
		count = 0;
		ROS_INFO("case 3");
		break;
	} // end of switch-case

}

void OpenMani::publishCallback(const ros::TimerEvent&)
{
	if (mode == DEMO_START)
	{
		demoSequence();
	}
}

int main(int argc, char **argv){
    
	ros::init(argc, argv,"needs");
	ros::AsyncSpinner spinner(1); 
	spinner.start();
	
	OpenMani OpenMani;

	if ( ! ros::master::check() )
		return false;
	
	//ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle nh("");
	
	ROS_INFO("11");
	ros::Timer publish_timer = nh.createTimer(ros::Duration(4), &OpenMani::publishCallback, &OpenMani);
	ROS_INFO("11");
	//ros::start();
	while (ros::ok())
	{
		ros::spinOnce();
	}
	
}


