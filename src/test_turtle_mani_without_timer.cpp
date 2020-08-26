#include "test_turtle_mani.hpp"

std::vector<double> kinematic_pose_sub;
std::vector<double> kinematic_pose_check;
std::vector<double> temp_position;
int cur_time;
ros::Publisher pub_;

  // Move group arm
std::string planning_group_name = "arm";

// Move group gripper
std::string planning_group_name2 = "gripper";

moveit::planning_interface::MoveGroupInterface* move_group_ = new moveit::planning_interface::MoveGroupInterface(planning_group_name);
moveit::planning_interface::MoveGroupInterface* move_group2_ = new moveit::planning_interface::MoveGroupInterface(planning_group_name2);

double moving_time;

bool setJointSpacePath(std::vector<double> kinematics_pose, double path_time)
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
	moving_time = my_plan.trajectory_.joint_trajectory.points.rbegin()->time_from_start.toSec();
	if (success == false)
	return false;

	move_group_->move();

	spinner.stop();
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

	//moving_time = my_plan.trajectory_.joint_trajectory.points.rbegin()->time_from_start.toSec()
	if (success == false)
		return false;

	move_group2_->move();
	
	cur_time = time(0);

	spinner.stop();
	return true;  
}


void demoSequence(int count)
{
	std::vector<double> joint_angle;
	std::vector<double> kinematics_position;
	std::vector<double> kinematics_orientation;
	std::vector<double> gripper_value;
	int add_time = 0;

	switch(count){
	case 0: // home pose
		kinematics_position.push_back( 0.047 );
		kinematics_position.push_back( 0.000 );
		kinematics_position.push_back( 0.340 );
		setJointSpacePath(kinematics_position, 2.0);

		do{
			add_time = time(0);
		}while((add_time-cur_time)==4);
		
		count ++;
		ROS_INFO("case 0");
		break;
		
	case 1:
		joint_angle.push_back(-0.01);
		setToolControl(joint_angle);
		count++;
		ROS_INFO("case 1");
		break;

	/*case 2: // initial pose
		kinematics_position.push_back( 0.00);
		kinematics_position.push_back( 0.00);
		kinematics_position.push_back( 0.085);
		setJointSpacePath(kinematic_pose_sub, 2.0);
		count ++;
		ROS_INFO("case 2");
		break;

	case 3:
		joint_angle.push_back(0.01);
		setToolControl(joint_angle);
		count=0;
		ROS_INFO("case 3");
		break;*/
	}
}

int main(int argc, char **argv){
    
	ros::init(argc, argv,"test_turtle_mani");
	ros::AsyncSpinner spinner(1); 
	spinner.start();
	
	int count = 0;

	if ( ! ros::master::check() )
		return false;
	
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle nh("");
	
	ROS_INFO("11");

	while (ros::ok())
	{	
		demoSequence(count);
		ros::spinOnce();
	}
	
}

