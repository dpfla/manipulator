#include "test_turtle_mani_with_kine.hpp"

OpenMani::OpenMani()
:n("OpenMani"),
 small_box_count(0),
 pick_large_box_count(0),
 wait_bot_count(0),
 release_box_count(0),
 mode(0),
 bot_ready(-1),
 arrive_home(-1),
 box_id(-1)
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
		
	box_pick_up_complete.data = 0;
	
	init();
}


OpenMani::~OpenMani()
{
	if (ros::isStarted()) 
	{
		ros::shutdown();
		ros::waitForShutdown();
	}
}

void OpenMani::init()
{
	kinematic_pose_sub_ = n.subscribe("rvecs_msg", 10, &OpenMani::Kinematic_Pose_Callback, this);
	box_id_sub_ = n.subscribe("box_id", 10, &OpenMani::Box_ID_Callback, this);
	lift_bot_state_sub_ = n.subscribe("lift_bot", 10, &OpenMani::Lift_Bot_Callback, this);
	arrive_home_sub_ = n.subscribe("arrive_home", 10, &OpenMani::Arrive_Home_Callback, this);
	box_pick_up_complete_pub_ = n.advertise<std_msgs::Int32>("box_pick_up_complete", 1000);
	current_mani_state_pub_ = n.advertise<std_msgs::Int32>("current_mani_state", 1000);
}

void OpenMani::Kinematic_Pose_Callback(const geometry_msgs::Pose &msg)
{
	kinematic_pose_sub.push_back(msg.position.x);
	kinematic_pose_sub.push_back(msg.position.y);
	kinematic_pose_sub.push_back(msg.position.z);
}

void OpenMani::Arrive_Home_Callback(const test_turtle_mani::Msg &msg){
	arrive_home = msg.id;
	ROS_INFO("sub: %d", arrive_home);
}

void OpenMani::Box_ID_Callback(const test_turtle_mani::Msg &msg)
{
	box_id = msg.id;
	ROS_INFO("sub: %d", box_id);
}

void OpenMani::Lift_Bot_Callback(const test_turtle_mani::Msg &msg)
{
	bot_ready = msg.id;
	ROS_INFO("sub: %d", bot_ready);
}

void OpenMani::Publisher()
{
	if(small_box_count == 3 && pick_large_box_count == 0 && wait_bot_count == 0 && release_box_count == 0)
		{
			current_mani_state.data = RELEASE_SAMLL_BOX;
			current_mani_state_pub_.publish(current_mani_state);
		}
		
	else if(small_box_count == 0 && pick_large_box_count == 2 && wait_bot_count == 0 && release_box_count == 0)
		{
			current_mani_state.data = PICK_UP_LARGE_BOX;
			current_mani_state_pub_.publish(current_mani_state);
		}

	else if(small_box_count == 0 && pick_large_box_count == 0 && wait_bot_count == 0 && release_box_count == 2)
		{
			current_mani_state.data = RELEASE_LARGE_BOX;
			current_mani_state_pub_.publish(current_mani_state);
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

void OpenMani::Pick_Up_Small_Box(const ros::TimerEvent&)
{
	std::vector<double> kinematics_position;
	std::vector<double> kinematics_orientation;

	switch(small_box_count){
	case 0: // home pose
		/*kinematics_position.push_back( 0.270);
		kinematics_position.push_back( 0.000);
		kinematics_position.push_back( 0.085);*/
		setTaskSpacePath(kinematic_pose_sub, 2.0);
		small_box_count ++;
		kinematic_pose_sub.clear();
		ROS_INFO("case 0");
		break;
		
	case 1:
		kinematics_orientation.push_back(-0.01);
		setToolControl(kinematics_orientation);
		ROS_INFO("case 1");
		box_pick_up_complete.data = 1;
		box_pick_up_complete_pub_.publish(box_pick_up_complete);
		small_box_count ++;
		break;
		
	case 2: 
		if(arrive_home == 1){
			kinematics_position.push_back( -0.250);
			kinematics_position.push_back(  0.054);
			kinematics_position.push_back(  0.120);
			setTaskSpacePath(kinematics_position, 2.0);
			small_box_count ++;
			ROS_INFO("case 2");
		}
		break;

	case 3:
		kinematics_orientation.push_back(0.01);
		setToolControl(kinematics_orientation);
		ROS_INFO("case 3");
		break;
	}
}

void OpenMani::Pick_Up_Large_Box(const ros::TimerEvent&)
{
	std::vector<double> kinematics_position;
	std::vector<double> kinematics_orientation;

	switch(pick_large_box_count){
	case 0: // home pose
		/*kinematics_position.push_back( 0.270);
		kinematics_position.push_back( 0.000);
		kinematics_position.push_back( 0.085);*/
		setTaskSpacePath(kinematic_pose_sub, 2.0);
		pick_large_box_count ++;
		kinematic_pose_sub.clear();
		break;
		
	case 1:
		kinematics_orientation.push_back(-0.01);
		setToolControl(kinematics_orientation);
		pick_large_box_count ++;
		break;
	
	case 2:
		kinematics_position.push_back( 0.000);
		kinematics_position.push_back( 0.000);
		kinematics_position.push_back( 0.000);
		setTaskSpacePath(kinematics_position, 2.0);
		current_mani_state.data = PICK_UP_LARGE_BOX;
		current_mani_state_pub_.publish(current_mani_state);
		mode = WAIT_BOT;
		break;
	}
}

void OpenMani::Wait_Bot(const ros::TimerEvent&)
{
	std::vector<double> kinematics_position;
	std::vector<double> kinematics_orientation;

	switch(wait_bot_count){
	case 0: // home pose
		kinematics_position.push_back( 0.00);
		kinematics_position.push_back( 0.00);
		kinematics_position.push_back( 0.00);
		setTaskSpacePath(kinematics_position, 2.0);
		wait_bot_count ++;
		ROS_INFO("wait_bot_count 0");
		break;
		
	case 1:
		kinematics_orientation.push_back(-0.01);
		setToolControl(kinematics_orientation);
		
		do{

		}while(bot_ready != READY);
		
		mode = RELEASE_BOX;
		
		ROS_INFO("wait_bot_count 1");
		bot_ready = -1;
		break;

	}
}


void OpenMani::Release_Box(const ros::TimerEvent&)
{
	std::vector<double> kinematics_position;
	std::vector<double> kinematics_orientation;

	switch(release_box_count){
	case 0: // home pose
		kinematics_position.push_back( 0.00);
		kinematics_position.push_back( 0.00);
		kinematics_position.push_back( 0.00);
		setTaskSpacePath(kinematics_position, 2.0);
		release_box_count ++;
		ROS_INFO("release_box_count 0");
		break;
		
	case 1:
		kinematics_orientation.push_back(-0.01);
		setToolControl(kinematics_orientation);	
		release_box_count ++;
		ROS_INFO("release_box_count 1");
		break;
	
	case 2:
		kinematics_position.push_back( 0.27);
		kinematics_position.push_back( 0.00);
		kinematics_position.push_back( 0.085);
		setTaskSpacePath(kinematics_position, 2.0);
		current_mani_state.data = RELEASE_LARGE_BOX;
		current_mani_state_pub_.publish(current_mani_state);
		ROS_INFO("release_box_count 2");
		break;
	}
}


void OpenMani::publishCallback(const ros::TimerEvent&)
{
	if (box_id == DETECT_SMALL_BOX)
	{
		pick_large_box_count = 0; wait_bot_count = 0; release_box_count = 0;
		ros::Timer small_box_timer = n.createTimer(ros::Duration(4), &OpenMani::Pick_Up_Small_Box, this);
		//Pick_Up_Small_Box();
	}

	else if (box_id == DETECT_LARGE_BOX)
	{
		small_box_count = 0; wait_bot_count = 0; release_box_count = 0;
		ros::Timer large_box_timer = n.createTimer(ros::Duration(4), &OpenMani::Pick_Up_Large_Box, this);
		//Pick_Up_Large_Box();
	}

	else if (mode == WAIT_BOT)
	{
		small_box_count = 0; pick_large_box_count = 0; release_box_count = 0;
		ros::Timer wait_bot_timer = n.createTimer(ros::Duration(4), &OpenMani::Wait_Bot, this);
		//Wait_Bot();
	}
	else if (mode == RELEASE_BOX)
	{
		small_box_count = 0; pick_large_box_count = 0; wait_bot_count = 0;
		ros::Timer release_box_timer = n.createTimer(ros::Duration(4), &OpenMani::Release_Box, this);
		Release_Box();
	}
}

int main(int argc, char **argv){
    
	ros::init(argc, argv,"test_turtle_mani");
	ros::AsyncSpinner spinner(1); 
	spinner.start();
	
	OpenMani Openmani;

	if ( ! ros::master::check() )
		return false;
	
	ros::NodeHandle nh("");
	
	ros::Timer publish_timer = nh.createTimer(ros::Duration(4), &OpenMani::publishCallback, &Openmani);
	
	while (ros::ok())
	{
		ros::spinOnce();
		
		Openmani.Publisher();
		
	}
}

 
