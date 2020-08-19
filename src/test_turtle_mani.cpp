#include "test_turtle_mani.hpp"

std::vector<double> kinematic_pose_sub;
std::vector<double> temp_position;
//ros::Publisher pub_;

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

bool OpenMani::setJointSpacePath(std::vector<double> kinematics_pose, double path_time)
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

	temp_position.push_back(current_pose.position.x);
	temp_position.push_back(current_pose.position.y);
	temp_position.push_back(current_pose.position.z);

	//test_turtle_mani::PoseMsg pose_msg;
	//pose_msg.cur_x = (float)temp_position[0];
	//pose_msg.cur_y = (float)temp_position[1];
	//pose_msg.cur_z = (float)temp_position[2];
	
	//pub_.publish(pose_msg);

	printf("CurMani X: %.2lf Y: %.2lf Z: %.2lf\n",
		temp_position.at(0),
		temp_position.at(1),
		temp_position.at(2));
}

void OpenMani::demoSequence()
{
	std::vector<double> joint_angle;
	std::vector<double> kinematics_position;
	std::vector<double> kinematics_orientation;
	std::vector<double> gripper_value;

	switch(count){
	case 0: // home pose
		kinematics_position.push_back( kinematic_pose_sub[0]+temp_position[1] );
		kinematics_position.push_back( kinematic_pose_sub[1]+temp_position[2] );
		kinematics_position.push_back( kinematic_pose_sub[2]+temp_position[0] );
		setJointSpacePath(kinematics_position, 2.0);
		count ++;
		ROS_INFO("case 0");
		break;
		
	/*case 1:
		joint_angle.push_back(-0.01);
		setToolControl(joint_angle);
		count++;
		ROS_INFO("case 1");
		break;

	case 2: // initial pose
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

void OpenMani::publishCallback(const ros::TimerEvent&)
{
	//ROS_INFO("%d", kinematic_pose_sub.empty());
	if (!kinematic_pose_sub.empty())
	{
		updateRobotState();
		demoSequence();
	}
}

void PoseCallback(const test_turtle_mani::Msg &msg){
	
	std::vector<float> kinematic_pose_;
	
	kinematic_pose_.push_back(msg.t_x * -1);  //+y
	kinematic_pose_.push_back(msg.t_y);  //+z
	kinematic_pose_.push_back(msg.t_z);  //+x

	kinematic_pose_sub.assign(kinematic_pose_.begin(), kinematic_pose_.end());
}

int main(int argc, char **argv){
    
	ros::init(argc, argv,"test_turtle_mani");
	ros::AsyncSpinner spinner(1); 
	spinner.start();
	
	OpenMani OpenMani;

	if ( ! ros::master::check() )
		return false;
	
	//ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle nh("");
	
	ROS_INFO("11");
	ros::Timer publish_timer = nh.createTimer(ros::Duration(4), &OpenMani::publishCallback, &OpenMani);
	ros::Subscriber sub_ = nh.subscribe("rvecs_msg", 10, PoseCallback);
	//pub_ = nh.advertise<test_turtle_mani::PoseMsg>("cur_pose", 1000);
	ROS_INFO("11");
	//ros::start();
	while (ros::ok())
	{
		ros::spinOnce();
	}
	
}


