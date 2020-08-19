#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "test_turtle_mani/Msg.h"
#include "test_turtle_mani/PoseMsg.h"
#include <sstream>

#include <eigen3/Eigen/Eigen>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>
#include <moveit_msgs/MoveGroupActionGoal.h>

#define RELEASE_SAMLL_BOX 1
#define PICK_UP_LARGE_BOX 2
#define RELEASE_LARGE_BOX 3

#define DETECT_SMALL_BOX  1
#define DETECT_LARGE_BOX 2
#define WAIT_BOT 3
#define RELEASE_BOX 4

class OpenMani
{
private:
	ros::NodeHandle n;
	std::vector<std::string> joint_name;
	int small_box_count;
	int pick_large_box_count;
	int wait_bot_count;
	int release_box_count;
	int mode;
	int count;
	int bot_ready;
	int ar_marker_id;
	std_msgs::String current_mani_state;
	std::string planning_group_name;
	std::string planning_group_name2;
	moveit::planning_interface::MoveGroupInterface* move_group_;
	moveit::planning_interface::MoveGroupInterface* move_group2_;
	ros::Subscriber kinematic_pose_sub_;
	ros::Subscriber ar_marker_sub_;
	ros::Subscriber lift_bot_state_sub_;
	ros::Publisher current_mani_state_pub_;
	
public:
	OpenMani();
	~OpenMani();

	bool setJointSpacePath(std::vector<double> kinematics_pose, double path_time);
	bool setToolControl(std::vector<double> joint_angle);
	void updateRobotState();
	bool setTaskSpacePath(std::vector<float> kinematics_pose, double path_time);
	void init_sub_pub();
	void Ar_Marker_Callback(const test_turtle_mani::Msg &msg);
	void Lift_Bot_Callback(const test_turtle_mani::Msg &msg);
	void Kinematic_Pose_Callback(const test_turtle_mani::Msg &msg);

	void publishCallback(const ros::TimerEvent&);
	void Wait_Bot();
	void Release_Box();
	void Pick_Up_Small_Box();
	void Pick_Up_Large_Box();
	void demoSequence();
};
	
