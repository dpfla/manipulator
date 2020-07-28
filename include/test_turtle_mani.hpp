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


#define NUM_OF_JOINT_AND_TOOL 5
#define HOME_POSE   1
#define DEMO_START  2
#define DEMO_STOP 3

class OpenMani
{
private:
	ros::NodeHandle n;
	std::vector<std::string> joint_name;
	int count;
	int mode;
	std::string planning_group_name;
	std::string planning_group_name2;
	moveit::planning_interface::MoveGroupInterface* move_group_;
	moveit::planning_interface::MoveGroupInterface* move_group2_;
	
public:
	OpenMani();
	~OpenMani();

	bool setJointSpacePath(std::vector<double> joint_angle, double path_time);
	bool setToolControl(std::vector<double> joint_angle);
	void updateRobotState();

	void publishCallback(const ros::TimerEvent&);
	void demoSequence();
};
	
