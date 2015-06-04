#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>


int main(int argc, char *argv[])
{
	ros::init (argc, argv, "testing_planning_interface");
	ros::NodeHandle nh;

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

	// first let's try and get a copy of the planning interface:
	planning_interface::PlannerManager


	
    return 0;
}

