#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <actionlib/client/simple_action_client.h>
#include <math.h>
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "geometry_msgs/Twist.h"
#include "kobuki_msgs/AutoDockingAction.h"

#ifndef KCL_docker
#define KCL_docker

/**
 * This file defines the RPDocker class.
 * RPDocker is used to call the kobuki_auto_docking control from PDDL.
 * PDDL "dock" and "undock" actions are listened for.
 */
namespace KCL_rosplan {

	class RPDocker
	{

	private:

		ros::ServiceClient update_knowledge_client;
		ros::Publisher action_feedback_pub;
		ros::Publisher cmd_vel_pub;
		actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> action_client;
		std::string name;

	public:

		/* constructor */
		RPDocker(ros::NodeHandle &nh, std::string turtlebot_name);

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
