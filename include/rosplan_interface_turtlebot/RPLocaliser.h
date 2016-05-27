#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <tf/transform_listener.h>
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#ifndef KCL_localiser
#define KCL_localiser

/**
 * This file defines the RPLocaliser class.
 * RPLocaliser is used to slowly rotate the base for localisation.
 * PDDL "localise" action is listened for.
 */
namespace KCL_rosplan {

	class RPLocaliser
	{

	private:

		tf::TransformListener tfl_;
		ros::ServiceClient update_knowledge_client;
		ros::ServiceClient clear_costmaps_client;
		ros::ServiceClient global_localization_client;
		ros::Publisher action_feedback_pub;
		ros::Publisher cmd_vel_pub;
		ros::Publisher talker_pub;

		std::map<std::string, geometry_msgs::PoseStamped> waypoints;
		std::string name;
		std::string prefix;

		void parsePose(geometry_msgs::PoseStamped &pose, std::string line);

	public:

		/* constructor */
		RPLocaliser(ros::NodeHandle &nh, std::string turtlebot_name, std::string tf_prefix);
		bool setupRoadmap(std::string filename);

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
