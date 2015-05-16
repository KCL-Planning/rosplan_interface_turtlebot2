#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "geometry_msgs/Twist.h"

#ifndef KCL_localiser
#define KCL_localiser

/**
 * This file defines the RPLocaliser class.
 * RPLocaliser is used to slowly rotate the base for localisation. It might in future have more interesting behaviour.
 * PDDL "localise" action is listened for.
 */
namespace KCL_rosplan {

	class RPLocaliser
	{

	private:

		ros::ServiceClient update_knowledge_client;
		ros::Publisher action_feedback_pub;
		ros::Publisher cmd_vel_pub;

	public:

		/* constructor */
		RPLocaliser(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
