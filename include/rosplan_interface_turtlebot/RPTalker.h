#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "geometry_msgs/Twist.h"

#ifndef KCL_talker
#define KCL_talker

/**
 * This file defines the RPTalker class.
 * RPTalker is used to send strings to espeak.
 * It can state all action names as they are dispatched.
 */
namespace KCL_rosplan {

	class RPTalker
	{

	private:
	public:

		/* constructor */
		RPTalker(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
