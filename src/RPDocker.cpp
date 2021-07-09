#include "rosplan_interface_turtlebot/RPDocker.h"

/* The implementation of RPDocker.h */
namespace KCL_rosplan {

	/* constructor */
	RPDocker::RPDocker(ros::NodeHandle &nh, std::string turtlebot_name)
		: action_client("/dock_drive_action", true), name(turtlebot_name) {

		// create the action client
		ROS_INFO("KCL: (Docker) waiting for action server to start on /dock_drive_action");
		action_client.waitForServer();

		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");

		// create publishers
		std::string aft = "default_feedback_topic";
		nh.getParam("action_feedback_topic", aft);
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>(aft, 10, true);
		cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10, true);
	}

	/* action dispatch callback */
	void RPDocker::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// dock the kobuki
		if(0==msg->name.compare("dock")) {

			ROS_INFO("KCL: (Docker) action received - dock");

			// Check robot name
			bool right_robot = false;
			for(size_t i=0; i<msg->parameters.size(); i++) {
				if(0==msg->parameters[i].value.compare(name)) {
					right_robot = true;
				}
			}
			if(!right_robot) {
				ROS_WARN("KCL: (Docker) aborting action dispatch; handling robot %s", name.c_str());
				return;
			}

			// dispatch auto dock action
			kobuki_msgs::AutoDockingGoal goal;
			action_client.sendGoal(goal);

			// publish feedback (enabled)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = rosplan_dispatch_msgs::ActionFeedback::ACTION_ENABLED;
			action_feedback_pub.publish(fb);

			bool finished_before_timeout = action_client.waitForResult(ros::Duration(/*5*msg->duration*/50));
			if (finished_before_timeout) {

				actionlib::SimpleClientGoalState state = action_client.getState();
				ROS_INFO("KCL: (Docker) action finished: %s - dock", state.toString().c_str());

				if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {

					// add predicate
					rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
					updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
					updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
					updatePredSrv.request.knowledge.attribute_name = "docked";
					diagnostic_msgs::KeyValue pair;
					pair.key = "v";
					pair.value = name;
					updatePredSrv.request.knowledge.values.push_back(pair);
					update_knowledge_client.call(updatePredSrv);

					// remove predicate
					updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
					updatePredSrv.request.knowledge.attribute_name = "undocked";
					update_knowledge_client.call(updatePredSrv);

					ros::Rate big_rate(0.5);
					big_rate.sleep();

					// publish feedback (achieved)
					 rosplan_dispatch_msgs::ActionFeedback fb;
					fb.action_id = msg->action_id;
					fb.status = rosplan_dispatch_msgs::ActionFeedback::ACTION_SUCCEEDED_TO_GOAL_STATE;
					action_feedback_pub.publish(fb);

				} else {

					// publish feedback (failed)
					rosplan_dispatch_msgs::ActionFeedback fb;
					fb.action_id = msg->action_id;
					fb.status = rosplan_dispatch_msgs::ActionFeedback::ACTION_FAILED;
					action_feedback_pub.publish(fb);

				}

			} else {

				action_client.cancelAllGoals();

				// publish feedback (failed)
				rosplan_dispatch_msgs::ActionFeedback fb;
				fb.action_id = msg->action_id;
				fb.status = rosplan_dispatch_msgs::ActionFeedback::ACTION_FAILED;
				action_feedback_pub.publish(fb);

				ROS_INFO("KCL: (Docker) action timed out");

			}

		}

		// undock the kobuki
		else if(0==msg->name.compare("undock")) {

			ROS_INFO("KCL: (Docker) action recieved - undock");

			// publish feedback (enabled)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = rosplan_dispatch_msgs::ActionFeedback::ACTION_ENABLED;
			action_feedback_pub.publish(fb);

			geometry_msgs::Twist base_cmd;
			base_cmd.linear.y = base_cmd.angular.z = 0;
			base_cmd.linear.x = -0.15;
			int count = 0;
			ros::Rate rate(10.0);
			while (ros::ok() && count < 20) {
				ros::spinOnce();
				cmd_vel_pub.publish(base_cmd);
				rate.sleep();
				count++;
			}

			ROS_INFO("KCL: (Docker) action complete - undock");

			// add predicate
			rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
			updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			updatePredSrv.request.knowledge.attribute_name = "undocked";
			diagnostic_msgs::KeyValue pair;
			pair.key = "v";
			pair.value = name;
			updatePredSrv.request.knowledge.values.push_back(pair);
			update_knowledge_client.call(updatePredSrv);

			// remove predicate
			updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
			updatePredSrv.request.knowledge.attribute_name = "docked";
			update_knowledge_client.call(updatePredSrv);

			ros::spinOnce();
			ros::Rate big_rate(0.5);
			big_rate.sleep();

			// publish feedback (achieved)
			fb.status = rosplan_dispatch_msgs::ActionFeedback::ACTION_SUCCEEDED_TO_GOAL_STATE;
			action_feedback_pub.publish(fb);
		}
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_docker");
		ros::NodeHandle nh("~");

		std::string turtlebot_name;
		nh.param("turtlebot_name", turtlebot_name, std::string("kenny"));

		// create PDDL action subscriber
		KCL_rosplan::RPDocker rpdo(nh, turtlebot_name);

		// listen for action dispatch
		std::string adt = "default_dispatch_topic";
		nh.getParam("action_dispatch_topic", adt);
		ros::Subscriber ds = nh.subscribe(adt, 1000, &KCL_rosplan::RPDocker::dispatchCallback, &rpdo);

		ROS_INFO("KCL: (Docker) Ready to receive");

		ros::spin();
		return 0;
	}
