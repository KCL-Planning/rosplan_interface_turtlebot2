#include "rosplan_interface_turtlebot/RPDocker.h"

/* The implementation of RPDocker.h */
namespace KCL_rosplan {

	/* constructor */
	RPDocker::RPDocker(ros::NodeHandle &nh) : action_client("/dock_drive_action", true){

		// create the action client
		ROS_INFO("KCL: (Docker) waiting for action server to start on /dock_drive_action");
		action_client.waitForServer();

		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");

		// create publishers
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10, true);
	}

	/* action dispatch callback */
	void RPDocker::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// dock the kobuki
		if(0==msg->name.compare("dock")) {

			ROS_INFO("KCL: (Docker) action recieved");

			// dispatch auto dock action
			kobuki_msgs::AutoDockingGoal goal;
			action_client.sendGoal(goal);

			// publish feedback (enabled)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action enabled";
			action_feedback_pub.publish(fb);

			bool finished_before_timeout = action_client.waitForResult(ros::Duration(5*msg->duration));
			if (finished_before_timeout) {

				actionlib::SimpleClientGoalState state = action_client.getState();
				ROS_INFO("KCL: (Docker) action finished: %s", state.toString().c_str());

				// add predicate
				rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
				updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				updatePredSrv.request.knowledge.attribute_name = "docked";
				diagnostic_msgs::KeyValue pair;
				pair.key = "v";
				pair.value = "kenny";
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
				fb.status = "action achieved";
				action_feedback_pub.publish(fb);

			} else {

				action_client.cancelAllGoals();

				// publish feedback (failed)
				rosplan_dispatch_msgs::ActionFeedback fb;
				fb.action_id = msg->action_id;
				fb.status = "action failed";
				action_feedback_pub.publish(fb);

				ROS_INFO("KCL: (Docker) action timed out");

			}

		}

		// undock the kobuki
		else if(0==msg->name.compare("undock")) {

			ROS_INFO("KCL: (Docker) action recieved");

			// publish feedback (enabled)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action enabled";
			action_feedback_pub.publish(fb);

			geometry_msgs::Twist base_cmd;
			base_cmd.linear.y = base_cmd.angular.z = 0;   
			base_cmd.linear.x = -0.25;
			int count = 0;
			ros::Rate rate(10.0);
			while (ros::ok() && count < 10) { 
				ros::spinOnce();
				cmd_vel_pub.publish(base_cmd);
				rate.sleep();
				count++;
			}

			ROS_INFO("KCL: (Localiser) action complete");

			// add predicate
			rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
			updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			updatePredSrv.request.knowledge.attribute_name = "undocked";
			diagnostic_msgs::KeyValue pair;
			pair.key = "v";
			pair.value = "kenny";
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
			fb.status = "action achieved";
			action_feedback_pub.publish(fb);
		}
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_docker");
		ros::NodeHandle nh;

		// create PDDL action subscriber
		KCL_rosplan::RPDocker rpdo(nh);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPDocker::dispatchCallback, &rpdo);
		ROS_INFO("KCL: (Docker) Ready to receive");

		ros::spin();
		return 0;
	}
