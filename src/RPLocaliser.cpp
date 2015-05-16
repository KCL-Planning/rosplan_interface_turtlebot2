#include "rosplan_interface_turtlebot/RPLocaliser.h"

/* The implementation of RPLocaliser.h */
namespace KCL_rosplan {

	/* constructor */
	RPLocaliser::RPLocaliser(ros::NodeHandle &nh) {

		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");

		// create publishers
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10, true);
	}

	/* action dispatch callback */
	void RPLocaliser::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// dock the kobuki
		if(0==msg->name.compare("localise")) {

			ROS_INFO("KCL: (Localiser) action recieved");

			// publish feedback (enabled)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action enabled";
			action_feedback_pub.publish(fb);

			geometry_msgs::Twist base_cmd;
			base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
			base_cmd.angular.x = base_cmd.angular.y = 0;
			base_cmd.angular.z = 0.5;

			double start = ros::WallTime::now().toSec();
			ros::Rate rate(10.0);
			while (ros::ok() && (ros::WallTime::now().toSec() - start < 60)){ 
				ros::spinOnce();
				cmd_vel_pub.publish(base_cmd);
				rate.sleep();
			}

			// predicate
			rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
			updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
			updatePredSrv.request.knowledge.attribute_name = "localised";
			diagnostic_msgs::KeyValue pair;
			pair.key = "v";
			pair.value = "kenny";
			updatePredSrv.request.knowledge.values.push_back(pair);
			update_knowledge_client.call(updatePredSrv);

			ROS_INFO("KCL: (Localiser) action complete");

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

		ros::init(argc, argv, "rosplan_interface_localisation");
		ros::NodeHandle nh;

		// create PDDL action subscriber
		KCL_rosplan::RPLocaliser rpdo(nh);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPLocaliser::dispatchCallback, &rpdo);
		ROS_INFO("KCL: (Localiser) Ready to receive");

		ros::spin();
		return 0;
	}
