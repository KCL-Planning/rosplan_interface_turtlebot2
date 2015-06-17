#include "rosplan_interface_turtlebot/RPLocaliser.h"

/* The implementation of RPLocaliser.h */
namespace KCL_rosplan {

	/* constructor */
	RPLocaliser::RPLocaliser(ros::NodeHandle &nh) {

		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");

		// costmap client
		clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

		// create publishers
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10, true);
		talker_pub = nh.advertise<std_msgs::String>("/kcl_rosplan/talker", 10, true);
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

			// get pose of the robot
			geometry_msgs::PoseStamped pBase, pMap;
			pBase.header.frame_id = "/base_link";
			pBase.pose.position.x = pBase.pose.position.y = pBase.pose.position.z = 0;
			pBase.pose.orientation.x = pBase.pose.orientation.y = pBase.pose.orientation.w = 0;
			pBase.pose.orientation.z = 1;

			try {
				tfl_.waitForTransform("/base_link", "/map", ros::Time::now(), ros::Duration(1.0));
				tfl_.transformPose("/map", pBase, pMap);
			} catch (tf::TransformException ex) {
				ROS_ERROR("KCL: (Localiser) transforme error: %s", ex.what());
			}

			double d = 10;
			std::string wpName = "";
			for (std::map<std::string,geometry_msgs::PoseStamped>::iterator wit=waypoints.begin(); wit!=waypoints.end(); ++wit) {
				double vX = wit->second.pose.position.x - pMap.pose.position.x;
				double vY = wit->second.pose.position.y - pMap.pose.position.y;
				if(sqrt(vX*vX + vY*vY) < d) {
					wpName = wit->first;
					d = sqrt(vX*vX + vY*vY);
				}
			}

			// std::cout << "OUTPUT: " << wpName << std::endl;
			std_msgs::String statement;
			if(""==wpName) {
				statement.data = "I am lost";
			} else {
				std::stringstream ss;
				ss << "I am close to " << wpName << std::endl;
				statement.data = ss.str();

				std_srvs::Empty emptySrv;
				clear_costmaps_client.call(emptySrv);

				// predicate localised
				rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
				updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				updatePredSrv.request.knowledge.attribute_name = "localised";
				diagnostic_msgs::KeyValue pair;
				pair.key = "v";
				pair.value = "kenny";
				updatePredSrv.request.knowledge.values.push_back(pair);
				update_knowledge_client.call(updatePredSrv);

				// remove old robot_at
				updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
				updatePredSrv.request.knowledge.attribute_name = "robot_at";
				update_knowledge_client.call(updatePredSrv);

				// predicate robot_at
				updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				updatePredSrv.request.knowledge.attribute_name = "robot_at";
				diagnostic_msgs::KeyValue pairWP;
				pairWP.key = "wp";
				pairWP.value = wpName;
				updatePredSrv.request.knowledge.values.push_back(pairWP);
				update_knowledge_client.call(updatePredSrv);

			}
			talker_pub.publish(statement);
			ros::Rate big_rate(0.5);
			big_rate.sleep();

			ROS_INFO("KCL: (Localiser) action complete");
			big_rate.sleep();

			// publish feedback (achieved)
			fb.status = "action achieved";
			action_feedback_pub.publish(fb);
		}
	}

	/**
	 * parses a pose with yaw from strings: "[f, f, f]"
	 */
	 void RPLocaliser::parsePose(geometry_msgs::PoseStamped &pose, std::string line) {

		int curr,next;
		curr=line.find("[")+1;
		next=line.find(",",curr);

		pose.pose.position.x = (double)atof(line.substr(curr,next-curr).c_str());
		curr=next+1; next=line.find(",",curr);

		pose.pose.position.y = (double)atof(line.substr(curr,next-curr).c_str());
		curr=next+1; next=line.find(",",curr);

		pose.pose.orientation.x = 0.0;
		pose.pose.orientation.y = 0.0;
		pose.pose.orientation.w = (double)atof(line.substr(curr,next-curr).c_str());
		pose.pose.orientation.z = sqrt(1 - pose.pose.orientation.w * pose.pose.orientation.w);
	}

	bool RPLocaliser::setupRoadmap(std::string filename) {

		ros::NodeHandle nh("~");

		// clear previous roadmap from knowledge base
		ROS_INFO("KCL: (RPLocaliser) Loading roadmap from file");

		// load configuration file
		std::ifstream infile(filename.c_str());
		std::string line;
		int curr,next;
		while(!infile.eof()) {
			// read waypoint
// std::cout << line << std::endl;
			std::getline(infile, line);
			curr=line.find("[");
			std::string name = line.substr(0,curr);

			// data
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = "map";
			parsePose(pose, line);
			waypoints[name] = pose;
		}
		infile.close();
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_localisation");
		ros::NodeHandle nh("~");

		// params
		std::string filename("waypoints.txt");
		nh.param("/waypoint_file", filename, filename);

		// create PDDL action subscriber
		KCL_rosplan::RPLocaliser rplo(nh);
		rplo.setupRoadmap(filename);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPLocaliser::dispatchCallback, &rplo);
		ROS_INFO("KCL: (Localiser) Ready to receive");

		ros::spin();
		return 0;
	}
