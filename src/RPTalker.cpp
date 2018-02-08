#include "rosplan_interface_turtlebot/RPTalker.h"

/* The implementation of RPTalker.h */
namespace KCL_rosplan {

	/* constructor */
	RPTalker::RPTalker(ros::NodeHandle &nh) {
		// nothing yet...
		runCommand("espeak -v en-sc -s 150 \"I am ready\"");
	}

	/**
	 * Runs external commands
	 */
	std::string RPTalker::runCommand(std::string cmd)
	{
		std::string data;
		FILE *stream;
		char buffer[1000];
		stream = popen(cmd.c_str(), "r");
		while ( fgets(buffer, 1000, stream) != NULL )
			data.append(buffer);
		pclose(stream);
		return data;
	}

	/* action dispatch callback */
	void RPTalker::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// read out load the action name
		std::stringstream ss;
		ss << "espeak -v en-sc -s 150 \"I am doing: " << msg->name << "\"";
		runCommand(ss.str());
	}

		/* action dispatch callback */
	void RPTalker::talkerCallback(const std_msgs::String::ConstPtr& msg) {
		// read out load the action name
		std::stringstream ss;
		ss << "espeak -v en-sc -s 150 \"" << msg->data << "\"";
		runCommand(ss.str());
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_talker");
		ros::NodeHandle nh;

		// create PDDL action subscriber
		KCL_rosplan::RPTalker rpta(nh);
	
		// listen for action dispatch
		std::string adt = "default_dispatch_topic";
		nh.getParam("action_dispatch_topic", adt);
		ros::Subscriber ds = nh.subscribe(adt, 1000, &KCL_rosplan::RPTalker::dispatchCallback, &rpta);
		ros::Subscriber ts = nh.subscribe("/kcl_rosplan/talker", 1000, &KCL_rosplan::RPTalker::talkerCallback, &rpta);
		ROS_INFO("KCL: (Talker) Ready to receive");

		ros::spin();
		return 0;
	}
