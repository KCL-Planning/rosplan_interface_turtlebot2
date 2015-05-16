#include "rosplan_interface_turtlebot/RPTalker.h"

/* The implementation of RPTalker.h */
namespace KCL_rosplan {

	/* constructor */
	RPTalker::RPTalker(ros::NodeHandle &nh) {
		// nothing yet...
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
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPTalker::dispatchCallback, &rpta);
		ROS_INFO("KCL: (Talker) Ready to receive");

		ros::spin();
		return 0;
	}
