//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 31/10/18.
//

#ifndef ROSPLAN_INTERFACE_TURTLEBOT2_RPASKER_H
#define ROSPLAN_INTERFACE_TURTLEBOT2_RPASKER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"

namespace KCL_rosplan {
    class RPAsker {
    private:
        ros::NodeHandle nh_;

        ros::Publisher tts_pub;
        ros::Subscriber dispatcher;
		ros::Publisher action_feedback_pub;
        ros::ServiceClient update_knowledge_client;


        void speak(const std::string &s);
        std::string robot_name;

    public:
        RPAsker(ros::NodeHandle& nh);

        ~RPAsker() = default;

        void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg);
    };
}

#endif //ROSPLAN_INTERFACE_TURTLEBOT2_RPASKER_H
