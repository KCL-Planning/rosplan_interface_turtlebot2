//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 02/11/18.
//

#include <rosplan_interface_turtlebot/RPWaitPapers.h>
#include <std_msgs/String.h>

#include <cstdlib>
#include "rosplan_interface_turtlebot/RPWaitPapers.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"

/* The implementation of RPTutorial.h */
namespace KCL_rosplan {

    /* constructor */
    RPWaitPapers::RPWaitPapers(ros::NodeHandle &nh) {
        // perform setup
        nh.param("interactive", interactive, true);
        nh.param("timeout", timeout, 60.0f);
        nh.param("busy_timeout", busy_wait_timeout, 75.0f);
        nh.getParam("people_distribution", people_distribution);
        nh.getParam("busy_distribution", busy_distribution);
        getPropsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/propositions");
        getPropsClient.waitForExistence(ros::Duration(60));
        busy_pub = nh.advertise<diagnostic_msgs::KeyValue>("/isbusy_mock", 1);
        somebody_pub = nh.advertise<diagnostic_msgs::KeyValue>("/somebodyat_mock", 1);
        papers_in =  nh.subscribe("/papers_loaded", 1, &RPWaitPapers::papers_cb, this);
        speech_pub =  nh.advertise<std_msgs::String>("/speech", 1);
        papers_loaded = false;
        update_kb = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>("/rosplan_knowledge_base/update_array");
        update_kb.waitForExistence(ros::Duration(60));
        updateKBDistributions();
    }

    /* action dispatch callback */
    bool RPWaitPapers::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // Get robot location
        rosplan_knowledge_msgs::GetAttributeService attrSrv;
        attrSrv.request.predicate_name = "robot_at";
        if (!getPropsClient.call(attrSrv)) {
            ROS_ERROR("KCL: (WaitPapers) Failed to call service %s: %s",
                      getPropsClient.getService().c_str(), attrSrv.request.predicate_name.c_str());
            return false;
        }
        if (attrSrv.response.attributes.size() == 0) return false;
        std::string robot_at;
        for (size_t i = 0; i < attrSrv.response.attributes.size(); ++i) {
            if (attrSrv.response.attributes[i].is_negative == 0) {
                robot_at = attrSrv.response.attributes[i].values[1].value;
                break;
            }
        }
        diagnostic_msgs::KeyValue kv;
        kv.key = robot_at;

        ros::Time start_t = ros::Time::now();
        ros::Duration timeout_t;
        bool printer_busy = busy(robot_at);
        if (printer_busy) timeout_t = ros::Duration(busy_wait_timeout);
        else timeout_t = ros::Duration(timeout);

        /*bool found_somebody = printer_busy or somebody_at(robot_at);
        if (found_somebody) kv.value = "yes";
        else kv.value = "no";
        somebody_pub.publish(kv);
        if (printer_busy) kv.value = "yes";
        else kv.value = "no";
        busy_pub.publish(kv);*/

        ros::Rate r(10);
        while (ros::Time::now()-start_t < timeout_t) {
            if (/*found_somebody && */papers_loaded) break;
            ros::spinOnce();
            r.sleep();
        }
        std_msgs::String speech;
        // complete the action
        ROS_INFO("KCL: (%s) Action completing.", msg->name.c_str());
        if (ros::Time::now()-start_t > timeout_t) {
            ROS_INFO("KCL: (%s) TIMEOUTED! Action failed.", msg->name.c_str());
            speech.data = "Oh, it seems there is nobody here";
            speech_pub.publish(speech);
            return false;
        }

        speech.data = "Thank you";
        speech_pub.publish(speech);
        return true;
    }

    bool RPWaitPapers::somebody_at(const std::string &loc) {
        if (interactive) {
            std::cout << "Is there somebody at " << loc << "? (y/n) ";
            char c;
            std::cin >> c;
            return c == 'y';
        }
        return (rand() % 100) <= (100 * people_distribution[loc]);
    }

    bool RPWaitPapers::busy(const std::string &loc) {
        if (interactive) {
            std::cout << "Is the printer at " << loc << " busy? (y/n) ";
            char c;
            std::cin >> c;
            return c == 'y';
        }
        return (rand() % 100) <= (100 * busy_distribution[loc]);
    }

    void RPWaitPapers::papers_cb(std_msgs::BoolConstPtr b) {
        papers_loaded = b->data;
	    ROS_INFO_STREAM("Got message: "  << papers_loaded);
    }

    void RPWaitPapers::updateKBDistributions() {
        auto kus = rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest();
        for (auto it = people_distribution.begin(); it != people_distribution.end(); ++it) {
            auto ki = rosplan_knowledge_msgs::KnowledgeItem();
            ki.knowledge_type = ki.FUNCTION;
            ki.attribute_name = "OCCUPATION_RATE";
            auto kv = diagnostic_msgs::KeyValue();
            kv.key = "w";
            kv.value = it->first;
            ki.values.push_back(kv);
            ki.function_value = it->second;
            kus.knowledge.push_back(ki);
            kus.update_type.push_back(kus.ADD_KNOWLEDGE);
        }
        for (auto it = busy_distribution.begin(); it != busy_distribution.end(); ++it) {
            auto ki = rosplan_knowledge_msgs::KnowledgeItem();
            ki.knowledge_type = ki.FUNCTION;
            ki.attribute_name = "BUSY_RATE";
            auto kv = diagnostic_msgs::KeyValue();
            kv.key = "w";
            kv.value = it->first;
            ki.values.push_back(kv);
            ki.function_value = it->second;
            kus.knowledge.push_back(ki);
            kus.update_type.push_back(kus.ADD_KNOWLEDGE);
        }
        auto srvcall = rosplan_knowledge_msgs::KnowledgeUpdateServiceArray();
        srvcall.request = kus;
        if (not update_kb.call(srvcall)) {
            ROS_ERROR("KCL (RPWaitPapers): Failed to call service to update Knowledge Base");
        }
    }
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {
    ros::init(argc, argv, "rosplan_wait_papers_interface");
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    KCL_rosplan::RPWaitPapers rpti(nh);

    rpti.runActionInterface();

    return 0;
}
