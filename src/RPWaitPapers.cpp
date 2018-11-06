//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 02/11/18.
//

#include <rosplan_interface_turtlebot/RPWaitPapers.h>

#include "random.h"
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
        ros::ServiceClient getPropsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/propositions");
        getPropsClient.waitForExistence(ros::Duration(60));
        busy_pub = nh.advertise<diagnostic_msgs::KeyValue>("/isbusy_mock", 1);
        somebody_pub = nh.advertise<diagnostic_msgs::KeyValue>("/someodyat_mock", 1);
        papers_in =  nh.subscribe("/papers_loaded", 1, &RPWaitPapers::papers_cb, this);
        papers_loaded = false;
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
        std::string robot_at = attrSrv.response.attributes[0].values[0].value;
        diagnostic_msgs::KeyValue kv;
        kv.key = robot_at;

        ros::Time start_t = ros::Time::now();
        ros::Duration timeout_t;
        bool printer_busy = busy(robot_at);
        if (printer_busy) timeout_t = ros::Duration(busy_wait_timeout);
        else timeout_t = ros::Duration(timeout);

        bool found_somebody = printer_busy or somebody_at(robot_at);
        if (found_somebody) kv.value = "yes";
        else kv.value = "no";
        somebody_pub.publish(kv);
        if (printer_busy) kv.value = "yes";
        else kv.value = "no";
        busy_pub.publish(kv);

        ros::Rate r(10);
        while (ros::Time::now()-start_t < timeout_t) {
            if (found_somebody && papers_loaded) return true;
            ros::spinOnce();
            r.sleep();
        }
        // complete the action
        ROS_INFO("KCL: (%s) TUTORIAL Action completing.", msg->name.c_str());
        if (ros::Time::now()-start_t > timeout_t) {
            ROS_INFO("KCL: (%s) TIMEOUTED! Action failed.", msg->name.c_str());
            return false;
        }


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
    }
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {
    ros::init(argc, argv, "rosplan_wait_papers_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    // create PDDL action subscriber
    KCL_rosplan::RPWaitPapers rpti(nh);

    rpti.runActionInterface();

    return 0;
}