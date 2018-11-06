//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 02/11/18.
//

#ifndef ROSPLAN_INTERFACE_TURTLEBOT2_RPWAITPAPERS_H
#define ROSPLAN_INTERFACE_TURTLEBOT2_RPWAITPAPERS_H

#include <ros/ros.h>
#include <vector>

#include "rosplan_action_interface/RPActionInterface.h"
#include "std_msgs/Bool.h"

/**
 * This file defines an action interface created in tutorial 10.
 */
namespace KCL_rosplan {

    class RPWaitPapers: public RPActionInterface {

    private:
        bool interactive;
        std::map<std::string, float> people_distribution;
        std::map<std::string, float> busy_distribution;
        float timeout;
        float busy_wait_timeout;
        ros::ServiceClient getPropsClient;
        ros::Publisher somebody_pub;
        ros::Publisher busy_pub;
        ros::Subscriber papers_in;
        bool papers_loaded;


        bool somebody_at(const std::string& loc);
        bool busy(const std::string& loc);
        void papers_cb(std_msgs::BoolConstPtr b);
    public:

        /* constructor */
        RPWaitPapers(ros::NodeHandle &nh);

        /* listen to and process action_dispatch topic */
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg);
    };
}
#endif


#endif //ROSPLAN_INTERFACE_TURTLEBOT2_RPWAITPAPERS_H
