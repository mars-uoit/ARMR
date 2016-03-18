#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include "radbot_processor/sampleAction.h"
#include "radbot_processor/psoAction.h"

ros::Subscriber sub;
ros::Publisher pub;
void moveBaseCB(const move_base_msgs::MoveBaseActionResultConstPtr ptr);

actionlib::SimpleActionClient<radbot_processor::sampleAction> * sampler;

int main(int argc, char** argv) {
    ros::init(argc, argv, "radbot_control");
    ros::NodeHandle nh("~");
    sub = nh.subscribe<move_base_msgs::MoveBaseActionResult>(
            "/move_base/result", 10, &moveBaseCB);
    pub = nh.advertise<geometry_msgs::Twist>( "/cmd_vel/maskable", 1 );

    sampler = new actionlib::SimpleActionClient<radbot_processor::sampleAction>("process_sampler", true);
    sampler->waitForServer();

    ROS_INFO("Control running");
    ros::Rate rate(10.0);
    while (ros::ok()) {
        //ROS_WARN("Here");
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void moveBaseCB(const move_base_msgs::MoveBaseActionResultConstPtr ptr) {
    ROS_DEBUG("Actual status: %s", ptr->status.text.c_str());
    if (ptr->status.status >= 2 && ptr->status.status < 4) {
        // get sample
        ROS_WARN("Getting Sample");
        radbot_processor::sampleGoal goal;
        goal.samples = 10;
        geometry_msgs::Twist msg;
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;
        pub.publish(msg);
        sampler->sendGoal(goal);
        while(!sampler->getState().isDone()){
            pub.publish(msg);
        }

    }
}
