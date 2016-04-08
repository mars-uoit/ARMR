#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include "radbot_processor/sampleAction.h"
#include "radbot_processor/psoAction.h"
#include <std_srvs/Empty.h>
#include "radbot_control/Autosample.h"
#include "radbot_control/Numsrc.h"

ros::Subscriber sub;
ros::Publisher pub;
bool automode = false;
int num_src = 1;
int particles;
int num_samples;

void getSample();
void moveBaseCB(const move_base_msgs::MoveBaseActionResultConstPtr ptr);
bool psoCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
void runPso();
bool enableCB(radbot_control::Autosample::Request &req,
              radbot_control::Autosample::Response &res);
bool manualCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool numSrcCB(radbot_control::Numsrc::Request &req,
              radbot_control::Numsrc::Response &res);

actionlib::SimpleActionClient<radbot_processor::sampleAction> * sampler;
actionlib::SimpleActionClient<radbot_processor::psoAction> * psoAc;

int main(int argc, char** argv) {
    ros::init(argc, argv, "radbot_control");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("num_particles", particles, 5000);
    pnh.param("num_samples", num_samples, 10);

    sub = pnh.subscribe<move_base_msgs::MoveBaseActionResult>(
            "/move_base/result", 10, &moveBaseCB);
    pub = pnh.advertise<geometry_msgs::Twist>("/cmd_vel/maskable", 1);
    ros::ServiceServer autoService = nh.advertiseService("autosample",
                                                         enableCB);
    ros::ServiceServer psoService = nh.advertiseService("pso_trigger", psoCB);
    ros::ServiceServer sampleService = nh.advertiseService("manual_sample",
                                                           manualCB);
    ros::ServiceServer sourcesService = nh.advertiseService("num_sources",
                                                            numSrcCB);

    sampler = new actionlib::SimpleActionClient<radbot_processor::sampleAction>(
            "process_sampler", true);
    psoAc = new actionlib::SimpleActionClient<radbot_processor::psoAction>(
            "process_pso", true);
    psoAc->waitForServer();
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
    if (ptr->status.status >= 2 && ptr->status.status < 4 && automode) {
        // get sample
        getSample();
    }
}

void getSample() {
    ROS_INFO("Control: Getting Sample");
    radbot_processor::sampleGoal goal;
    goal.samples = num_samples;
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    pub.publish(msg);
    sampler->sendGoal(goal);
    while (!sampler->getState().isDone()) {
        pub.publish(msg);
    }
    ROS_INFO("Control: Finished Getting Sample");
    //runPso();
}

bool psoCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    runPso();
    return true;
}

void runPso() {
    ROS_INFO("Control: Running PSO");
    radbot_processor::psoGoal goal;
    goal.numSrc = num_src;
    goal.particles = particles;
    psoAc->sendGoal(goal);
    psoAc->waitForResult();  //comment below line to not hang while running
    radbot_processor::psoResult state = *psoAc->getResult();
    ROS_WARN_STREAM("Control: Pso Results: " << state);
}

bool enableCB(radbot_control::Autosample::Request &req,
              radbot_control::Autosample::Response &res) {
    ROS_INFO_STREAM("Control: enable srv called");
    automode = req.data;
    res.success = true;
    return true;
}

bool manualCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    getSample();
    return true;
}

bool numSrcCB(radbot_control::Numsrc::Request &req,
              radbot_control::Numsrc::Response &res) {
    ROS_INFO_STREAM("Control: num_sources srv called");
    num_src = req.sources;
    return true;
}

