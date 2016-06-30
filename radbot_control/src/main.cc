#include <ros/ros.h>
#include <stdio.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include "radbot_processor/sampleAction.h"
#include "radbot_processor/psoAction.h"
#include <std_srvs/Empty.h>
#include "radbot_control/Autosample.h"
#include "radbot_control/Numsrc.h"
//costmap
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>


ros::Subscriber sub;
ros::Publisher pub;
ros::Publisher marker_pub;
bool automode = false;
int num_src = 1;
int particles;
int num_samples;
std::string frame;
//cost map for radiation measurements
//boost::shared_ptr<costmap_2d::Costmap2DROS> rad_costmap_ros;
// tf::TransformListener tf_listener;

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
    pnh.param<std::string>("marker_frame", frame, "odom");

    sub = pnh.subscribe<move_base_msgs::MoveBaseActionResult>(
            "/move_base/result", 10, &moveBaseCB);
    pub = pnh.advertise<geometry_msgs::Twist>("/cmd_vel/maskable", 1);
    marker_pub = pnh.advertise<visualization_msgs::Marker>(
            "visualization_marker", 1);
    //costmap
    tf::TransformListener tf_listener;
    boost::shared_ptr<costmap_2d::Costmap2DROS> rad_costmap_ros = 
        boost::shared_ptr<costmap_2d::Costmap2DROS>(new 
        costmap_2d::Costmap2DROS("rad_costmap", tf_listener));

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
    
    rad_costmap_ros->resetLayers();

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

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;

    marker.ns = "basic_shapes";
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.z = 0.125;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = .25;
    marker.scale.y = .25;
    marker.scale.z = .25;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(3600);
    //text
    visualization_msgs::Marker marker_text;
    marker_text.header.frame_id = frame;
    marker_text.ns = "basic_text";
    marker_text.frame_locked = true;
    marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_text.action = visualization_msgs::Marker::ADD;
    marker_text.pose.position.z = .3;
    marker_text.scale.x = .2;
    marker_text.scale.y = .2;
    marker_text.scale.z = .2;
    marker_text.color.r = 1.0f;
    marker_text.color.g = 1.0f;
    marker_text.color.b = 1.0f;
    marker_text.color.a = 1.0;
    marker_text.lifetime = ros::Duration(3600);
    for (int i = 0; i < state.params.size() / 3; i++) {
        marker.id = i;
        marker.header.stamp = ros::Time::now();
        marker.pose.position.x = state.params[0 + i * 3];
        marker.pose.position.y = state.params[1 + i * 3];
        marker_pub.publish(marker);
        marker_text.id = i;
        marker_text.header.stamp = ros::Time::now();
        marker_text.pose.position.x = state.params[0 + i * 3];
        marker_text.pose.position.y = state.params[1 + i * 3];
        char buff[50];
        sprintf(buff, "Source #%d, CPS: %d", i+1, (int)state.params[2 + i * 3]);
        marker_text.text = buff;
        marker_pub.publish(marker_text);
    }
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
    visualization_msgs::Marker marker;
    marker.action = 3; //delete all
    marker_pub.publish(marker);
    return true;
}

