#include <ros/ros.h>
#include <ros/console.h>
#include <stdio.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "radbot_processor/sampleAction.h"
#include "radbot_processor/psoAction.h"
#include <std_srvs/Empty.h>
#include "radbot_control/Autosample.h"
#include "radbot_control/Numsrc.h"
#include "frontier_exploration/ExploreTaskActionGoal.h"
#include "frontier_exploration/ExploreTaskActionResult.h"
#include <move_base_msgs/MoveBaseAction.h>
//costmap
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>


ros::Subscriber move_sub;
ros::Subscriber explore_sub;
ros::Subscriber home_sub;
geometry_msgs::PoseStamped goal_pose;
tf::TransformListener * tf_listener;
ros::Publisher pub;
ros::Publisher marker_pub;
ros::Publisher marker_text_pub;
bool automode = false;
int num_src = 1;
int particles;
int num_samples;
std::string frame;
visualization_msgs::Marker sample_marker;

void getSample();
void moveBaseCB(const move_base_msgs::MoveBaseActionResultConstPtr ptr);
bool psoCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
void runPso();
bool enableCB(radbot_control::Autosample::Request &req,
              radbot_control::Autosample::Response &res);
bool manualCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
bool numSrcCB(radbot_control::Numsrc::Request &req,
              radbot_control::Numsrc::Response &res);
bool sampleMarkerCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

void setHomeCB(const frontier_exploration::ExploreTaskActionGoalConstPtr ptr);
void goHomeCB(const frontier_exploration::ExploreTaskActionResultConstPtr ptr);

actionlib::SimpleActionClient<radbot_processor::sampleAction> * sampler;
actionlib::SimpleActionClient<radbot_processor::psoAction> * psoAc;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> * move_client;

int main(int argc, char** argv) {
    ros::init(argc, argv, "radbot_control");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("num_particles", particles, 5000);
    pnh.param("num_samples", num_samples, 10);
    pnh.param<std::string>("marker_frame", frame, "odom");

    move_sub = pnh.subscribe<move_base_msgs::MoveBaseActionResult>(
            "/move_base/result", 10, &moveBaseCB);
    explore_sub = pnh.subscribe<frontier_exploration::ExploreTaskActionGoal>(
            "/explore_server/goal", 10, &setHomeCB);
    home_sub = pnh.subscribe<frontier_exploration::ExploreTaskActionResult>(
            "/explore_server/result", 10, &goHomeCB);
    pub = pnh.advertise<geometry_msgs::Twist>("/cmd_vel/maskable", 1);
    marker_pub = pnh.advertise<visualization_msgs::Marker>(
            "visualization_marker", 1);
    marker_text_pub = pnh.advertise<visualization_msgs::MarkerArray>(
            "visualization_text", 1);

    tf_listener = new tf::TransformListener(pnh, ros::Duration(10.0), true);
    //costmap
    tf::TransformListener costmap_listener;
    boost::shared_ptr<costmap_2d::Costmap2DROS> rad_costmap_ros = 
        boost::shared_ptr<costmap_2d::Costmap2DROS>(new 
        costmap_2d::Costmap2DROS("rad_costmap", costmap_listener));
    //services
    ros::ServiceServer autoService = nh.advertiseService("autosample",
                                                         enableCB);
    ros::ServiceServer psoService = nh.advertiseService("pso_trigger", psoCB);
    ros::ServiceServer sampleService = nh.advertiseService("manual_sample",
                                                           manualCB);
    ros::ServiceServer sourcesService = nh.advertiseService("num_sources",
                                                            numSrcCB);
    ros::ServiceServer sampleMarkerService = nh.advertiseService("reset_sample_markers",
                                                            sampleMarkerCB);
    //actions
    sampler = new actionlib::SimpleActionClient<radbot_processor::sampleAction>(
            "process_sampler", true);
    psoAc = new actionlib::SimpleActionClient<radbot_processor::psoAction>(
            "process_pso", true);
    move_client = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
            "move_base", true);
    psoAc->waitForServer();
    sampler->waitForServer();
    move_client->waitForServer();

    //sample positions
    sample_marker.header.frame_id = "map";

    sample_marker.ns = "basic_shapes";
    sample_marker.id = 1;
    sample_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    sample_marker.action = visualization_msgs::Marker::ADD;

    sample_marker.pose.orientation.w = 1.0;
    sample_marker.scale.x = .25;
    sample_marker.scale.y = .25;
    sample_marker.color.r = 0.0f;
    sample_marker.color.g = 1.0f;
    sample_marker.color.b = 0.0f;
    sample_marker.color.a = 1.0;
    sample_marker.lifetime = ros::Duration(3600);

    rad_costmap_ros->resetLayers();

    ROS_INFO("Control running");
    ros::Rate rate(10.0);
    while (ros::ok()) {
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

void setHomeCB(const frontier_exploration::ExploreTaskActionGoalConstPtr ptr) {
    tf::StampedTransform robot_pose;
    tf_listener->waitForTransform(ptr->goal.explore_center.header.frame_id, "base_link", ros::Time(0), ros::Duration(1.0));
    tf_listener->lookupTransform(ptr->goal.explore_center.header.frame_id, "base_link", ros::Time(0), robot_pose);
    goal_pose.pose.position.x = robot_pose.getOrigin().x();
    goal_pose.pose.position.y = robot_pose.getOrigin().y();
    goal_pose.pose.position.z = robot_pose.getOrigin().z();
    tf::quaternionTFToMsg(robot_pose.getRotation(), goal_pose.pose.orientation);
    goal_pose.header.frame_id = ptr->goal.explore_center.header.frame_id;

    ROS_DEBUG_STREAM("set home");
}

void goHomeCB(const frontier_exploration::ExploreTaskActionResultConstPtr ptr) {
    move_base_msgs::MoveBaseGoal move_client_goal;
    move_client_goal.target_pose = goal_pose;
    move_client->sendGoal(move_client_goal);

    ROS_DEBUG_STREAM("go home");
}

void getSample() {
    ROS_INFO("Control: Getting Sample");
    radbot_processor::sampleGoal goal;
    radbot_processor::sampleResult result;
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
    result = *sampler->getResult();
    geometry_msgs::Point temp_point;
    temp_point.x = result.x;
    temp_point.y = result.y;
    temp_point.z = 0;
    sample_marker.points.push_back(temp_point);
    sample_marker.header.stamp = ros::Time::now();
    marker_pub.publish(sample_marker);
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
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1.0;
    marker.scale.x = .25;
    marker.scale.y = .25;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(3600);
    //text
    visualization_msgs::MarkerArray marker_text_array;
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
        geometry_msgs::Point temp_point;

        temp_point.x = state.params[0 + i * 3];
        temp_point.y = state.params[1 + i * 3];
        temp_point.z = 0;

        marker.points.push_back(temp_point);

        marker_text.id = i;
        marker_text.header.stamp = ros::Time::now();
        marker_text.pose.position.x = state.params[0 + i * 3];
        marker_text.pose.position.y = state.params[1 + i * 3];
        char buff[50];
        sprintf(buff, "Source #%d, CPS: %d", i+1, (int)state.params[2 + i * 3]);
        marker_text.text = buff;
        marker_text_array.markers.push_back(marker_text);
    }
    marker.header.stamp = ros::Time::now();
    marker_pub.publish(marker);
    marker_text_pub.publish(marker_text_array);
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
    marker.action = 2; //delete
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker_pub.publish(marker);
    visualization_msgs::MarkerArray marker_array;
    marker.action = 3; //delete all
    marker.ns = "basic_text";
    marker_array.markers.push_back(marker);
    marker_text_pub.publish(marker_array);
    return true;
}
bool sampleMarkerCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    visualization_msgs::Marker marker;
    marker.action = 2; //delete
    marker.ns = "basic_shapes";
    marker.id = 1;
    sample_marker.points.clear();
    marker_pub.publish(marker);
}

