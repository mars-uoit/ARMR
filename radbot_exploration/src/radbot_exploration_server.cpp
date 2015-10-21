#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>

#include <radbot_exploration/ExploreTaskAction.h>

#include <tf/transform_listener.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <radbot_exploration/geometry_tools.h>

namespace radbot_exploration{

/**
 * @brief Server for frontier exploration action, runs the state machine associated with a
 * structured frontier exploration task and manages robot movement through move_base.
 */
class RadbotExplorationServer
{

public:

    /**
     * @brief Constructor for the server, sets up this node's ActionServer for exploration and ActionClient to move_base for robot movement.
     * @param name Name for SimpleActionServer
     */
    RadbotExplorationServer(std::string name) :
        tf_listener_(ros::Duration(10.0)),
        private_nh_("~"),
        as_(nh_, name, boost::bind(&RadbotExplorationServer::executeCb, this, _1), false),
        move_client_("move_base",true)
    {
        private_nh_.param<double>("goal_aliasing", goal_aliasing_, 0.1);
        private_nh_.param<double>("row_width", row_width_, 1.5);
        private_nh_.param<double>("padding", padding_, 0.5);
        private_nh_.param<std::string>("global_frame", global_frame_, "gps");

        as_.registerPreemptCallback(boost::bind(&RadbotExplorationServer::preemptCb, this));
        as_.start();
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    tf::TransformListener tf_listener_;
    actionlib::SimpleActionServer<radbot_exploration::ExploreTaskAction> as_;

    std::string global_frame_;
    double goal_aliasing_, row_width_, padding_;
    bool success_, moving_, centering_;


    boost::mutex move_client_lock_;
    radbot_exploration::ExploreTaskFeedback feedback_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_client_;
    std::vector <geometry_msgs::Pose> goals_;
    std::vector<geometry_msgs::Pose>::iterator goalsIt_;
    move_base_msgs::MoveBaseGoal move_client_goal_;

    /**
     * @brief Execute callback for actionserver, run after accepting a new goal
     * @param goal ActionGoal containing boundary of area to explore, and a valid centerpoint for the area.
     */

    void executeCb(const radbot_exploration::ExploreTaskGoalConstPtr &goal)
    {

        success_ = false;
        moving_ = false;

       //wait for move_base
        if(!move_client_.waitForServer()){
            as_.setAborted();
            return;
        }

        // generate fundamental vectors.

        geometry_msgs::Polygon polygon = goal->explore_boundary.polygon;
        geometry_msgs::Pose right_unit, left_unit, right_basis, left_basis, temp_pose;
        geometry_msgs::Point32 bottom_unit;

        
        double length = pointsDistance(polygon.points[2], polygon.points[1]);
        right_unit.position.x = (polygon.points[2].x-polygon.points[1].x)/length;
        right_unit.position.y = (polygon.points[2].y-polygon.points[1].y)/length;
        right_unit.position.z = (polygon.points[2].z-polygon.points[1].z)/length;
        

        double yaw = atan2(right_unit.position.y, right_unit.position.x);
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), right_unit.orientation);

        length = pointsDistance(polygon.points[3], polygon.points[0]);
        left_unit.position.x = (polygon.points[3].x-polygon.points[0].x)/length;
        left_unit.position.y = (polygon.points[3].y-polygon.points[0].y)/length;
        left_unit.position.z = (polygon.points[3].z-polygon.points[0].z)/length;
        
        yaw = atan2(left_unit.position.y, left_unit.position.x);
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), left_unit.orientation);
        
        length = pointsDistance(polygon.points[1], polygon.points[0]);
        bottom_unit.x = (polygon.points[1].x-polygon.points[0].x)/length;
        bottom_unit.y = (polygon.points[1].y-polygon.points[0].y)/length;
        bottom_unit.z = (polygon.points[1].z-polygon.points[0].z)/length;

        
        right_basis.position.x = polygon.points[1].x + (right_unit.position.x - bottom_unit.x)*padding_;
        right_basis.position.y = polygon.points[1].y + (right_unit.position.y - bottom_unit.y)*padding_;
        right_basis.position.z = polygon.points[1].z + (right_unit.position.z - bottom_unit.z)*padding_;

        yaw = atan2((polygon.points[0].y-polygon.points[1].y), (polygon.points[0].x-polygon.points[1].x));
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), right_basis.orientation);
        
        left_basis.position.x = polygon.points[0].x + (left_unit.position.x + bottom_unit.x)*padding_;
        left_basis.position.y = polygon.points[0].y + (left_unit.position.y + bottom_unit.y)*padding_;
        left_basis.position.z = polygon.points[0].z + (left_unit.position.z + bottom_unit.z)*padding_;
        
        yaw = atan2((polygon.points[1].y-polygon.points[0].y), (polygon.points[1].x-polygon.points[0].x));
        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw), left_basis.orientation);

        //generate goals.

        int finished =0;
        int counter = 0;

        temp_pose.position.x = left_basis.position.x;
        temp_pose.position.y = left_basis.position.y;
        temp_pose.position.z = left_basis.position.z;
        temp_pose.orientation = left_basis.orientation;
        goals_.push_back(temp_pose);

        while(1){
            temp_pose.position.x = right_basis.position.x + (right_unit.position.x * row_width_ * counter);
            temp_pose.position.y = right_basis.position.y + (right_unit.position.y * row_width_ * counter);
            temp_pose.position.z = right_basis.position.z + (right_unit.position.z * row_width_ * counter);
            temp_pose.orientation = right_unit.orientation;
            goals_.push_back(temp_pose);

            counter++;
            temp_pose.position.x = right_basis.position.x + (right_unit.position.x * row_width_ * counter);
            temp_pose.position.y = right_basis.position.y + (right_unit.position.y * row_width_ * counter);
            temp_pose.position.z = right_basis.position.z + (right_unit.position.z * row_width_ * counter);
            temp_pose.orientation = right_basis.orientation;
            if(!pointInPolygon(temp_pose.position, polygon)) {
                finished = 2;
                break;
            }
            goals_.push_back(temp_pose);

            temp_pose.position.x = left_basis.position.x + (left_unit.position.x * row_width_ * counter);
            temp_pose.position.y = left_basis.position.y + (left_unit.position.y * row_width_ * counter);
            temp_pose.position.z = left_basis.position.z + (left_unit.position.z * row_width_ * counter);
            temp_pose.orientation = left_unit.orientation;
            goals_.push_back(temp_pose);
            
            counter++;
            temp_pose.position.x = left_basis.position.x + (left_unit.position.x * row_width_ * counter);
            temp_pose.position.y = left_basis.position.y + (left_unit.position.y * row_width_ * counter);
            temp_pose.position.z = left_basis.position.z + (left_unit.position.z * row_width_ * counter);
            temp_pose.orientation = left_basis.orientation;
            if(!pointInPolygon(temp_pose.position, polygon)) {
                finished = 1;
                break;
            }
            goals_.push_back(temp_pose);
        }
        if(finished == 1) {
            temp_pose.position.x = polygon.points[3].x;
            temp_pose.position.y = polygon.points[3].y;
            temp_pose.position.z = polygon.points[3].z;
            temp_pose.orientation = left_basis.orientation;
            goals_.push_back(temp_pose);

            temp_pose.position.x = polygon.points[2].x;
            temp_pose.position.y = polygon.points[2].y;
            temp_pose.position.z = polygon.points[2].z;
            temp_pose.orientation = right_basis.orientation;
            goals_.push_back(temp_pose);
        } else if (finished == 2) {
            
            temp_pose.position.x = polygon.points[2].x;
            temp_pose.position.y = polygon.points[2].y;
            temp_pose.position.z = polygon.points[2].z;
            temp_pose.orientation = right_basis.orientation;
            goals_.push_back(temp_pose);

            temp_pose.position.x = polygon.points[3].x;
            temp_pose.position.y = polygon.points[3].y;
            temp_pose.position.z = polygon.points[3].z;
            temp_pose.orientation = left_basis.orientation;
            goals_.push_back(temp_pose);
        } else{
            ROS_ERROR("Failed to finish goal array");
        }

        goalsIt_ = goals_.begin();
        
        //placeholder for next goal to be sent to move base
        geometry_msgs::PoseStamped goal_pose;

        //loop until all frontiers are explored
        ros::Rate rate(0.5);
        geometry_msgs::PoseStamped new_pose, newnew_pose;
        while(ros::ok() && as_.isActive()){



            //evaluate if robot is within exploration boundary using robot_pose in boundary frame

            tf::StampedTransform transform;
            if(!tf_listener_.waitForTransform(global_frame_, "base_link",ros::Time::now(),ros::Duration(10))) {
                ROS_ERROR_STREAM("Couldn't transform from "<<global_frame_<<" to "<< "base_link");
                return;
            }
            while(1) {
                try
                {
                    tf_listener_.lookupTransform(global_frame_, "base_link", ros::Time(0), transform);
                    break;
                }
                catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(1.0).sleep();
                }
            }
            temp_pose.position.x = transform.getOrigin().x();
            temp_pose.position.y = transform.getOrigin().y();
            temp_pose.position.z = transform.getOrigin().z();

            //check if robot is not within exploration boundary and needs to return to center of search area
            if(goal->explore_boundary.polygon.points.size() > 0 && !pointInPolygon(temp_pose.position,goal->explore_boundary.polygon)){
                
                //check if robot has explored at least one frontier, and promote debug message to warning
                if(success_){
                    ROS_WARN("Robot left exploration boundary, returning to center");
                }else{
                    ROS_DEBUG("Robot not initially in exploration boundary, traveling to center");
                }
                //get current robot position in frame of exploration center
                geometry_msgs::PointStamped eval_point;
                eval_point.header.frame_id = global_frame_;
                eval_point.point = temp_pose.position;
                if(eval_point.header.frame_id != goal->explore_center.header.frame_id){
                    geometry_msgs::PointStamped temp = eval_point;
                    tf_listener_.transformPoint(goal->explore_center.header.frame_id, temp, eval_point);
                }

                //set goal pose to exploration center
                goal_pose.header = goal->explore_center.header;
                goal_pose.pose.position = goal->explore_center.point;
                goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw( yawOfVector(eval_point.point, goal->explore_center.point) );
                centering_ = true;
   
            } else{ 
                if(centering_){
                    goal_pose = newnew_pose;
                    centering_ = false;
                    moving_ = false;
                } else if(goalsIt_ != goals_.end() && !moving_) {
                    success_ = true;
                    new_pose.header.frame_id = global_frame_;
                    new_pose.pose.position = goalsIt_->position;
                    new_pose.pose.orientation = goalsIt_->orientation;
                    if(global_frame_ != "odom")
                    /*{
                      try
                      {
                        tf_listener_.transformPose("odom", new_pose, newnew_pose);
                        newnew_pose.pose.orientation.x =0;
                        newnew_pose.pose.orientation.y =0;
                        newnew_pose.pose.orientation.z =0;
                        newnew_pose.pose.orientation.w =1;
                        new_pose = newnew_pose;
                      }
                      catch (tf::TransformException ex){
                        ROS_ERROR("%s",ex.what());
                      }
                    }*/
                    goal_pose = new_pose;
                    goalsIt_++;

                } else if(success_ && !moving_){
                    ROS_WARN("Finished exploring room");
                    as_.setSucceeded();
                    boost::unique_lock<boost::mutex> lock(move_client_lock_);
                    move_client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
                    return;

                }else if(!ros::ok()){ //search is not successful

                    ROS_ERROR("Failed exploration");
                    as_.setAborted();
                    return;
                }

            }

            //check if new goal is close to old goal, hence no need to resend
            if(!moving_ || centering_){
                ROS_DEBUG("New exploration goal");
                move_client_goal_.target_pose = goal_pose;
                boost::unique_lock<boost::mutex> lock(move_client_lock_);
                if(as_.isActive()){
                    move_client_.sendGoal(move_client_goal_, boost::bind(&RadbotExplorationServer::doneMovingCb, this, _1, _2),0,boost::bind(&RadbotExplorationServer::feedbackMovingCb, this, _1));
                    moving_ = true;
                }
                lock.unlock();
            }

            //wait for movement to finish before continuing
            if(centering_)
            {
                rate.sleep();
            }
            else {
                while(ros::ok() && as_.isActive() && moving_) {
                  move_client_goal_.target_pose = goal_pose;
                  /*boost::unique_lock<boost::mutex> lock(move_client_lock_);
                  if(as_.isActive()){
                    move_client_.sendGoal(move_client_goal_, boost::bind(&RadbotExplorationServer::doneMovingCb, this, _1, _2),0,boost::bind(&RadbotExplorationServer::feedbackMovingCb, this, _1));
                    moving_ = true;
                  }
                lock.unlock();*/
                ros::WallDuration(1).sleep();
                }
            }
            
        }

        //goal should never be active at this point
        ROS_ASSERT(!as_.isActive());

    }


    /**
     * @brief Preempt callback for the server, cancels the current running goal and all associated movement actions.
     */
    void preemptCb(){

        boost::unique_lock<boost::mutex> lock(move_client_lock_);
        move_client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
        ROS_WARN("Current exploration task cancelled");

        if(as_.isActive()){
            as_.setPreempted();
        }

    }

    /**
     * @brief Feedback callback for the move_base client, republishes as feedback for the exploration server
     * @param feedback Feedback from the move_base client
     */
    void feedbackMovingCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){

        feedback_.base_position = feedback->base_position;
        as_.publishFeedback(feedback_);

    }

    /**
     * @brief Done callback for the move_base client, checks for errors and aborts exploration task if necessary
     * @param state State from the move_base client
     * @param result Result from the move_base client
     */
    void doneMovingCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result){

        if (state == actionlib::SimpleClientGoalState::ABORTED){
            ROS_ERROR("Failed to move");
            //as_.setAborted();
            moving_ = false;
        }else if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
            moving_ = false;
        }

    }

};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explore_server");

    radbot_exploration::RadbotExplorationServer server(ros::this_node::getName());
    ros::spin();
    return 0;
}
