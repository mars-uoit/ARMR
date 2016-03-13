#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

ros::Publisher publisher;

int main(int argc, char** argv){
  ros::init(argc, argv, "radbot_control");
  ros::NodeHandle nh("~");
  
  ros::Rate rate(10.0);
  while(ros::ok()){

    rate.sleep();
  }

  return 0;
}
