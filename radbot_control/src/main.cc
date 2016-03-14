#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

ros::Subscriber sub;
void moveBaseCB(const move_base_msgs::MoveBaseActionResultConstPtr ptr);

int main(int argc, char** argv) {
	ros::init(argc, argv, "radbot_control");
	ros::NodeHandle nh("~");
	sub = nh.subscribe<move_base_msgs::MoveBaseActionResult>(
			"/move_base/result", 10, &moveBaseCB);

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
		ROS_INFO("Getting Sample");
	}
}
