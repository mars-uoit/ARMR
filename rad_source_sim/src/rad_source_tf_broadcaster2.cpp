#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ursa_driver/ursa_counts.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);
  while (node.ok()){
    transform.setOrigin( tf::Vector3(5.0, 5.0, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "rad_source2"));
    rate.sleep();
  }
  return 0;
};
