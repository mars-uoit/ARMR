#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ursa_driver/ursa_counts.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform rad_source1;
  tf::Transform rad_source2;
  tf::Transform rad_source3;
  tf::Transform rad_source4;

  node.param("x1", 10);

  ros::Rate rate(10.0);
  while (node.ok())
  {
    rad_source1.setOrigin( tf::Vector3(3.0, 0.0, 0.0) );
    rad_source1.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(rad_source1, ros::Time::now(), "odom", "rad_source1"));

    rad_source2.setOrigin( tf::Vector3(5.0, 5.0, 0.0) );
    rad_source2.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(rad_source2, ros::Time::now(), "odom", "rad_source2"));

    rad_source3.setOrigin( tf::Vector3(0.0, 5.0, 0.0) );
    rad_source3.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(rad_source3, ros::Time::now(), "odom", "rad_source3"));

    rad_source4.setOrigin( tf::Vector3(-5.0, -5.0, 0.0) );
    rad_source4.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(rad_source4, ros::Time::now(), "odom", "rad_source4"));

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};