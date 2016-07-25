#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "ursa_driver/ursa_counts.h"

  double rad1_location_x;
  double rad1_location_y;
  double rad1_relative_position_x;
  double rad1_relative_position_y;
  double rad1_source_strength; //The strength of radition source 1
  double rad1_measured_dose; //The measured radition dose recieved from source 1

  double rad2_location_x;
  double rad2_location_y;
  double rad2_relative_position_x;
  double rad2_relative_position_y;
  double rad2_source_strength; //The strength of radition source 2
  double rad2_measured_dose; //The measured radition dose recieved from source 2

  double rad3_location_x;
  double rad3_location_y;
  double rad3_relative_position_x;
  double rad3_relative_position_y;
  double rad3_source_strength; //The strength of radition source 3
  double rad3_measured_dose; //The measured radition dose recieved from source 3

  double rad4_location_x;
  double rad4_location_y;
  double rad4_relative_position_x;
  double rad4_relative_position_y;
  double rad4_source_strength; //The strength of radition source 4
  double rad4_measured_dose; //The measured radition dose recieved from source 4

  double total_rad_dose;
  std::string global_frame;

int main(int argc, char** argv){
  ros::init(argc, argv, "rad_source_sim");
  ros::NodeHandle nh("~");
  ros::NodeHandle node("~");
  ros::Publisher publisher = nh.advertise<ursa_driver::ursa_counts>("/ursa_node/counts", 10);

  tf::TransformBroadcaster br;
  tf::Transform rad_source1;
  tf::Transform rad_source2;
  tf::Transform rad_source3;
  tf::Transform rad_source4;

  tf::TransformListener listener; 
  tf::StampedTransform rad_source1_transform;
  tf::StampedTransform rad_source2_transform;
  tf::StampedTransform rad_source3_transform;
  tf::StampedTransform rad_source4_transform;

  node.param<std::string>("global_frame", global_frame, "map");
  node.param("x1", rad1_location_x, 0.0);
  node.param("y1", rad1_location_y, 0.0);
  node.param("rad_strength1", rad1_source_strength, 0.0);

  node.param("x2", rad2_location_x, 0.0);
  node.param("y2", rad2_location_y, 0.0);
  node.param("rad_strength2", rad2_source_strength, 0.0);

  node.param("x3", rad3_location_x, 0.0);
  node.param("y3", rad3_location_y, 0.0);
  node.param("rad_strength3", rad3_source_strength, 0.0);

  node.param("x4", rad4_location_x, 0.0);
  node.param("y4", rad4_location_y, 0.0);
  node.param("rad_strength4", rad4_source_strength, 0.0);

    //******************************** TF Broadaster - End ********************************

  ros::Rate rate(1.0);
  while (node.ok())
  {
    rad_source1.setOrigin( tf::Vector3(rad1_location_x, rad1_location_y, 0.0) );
    rad_source1.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(rad_source1, ros::Time::now(), global_frame, "rad_source1"));

    rad_source2.setOrigin( tf::Vector3(rad2_location_x, rad2_location_y, 0.0) );
    rad_source2.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(rad_source2, ros::Time::now(), global_frame, "rad_source2"));

    rad_source3.setOrigin( tf::Vector3(rad3_location_x, rad3_location_y, 0.0) );
    rad_source3.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(rad_source3, ros::Time::now(), global_frame, "rad_source3"));

    rad_source4.setOrigin( tf::Vector3(rad4_location_x, rad4_location_y, 0.0) );
    rad_source4.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(rad_source4, ros::Time::now(), global_frame, "rad_source4"));

    //******************************** TF Broadaster - End ********************************


    //******************************** TF Listener - Start ********************************
    try
    {
      listener.lookupTransform("base_link", "rad_source1", ros::Time(0), rad_source1_transform);

      listener.lookupTransform("base_link", "rad_source2", ros::Time(0), rad_source2_transform);

      listener.lookupTransform("base_link", "rad_source3", ros::Time(0), rad_source3_transform);

      listener.lookupTransform("base_link", "rad_source4", ros::Time(0), rad_source4_transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    rad1_relative_position_x = rad_source1_transform.getOrigin().x();
    rad1_relative_position_y = rad_source1_transform.getOrigin().y();

    rad2_relative_position_x = rad_source2_transform.getOrigin().x();
    rad2_relative_position_y = rad_source2_transform.getOrigin().y();

    rad3_relative_position_x = rad_source3_transform.getOrigin().x();
    rad3_relative_position_y = rad_source3_transform.getOrigin().y();

    rad4_relative_position_x = rad_source4_transform.getOrigin().x();
    rad4_relative_position_y = rad_source4_transform.getOrigin().y();

    rad1_measured_dose = (1/(pow(rad1_relative_position_x,2) + pow(rad1_relative_position_y, 2)) * rad1_source_strength);
    rad2_measured_dose = (1/(pow(rad2_relative_position_x,2) + pow(rad2_relative_position_y, 2)) * rad2_source_strength);
    rad3_measured_dose = (1/(pow(rad3_relative_position_x,2) + pow(rad3_relative_position_y, 2)) * rad3_source_strength);
    rad4_measured_dose = (1/(pow(rad4_relative_position_x,2) + pow(rad4_relative_position_y, 2)) * rad4_source_strength);

    total_rad_dose =  rad1_measured_dose + rad2_measured_dose + rad3_measured_dose + rad4_measured_dose;

    ROS_DEBUG_STREAM ("Rad doses:" << "\t1: " << rad1_measured_dose << "\t2: " << rad2_measured_dose << "\t3: " << rad3_measured_dose << "\t4: " << rad4_measured_dose << "\tTotal: "<< total_rad_dose);
       
    ursa_driver::ursa_counts temp;

    temp.header.stamp = ros::Time::now();
    temp.header.frame_id = "rad_link";
    temp.counts = total_rad_dose;
    publisher.publish(temp);

    ros::spinOnce();
    rate.sleep();
  }
  //******************************** TF Listener - End ********************************
  return 0;
};
