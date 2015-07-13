/*
 * main.cc
 *
 *  Created on: Jun 26, 2015
 *      Author: hosmar
 */

#include "ros/ros.h"
#include "tf/transform_listener.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "aar_mobile_base");
  ros::NodeHandle nh;
  std::vector<tf::StampedTransform> locations;
  //add in service listeners

  tf::TransformListener listener;

/*  while (ros::ok())
  {
    tf::StampedTransform transform;

    try
    {
      listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    if (sqrt(
        pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2))
        > 2)
    {
      //call sample routine
    }

      ros::spinOnce();
  }*/

}
