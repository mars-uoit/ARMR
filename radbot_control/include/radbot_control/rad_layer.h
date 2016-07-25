#ifndef RAD_LAYER_H_
#define RAD_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include "ursa_driver/ursa_counts.h"
#include <std_srvs/SetBool.h>

namespace radbot_control
{

class RadLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  RadLayer();
  ~RadLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();
  virtual void reset();
  
private:
  bool enableCB(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  
  void countsCB(const ursa_driver::ursa_countsConstPtr counts);
  void paintCostmap(double x, double y, int cost);
  
  ros::Subscriber counts_sub_;
  tf::TransformListener tf_listener_;
  int current_cost_;
  int max_rad_;
  double shepard_;
  double min_dist_;
  tf::Vector3 last_measure_;
  bool enabled_;
  ros::ServiceServer enableService_;
};
}
#endif
