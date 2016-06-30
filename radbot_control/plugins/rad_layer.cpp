#include<radbot_control/rad_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(radbot_control::RadLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace radbot_control
{

  RadLayer::RadLayer() {}
  RadLayer::~RadLayer() {}

  void RadLayer::onInitialize()
  {
    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    default_value_ = FREE_SPACE;
    matchSize();

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
        &RadLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
    
    current_cost_ = 0;
    counts_sub_ = nh.subscribe("/counts", 1, &RadLayer::countsCB, this);  
    
  }
  
  void RadLayer::countsCB(const ursa_driver::ursa_countsConstPtr counts){
    ROS_INFO("cost %d", current_cost_);
    current_cost_++;
    if(current_cost_ > 254) current_cost_ = 0;
    //paintCostmap(1,1,current_cost_);
  }
  
  void RadLayer::paintCostmap(int x, int y, int cost){
    int max_j = getSizeInCellsX();
    int max_i = getSizeInCellsY();
    for (int j = 0; j < max_j; j++)
    {
      for (int i = 0; i < max_i; i++)
      {
        setCost(i,j,cost);
      }
    }
  
  }


  void RadLayer::matchSize()
  {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
  }


  void RadLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
  }

  void RadLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                             double* min_y, double* max_x, double* max_y)
  {
    if (!enabled_)
      return;

        //update the whole costmap
        *min_x = getOriginX();
        *min_y = getOriginY();
        *max_x = getSizeInMetersX()+getOriginX();
        *max_y = getSizeInMetersY()+getOriginY();              
  }

  void RadLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                            int max_j)
  {
    if (!enabled_)
      return;

    for (int j = min_j; j < max_j; j++)
    {
      for (int i = min_i; i < max_i; i++)
      {
        int index = getIndex(i, j);
        if (costmap_[index] == NO_INFORMATION)
          continue;
        master_grid.setCost(i, j, costmap_[index]); 
      }
    }
  }
  
  void RadLayer::reset(){

      //reset costmap_ char array to default values
      memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));

  }

} // end namespace

