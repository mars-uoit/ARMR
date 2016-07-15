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
    nh.param<int>("max_counts", max_rad_, 50000);
    nh.param<double>("shepard_power", shepard_, 5);
    nh.param<double>("measure_dist", min_dist_, .3);
    
    current_cost_ = 0;
    counts_sub_ = nh.subscribe("/counts", 1, &RadLayer::countsCB, this);  
    enableService_ = nh.advertiseService("heatmap_enable", &RadLayer::enableCB, this);
    
    last_measure_.setZero();
    enabled_ = true;
    
  }
  
  bool RadLayer::enableCB(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response){
  if(request.data)
  {
    enabled_ = true;
  }
  else{
    enabled_ = false;
    reset();
  }
  response.success = true;
  return true;
  }
  
  void RadLayer::countsCB(const ursa_driver::ursa_countsConstPtr counts){
    //ROS_INFO("cost %d, counts %d", current_cost_, counts->counts);
    //if (counts->counts>max_rad_) max_rad_=counts->counts;
    current_cost_ = 255*counts->counts/max_rad_;
    if(current_cost_ > 254) current_cost_ = 254;
    tf::StampedTransform robot_pose;
    tf_listener_.waitForTransform("/map", counts->header.frame_id, ros::Time(0), ros::Duration(10.0));
    tf_listener_.lookupTransform("/map", counts->header.frame_id, ros::Time(0), robot_pose);
    
    tfScalar dist = robot_pose.getOrigin().distance(last_measure_);
    
    if (dist > min_dist_ && enabled_){
    last_measure_ = robot_pose.getOrigin();
    //ROS_WARN("Rad robot pos x:%f y: %f",robot_pose.getOrigin().x(),robot_pose.getOrigin().y());
    paintCostmap(robot_pose.getOrigin().x(),robot_pose.getOrigin().y(),current_cost_);
    }
  }

  void RadLayer::paintCostmap(double x, double y, int cost){
    double res = getResolution();
    int max_i = (x + 5 - getOriginX())/res;
    int max_j = (y + 5 - getOriginY())/res;
    int min_i = (x - 5 - getOriginX())/res;
    int min_j = (y - 5 - getOriginY())/res;
    double weight=0;
    int new_cost;
    double i_meter, j_meter, distance;
    
    for (int i = min_i; i< max_i; i++)
    {
      i_meter = getOriginX() + i * res;
      for (int j = min_j; j < max_j; j++)
      {
        j_meter = getOriginY() + j * res;
        distance = pow((pow(i_meter - x, 2) + pow(j_meter - y, 2)), 0.5);
        if (distance < res * 4)
        {
          setCost(i,j,cost);
        }
        else
        {
          weight = 1/(pow(distance, shepard_)*3);
          
        }
        //if (distance > 2)
        //{
        //  weight *= .25;
        //}
        new_cost = ((weight * cost) + 5*getCost(i,j)) / ((weight + 5));
          if(new_cost > 254) new_cost = 254;
          setCost(i,j,new_cost);
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
        //*min_x = getOriginX();
        //*min_y = getOriginY();
        //*max_x = getSizeInMetersX()+getOriginX();
        //*max_y = getSizeInMetersY()+getOriginY(); 
        
        //ROS_WARN("min x: %f min : %f, max x: %f max y: %f", *min_x, *min_y, *max_x, *max_y);
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

