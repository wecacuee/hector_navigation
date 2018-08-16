//
// Created by rakesh on 30/07/18.
//

#ifndef HECTOR_NAVIGATION_INFO_GAIN_CLIENT_H
#define HECTOR_NAVIGATION_INFO_GAIN_CLIENT_H

// TODO: make these proper parameters
#define MAP_RESOLUTION 0.1         // meters/pixels
#define MAP_SIZE cv::Size(1360, 1020)
// the inflations are on the either side of the wall
#define OBSTACLE_INFLATION_SIZE 0.5   // meters

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core.hpp>

#include <hector_exploration_planner/custom_costmap_2d_ros.h>
#include <hector_exploration_planner/frontier_analysis.h>

namespace hector_exploration_planner
{

// forward declaration
class HectorExplorationPlanner;

class InfoGainClient
{
public:

  /**
   *
   * @param node_handle
   */
  InfoGainClient(ros::NodeHandle &node_handle,
                 hector_exploration_planner::HectorExplorationPlanner *planner,
                 hector_exploration_planner::CustomCostmap2DROS *custom_costmap_2d_ros);

  std::vector<int> getInfoGain(cv::Mat &prediction, cv::Mat &prediction_gt, bool use_gt = false);

  frontier_analysis::Pose2D getRobotPose();

protected:
  sensor_msgs::ImagePtr convert_to_ros_image(cv::Mat mat);
  cv::Mat convert_to_cv_image(sensor_msgs::Image &msg);

  ros::NodeHandle &nh_;
  ros::ServiceClient service_client_;

  boost::shared_ptr<hector_exploration_planner::HectorExplorationPlanner> planner_;
  boost::shared_ptr<hector_exploration_planner::CustomCostmap2DROS> costmap_2d_ros_;

}; // class InfoGainClient
} // namespace hector_exploration_planner
#endif //HECTOR_NAVIGATION_INFO_GAIN_CLIENT_H
