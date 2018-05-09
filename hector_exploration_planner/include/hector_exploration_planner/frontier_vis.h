//
// Created by rakesh on 09/05/18.
//

#ifndef HECTOR_NAVIGATION_FRONTIER_VIS_H
#define HECTOR_NAVIGATION_FRONTIER_VIS_H

#include <geometry_msgs/PoseStamped.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

namespace hector_exploration_planner
{
class FrontierVis
{
public:
  FrontierVis();
  void publishVisOnDemand(const std::vector<geometry_msgs::PoseStamped> frontiers);
  void run();

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_visualizer_;
  boost::shared_ptr<boost::thread> thread_handler_;
  boost::mutex mutex_;
  boost::atomic_bool update_cloud_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;
}; // class FrontierVis
} // namespace hector_exploration_planner

#endif //HECTOR_NAVIGATION_FRONTIER_VIS_H
