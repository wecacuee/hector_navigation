//
// Created by rakesh on 09/05/18.
//

#include <hector_exploration_planner/frontier_vis.h>
#include <tf/tf.h>
#include <boost/thread.hpp>
#include <cmath>

namespace hector_exploration_planner
{

FrontierVis::FrontierVis():
  pcl_visualizer_(new pcl::visualization::PCLVisualizer("frontiers")),
  thread_handler_(new boost::thread(&FrontierVis::run, this)),
  cloud_(new pcl::PointCloud<pcl::PointXYZ>),
  normals_(new pcl::PointCloud<pcl::Normal>),
  update_cloud_(false)
{
  // run the main thread
  thread_handler_->detach();
}

void FrontierVis::run()
{
  while (!pcl_visualizer_->wasStopped ()) {
    pcl_visualizer_->spinOnce(100);

    if (update_cloud_) {
      boost::lock_guard<boost::mutex> guard(mutex_);

      if (!pcl_visualizer_->updatePointCloud(cloud_))
      {
        pcl_visualizer_->addPointCloud<pcl::PointXYZ>(cloud_, "points");
        pcl_visualizer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "points");
      }
      pcl_visualizer_->removePointCloud("normals", 0);
      pcl_visualizer_->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud_, normals_, 150, 0.35, "normals");
      update_cloud_ = false;
    }
  }
}

void FrontierVis::publishVisOnDemand(const std::vector<geometry_msgs::PoseStamped> &frontiers)
{
  boost::lock_guard<boost::mutex> guard(mutex_);

  cloud_->clear();
  normals_->clear();
  for (const auto &frontier: frontiers) {
    cloud_->push_back(
      pcl::PointXYZ(frontier.pose.position.x, frontier.pose.position.y, frontier.pose.position.z)
    );

    auto yaw = tf::getYaw(frontier.pose.orientation);
    normals_->push_back(
      pcl::Normal(std::cos(yaw), std::sin(yaw), 0)
    );
  }
  update_cloud_ = true;
}
} // namespace hector_exploration_planner