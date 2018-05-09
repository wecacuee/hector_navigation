//
// Created by rakesh on 09/05/18.
//

#ifndef HECTOR_NAVIGATION_FRONTIER_VIS_H
#define HECTOR_NAVIGATION_FRONTIER_VIS_H

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

namespace hector_exploration_planner
{
class FrontierVis
{
public:
  FrontierVis(const std::string &topic_name)
  {
    ros::NodeHandle nh("~");
    frontier_pub_ = nh.advertise<>(topic_name, 1, false);
  }

  void publishVisOnDemand(const std::vector<geometry_msgs::PoseStamped>)
  {
    if (frontier_pub_.getNumSubscribers() > 0) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time();
      marker.ns = "hector_exploration_planner";
      marker.id = id++;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::DELETE;
      frontier_pub_.publish(marker);

      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = wx;
      marker.pose.position.y = wy;
      marker.pose.position.z = 0.0;
      marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.color.a = 1.0;

      if(frontier_is_valid){
        marker.color.r = 0.0;
        marker.color.g = 1.0;
      }else{
        marker.color.r = 1.0;
        marker.color.g = 0.0;
      }

      marker.color.b = 0.0;
      marker.lifetime = ros::Duration(0); // (50,0);
      frontier_pub_.publish(marker);
    }
  }
protected:
  ros::Publisher frontier_pub_;
}; // class FrontierVis
} // namespace hector_exploration_planner

#endif //HECTOR_NAVIGATION_FRONTIER_VIS_H
