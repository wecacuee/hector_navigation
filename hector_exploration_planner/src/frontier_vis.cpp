//
// Created by rakesh on 09/05/18.
//
#define NORMAL_LENGTH 50

#include <hector_exploration_planner/frontier_vis.h>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cmath>


namespace hector_exploration_planner
{

FrontierVis::FrontierVis(const std::string &topic_name):
  nh_("~"),
  image_transport_(nh_),
  image_pub_(image_transport_.advertise(topic_name, 1))
{

}

void FrontierVis::drawPose(cv::Mat img, cv::Point point, double yaw,
                           cv::Scalar point_color, cv::Scalar normal_color)
{
  drawPoint(img, point, point_color);
  cv::Point normal_end(
    point.x + std::cos(yaw) * NORMAL_LENGTH,
    point.y + std::sin(yaw) * NORMAL_LENGTH
  );

  cv::line(img, point, normal_end, normal_color);
}

void FrontierVis::drawPoint(cv::Mat img, cv::Point point, cv::Scalar color)
{
  cv::circle(img, point, 3, color, -1);
}

void FrontierVis::publishVisOnDemand(const std::vector<geometry_msgs::PoseStamped> &frontiers,
                                     const costmap_2d::Costmap2D& costmap,
                                     const costmap_2d::Costmap2DROS& costmap_ros)
{
  boost::lock_guard<boost::mutex> guard(mutex_);

  cv::Mat map(costmap.getSizeInCellsY(), costmap.getSizeInCellsY(), CV_8UC3, cv::Scalar(0, 0, 0));

  tf::Stamped<tf::Pose> robot_pose;
  costmap_ros.getRobotPose(robot_pose);
  unsigned int robot_map_x, robot_map_y;
  auto robot_position = robot_pose.getOrigin();
  costmap.worldToMap(robot_position.x(), robot_position.y(), robot_map_x, robot_map_y);
  drawPose(map, cv::Point(robot_map_x, robot_map_y), tf::getYaw(robot_pose.getRotation()), cv::Scalar(255, 255, 255), cv::Scalar(0, 255, 0));


  for (const auto &frontier: frontiers) {
    unsigned int map_x;
    unsigned int map_y;
    costmap.worldToMap(frontier.pose.position.x, frontier.pose.position.y, map_x, map_y);
    cv::Point frontier_point(map_x, map_y);
    drawPoint(map, frontier_point, cv::Scalar(0, 0, 255));
  }

  cv::Mat map_flipped;
  cv::flip(map, map_flipped, 0);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", map_flipped).toImageMsg();
  image_pub_.publish(msg);
}
} // namespace hector_exploration_planner