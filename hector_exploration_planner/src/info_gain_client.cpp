//
// Created by rakesh on 30/07/18.
//

#include <hector_exploration_planner/info_gain_client.h>
#include <online_map_completion_msgs/InfoGains.h>

#include <tf/transform_datatypes.h>

#include <cv_bridge/cv_bridge.h>

using namespace hector_exploration_planner::frontier_analysis;
namespace hector_exploration_planner
{

InfoGainClient::InfoGainClient(ros::NodeHandle &nh,
                               hector_exploration_planner::HectorExplorationPlanner *planner,
                               hector_exploration_planner::CustomCostmap2DROS *custom_costmap_2d_ros)
  : nh_(nh)
{
  service_client_ = nh_.serviceClient<online_map_completion_msgs::InfoGains>("info_gain");

  planner_ = boost::shared_ptr<hector_exploration_planner::HectorExplorationPlanner>(
    planner,
    [](hector_exploration_planner::HectorExplorationPlanner *planner) {
      // can't do anything here, the creator of planner pointer destroys it :(
    }
  );

  costmap_2d_ros_ = boost::shared_ptr<hector_exploration_planner::CustomCostmap2DROS>(
    custom_costmap_2d_ros,
    [](hector_exploration_planner::CustomCostmap2DROS* custom_costmap_2d_ros) {
      // can't do anything here, the creator of planner pointer destroys it :(
    }
  );
}

std::vector<int> InfoGainClient::getInfoGain()
{
  cv::Mat costmap;
  std::vector<cv::Rect> bounding_boxes;
  std::vector<std::vector<frontier_analysis::Pose2D>> frontier_clusters;

  auto robot_pose = getRobotPose();

  frontier_analysis::get_frontier_info(
    planner_,
    costmap_2d_ros_,
    robot_pose,
    MAP_RESOLUTION,
    MAP_SIZE,
    costmap,
    bounding_boxes,
    frontier_clusters
  );

  auto ros_costmap_image = convert_to_ros_image(costmap);

  online_map_completion_msgs::InfoGains srv;
  srv.request.multi_channel_occupancy_grid = *ros_costmap_image;
  // TODO: fill

  for (const auto &cluster: frontier_clusters)
  {
    geometry_msgs::PoseArray pose_array;
    pose_array.header.stamp = ros::Time::now();
    for (const auto &point: cluster)
    {
      geometry_msgs::Pose pose;
      pose.position.x = point.position.x;
      pose.position.y = point.position.y;
      pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, point.orientation);
      pose_array.poses.push_back(pose);
    }
    srv.request.frontier_clusters.push_back(pose_array);
  }

  for (const auto &bounding_box: bounding_boxes)
  {
    sensor_msgs::RegionOfInterest roi;
    roi.width = bounding_box.width;
    roi.height = bounding_box.height;
    roi.x_offset = bounding_box.x;
    roi.y_offset = bounding_box.y;
    srv.request.frontier_rois.push_back(roi);
  }

  std::vector<int> infomation_gains;

  if (service_client_.call(srv))
  {
    for (const auto &i: srv.response.info_gains)
    {
      infomation_gains.push_back(i.data);
    }
  }
  else
  {
    ROS_ERROR("Infomation gain service call failed");
  }
}

frontier_analysis::Pose2D InfoGainClient::getRobotPose()
{
  geometry_msgs::PoseStamped pose;
  tf::Stamped<tf::Pose> robot_pose_tf;
  bool robot_pose_status = costmap_2d_ros_->getRobotPose(robot_pose_tf);

  if (!robot_pose_status || robot_pose_tf.getRotation().length2() < 1e-5)
  {
    ROS_ERROR("Failed to get robot pose from costmap");
  }

  tf::poseStampedTFToMsg(robot_pose_tf, pose);

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(pose.pose.orientation, orientation);
  if (orientation.length2() < 1e-5)
  {
    ROS_ERROR("poseStampedTFToMsg incorrect");
  }

  auto costmap = costmap_2d_ros_->getCostmap();
  return frontier_analysis::worldPose2MapPose(
    pose,
    costmap->getResolution(),
    costmap->getSizeInCellsX(),
    costmap->getSizeInCellsY()
  );
}

sensor_msgs::ImagePtr InfoGainClient::convert_to_ros_image(cv::Mat mat)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr->image = mat;

  return cv_ptr->toImageMsg();
}

} // namespace hector_exploration_planner