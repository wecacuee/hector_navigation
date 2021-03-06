//
// Created by rakesh on 30/07/18.
//

#ifndef HECTOR_NAVIGATION_FRONTIER_ANALYSIS_H
#define HECTOR_NAVIGATION_FRONTIER_ANALYSIS_H

// preprocessing operations on frontiers
#define FRONTIER_HOLE_FILLING_KERNEL 1
#define FRONTIER_OPENING_KERNEL 1

#define MIN_FRONTIER_CLUSTER_SIZE 20

#include <hector_exploration_planner/custom_costmap_2d_ros.h>
#include <hector_exploration_planner/hector_exploration_planner.h>
#include <costmap_2d/static_layer.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/core/core.hpp>

#include <vector>

namespace hector_exploration_planner
{
class HectorExplorationPlanner;
namespace frontier_analysis
{

struct Pose2D
{
  cv::Point position;
  double orientation;

  Pose2D(int x, int y, double yaw)
  {
    position.x = x;
    position.y = y;
    orientation = yaw;
  }

  Pose2D(const cv::Point& position, double yaw)
  {
    this->position = position;
    this->orientation = yaw;
  }

  Pose2D()
  {
    position.x = 0;
    position.y = 0;
    orientation = 0;
  }
};

/**
 * Overload the function getRawMap, build a cvMat using Costmap2D
 * @param costmap_2d
 * @return
 */
cv::Mat getRawMap(const Costmap2D &costmap_2d);

/**
 * @brief get cv::Mat Raw image from costmap_2d_ros
 * @param costmap_2d_ros
 * @return Raw image with just one channels, 0: free, 254: obstacle, 255: unknown
 */
cv::Mat getRawMap(const boost::shared_ptr<hector_exploration_planner::CustomCostmap2DROS> &costmap_2d_ros);

/**
 * @brief Split raw cv::Mat map into 3 channels map, represent free, obstacle and unkown in each channel
 * @param rawMap One channel raw Map with values between 0-255, 0: free, 254: obstacle, 255: unknown
 * @return Image with three channels
 */
cv::Mat splitRawMap(const cv::Mat &rawMap);

/**
 * overload getMap, to get the image of costmap
 * @param costmap_2d
 * @return
 */
cv::Mat getMap(const Costmap2D &costmap_2d);

/**
 * @brief get cv::Mat image from raw costmap_2d_ros
 * @param costmap_2d_ros
 * @return Image with three channels, corresponding to unknown, free and obstacle
 */
cv::Mat getMap(const boost::shared_ptr<hector_exploration_planner::CustomCostmap2DROS> &costmap_2d_ros);

/**
 * @brief getFrontierPoints from costmap, the resolution is the same with original map
 */
void getFrontierPoints(boost::shared_ptr<hector_exploration_planner::HectorExplorationPlanner> planner,
                       std::vector<std::vector<Pose2D>> &all_clusters_cv);

/**
 * @brief get frontier cluter centers from costmap, the resolution is the same with original map
 */
void getFronierCenters(boost::shared_ptr<hector_exploration_planner::HectorExplorationPlanner> planner,
                       std::vector<Pose2D> &frontier_clusters_centers);

/**
 * @brief Resize a single Point by a ratio
 * @param inputPoint the point to be resized
 * @param ratio
 * @return resized Point
 */
Pose2D resizePoint(const Pose2D &inputPoint, double ratio);

/**
 * @brief resize Frontier Points by a ratio, overload function
 * @param inputPoints
 * @param outputPoints
 * @param ratio
 */
void resizePoints(const std::vector<Pose2D> &inputPoints,
                    std::vector<Pose2D> &outputPoints, double ratio);

/**
 * @brief resize Frontier Points by a ratio, overload function
 * @param inputPoints
 * @param outputPoints
 * @param ratio
 */
void resizePoints(const std::vector<std::vector<Pose2D>> &inputPoints,
                  std::vector<std::vector<Pose2D>> &outputPoints, double ratio);

Pose2D convertToGroundTruthSize(const Pose2D &point, const cv::Size &orgSize, const cv::Size &groundtruth_size);

/**
 * @brief adapt Frontier Points in original map to the padded/clipped map whose size is equal to the ground truth map's
 * @param inputPoints
 * @param outputPoints
 * @param orgSize
 * @param groundtruth_size
 */
void convertToGroundTruthSize(const std::vector<Pose2D> &inputPoints,
                              std::vector<Pose2D> &outputPoints,
                              const cv::Size& orgSize,const cv::Size& groundtruth_size);

/**
 * @brief adapt Frontier Points in original map to the padded/clipped map whose size is equal to the ground truth map's
 * @param inputPoints
 * @param outputPoints
 * @param orgSize
 * @param groundtruth_size
 */
void convertToGroundTruthSize(const std::vector<std::vector<Pose2D>> &inputPoints,
                              std::vector<std::vector<Pose2D>> &outputPoints,
                              const cv::Size &orgSize,
                              const cv::Size &groundtruth_size);

/**
 * @brief generate Bounding Box for each frontier cluster
 * @param inputPoints all frontier clusters
 * @return Bounding Boxes of every cluster, each one represented by a rectangle
 */
std::vector<cv::Rect> generateBoundingBox(std::vector<std::vector<Pose2D>> &inputPoints);

/**
 * @brief generate a Bounding Box Image from all Bounding Box Rectangles, every Rectangle should in the range of image size
 * @param inputRects  all Bounding Box Rectangles
 * @param size the size of output Image
 * @return Output Bounding Box Image
 */
cv::Mat generateBoundingBoxImage(std::vector<cv::Rect> &inputRects, cv::Size size);


/**
 * @brief generate a Verify Image from Frontier Points, Bounding Box Rectangles and free-space
 * @param costmap
 * @param boundingBoxes Bounding Boxes for each Frontiers
 * @param cluster_frontiers Frontiers Points
 * @param plan trajectory plan
 * @param another_plan another trajectory plan
 * @param robotPose robot pose after execute plan
 * @param robot_poses robot poses in executing plan
 * @return Verify Image
 */
cv::Mat generateVerifyImage(const cv::Mat &costmap, const std::vector<cv::Rect> &boundingBoxes,
                            const std::vector<std::vector<Pose2D>> &cluster_frontiers,
                            const std::vector<Pose2D> &plan,
                            const std::vector<Pose2D> &another_plan,
                            const Pose2D &robotPose,
                            const std::vector<Pose2D> &robot_poses);

/**
 *
 * @param costmap_2d_ros
 * @param desired_resolution
 * @return
 */
cv::Mat getMap(const boost::shared_ptr<hector_exploration_planner::CustomCostmap2DROS> &costmap_2d_ros,
               double desired_resolution);

/**
 *
 * @param costmap_2d_ros
 * @param clustered_frontier_poses
 * @param transform_gt_est
 * @return
 */
cv::Mat getBoundingBoxImage(const boost::shared_ptr<hector_exploration_planner::CustomCostmap2DROS> &costmap_2d_ros,
                            const std::vector<std::vector<geometry_msgs::PoseStamped> > clustered_frontier_poses);

/**
 * @brief clip and padding the original map to fit groundtruth
 * @param original_map
 * @param groundtruth_size
 * @return
 */
cv::Mat convertToGTSizeFillUnknown(const cv::Mat &original_map, const cv::Size groundtruth_size);

/**
 * @brief clip and padding the original map to fit groundtruth
 * @param original_map
 * @param groundtruth_size
 * @param padValue the value for padding area
 * @return
 */
cv::Mat convertToGTSize(const cv::Mat &original_map,
                        const cv::Size groundtruth_size, unsigned char padValue = 0);

/**
 *
 * @param original_map the original map with probabilities between 0 to 100 and -1 for invalid
 * @return map with only free space colored with max intensity
 */
cv::Mat thresholdCostmap(const cv::Mat &original_map);

/**
 * Split costmap into free, obstacle and unknown area
 * @param original the original map, 0: free, 254: obstacle, 255:unknown
 * @param free the free space
 * @param obstacle the obstacle map
 * @param unknown the unknown area
 */
void thresholdCostmap(const cv::Mat &original, cv::Mat &unknown, cv::Mat &free, cv::Mat &obstacle);

/**
 *
 * @param costmap_2d_ros
 * @param odometry
 * @return transform from estimated coordinates to groundtruth coordinates
 */
tf::Transform getTransformGroundtruthEstimated(
  const boost::shared_ptr<hector_exploration_planner::CustomCostmap2DROS> &costmap_2d_ros,
  const nav_msgs::Odometry &odometry);

/**
 * @brief affine transformation to center the image to (0, 0) in world (metric) frame
 * @param static_costmap static costmap
 * @return affine transformation
 */
cv::Mat getMapCenteringAffineTransformation(const boost::shared_ptr<costmap_2d::Costmap2D> static_costmap);


/**
 * @brief affine transformation to align estimated map with groundtruth
 * @param static_costmap static costmap
 * @param transform_gt_est transform from estimated map coordinates to groundtruth (in meters)
 * @return affine transformation
 */
cv::Mat getMapGroundtruthAffineTransformation(const boost::shared_ptr<costmap_2d::Costmap2D> static_costmap,
                                              const tf::Transform &transform_gt_est);


/**
 * @brief Based on costmap_2d_ros getRobotPose method
 * @param source_pose
 * @param target_frame_id
 * @param target_pose
 * @return if the pose query was successful
 */
bool transformPose(tf::Stamped<tf::Pose> source_pose,
                   std::string target_frame_id,
                   tf::Stamped<tf::Pose> &target_pose);

/**
 *
 * @param costmap_image image in the resolution/size of costmap (e.g. frontier image)
 * @param costmap_2d_ros
 * @param desired_resolution
 * @return
 */
cv::Mat resizeToDesiredResolution(const cv::Mat &costmap_image,
                                  const boost::shared_ptr<hector_exploration_planner::CustomCostmap2DROS> costmap_2d_ros,
                                  double desired_resolution);

/**
 *
 * @param costmap_bounding_rect bounding rectangle in the resolution/size of costmap (e.g. frontier ROI)
 * @param costmap_2d_ros
 * @param desired_resolution
 * @return
 */
cv::Rect resizeToDesiredResolution(const cv::Rect &costmap_bounding_rect,
                                   const boost::shared_ptr<hector_exploration_planner::CustomCostmap2DROS> &costmap_2d_ros,
                                   double desired_resolution);

/**
 * @brief Convert a point in world coordinates (meters) to the point
 * in map coordinate (pixel), the orientation is not changed
 * @param world_pose point (with orientation) in world coordinates
 * @param costmap costmap
 * @return converted Point
 */
Pose2D worldPose2MapPose(const geometry_msgs::PoseStamped &world_pose,
                         const costmap_2d::Costmap2D &costmap);

/**
 * @brief Convert a cluster of points in world coordinates (meters)
 * to the points in map coordinate (pixel), the orientation is not changed
 * @param world_points
 * @param costmap
 * @return converted Point
 */
std::vector<Pose2D> worldPosesToMapPoses(const std::vector<geometry_msgs::PoseStamped> &world_points,
                                         const costmap_2d::Costmap2D &costmap);

/**
 *
 * @param world_points points in world (metric) coords
 * @param costmap_2d_ros
 * @param transform_gt_est transform from estimated coords to groundtruth coords
 * @return points in (cost)map coordinates
 */
std::vector<cv::Point> worldPointsToMapPoints(const std::vector<geometry_msgs::PoseStamped> &world_points,
                                              const boost::shared_ptr<hector_exploration_planner::CustomCostmap2DROS> &costmap_2d_ros);

/**
 * @brief preprocessing operations (closing (hole filling) and opening)
 * @param frontier_img_in input image
 * @param frontier_img_out output image
 * @return
 */
void preprocessFrontierImg(cv::Mat &frontier_img_in, cv::Mat &frontier_img_out);

/**
 *
 * @brief group frontier points based on the clusters they belong to
 * @param frontier_img input binary image of frontier points
 * @param clustered_frontier_points clusters of frontiers
 * @return grouped frontiers
 */
std::vector<std::vector<cv::Point> > groupFrontiers(cv::Mat &frontier_img,
                                                    std::vector<cv::Point> clustered_frontier_points);

/**
 *
 * @brief color the frontier image based on clusters
 * @param frontier_img input frontier binary image (single channeled)
 * @param grouped_frontiers coordinates of the frontiers grouped into clusters
 * @param rng random number generator
 * @param colored_frontier_img output image (three channeled)
 * @return vector of colors for each cluster
 */
std::vector<cv::Scalar> colorFrontiers(cv::Mat &frontier_img,
                                       std::vector<std::vector<cv::Point> > grouped_frontiers,
                                       cv::RNG &rng,
                                       cv::Mat &colored_frontier_img);

/**
 * @brief load stage world into occupancy map
 * @param world file of stage
 * @param costmap ros costmap_2d of the map being built
 * @param output_image occupancy map
 */
void loadStageWorld(std::string world_file,
                    const costmap_2d::Costmap2D &costmap,
                    cv::Mat &output_img);

/**
 *
 * @param occupancy_unknown_img image containing occupied pixels and unknown unoccupied images
 * @param grouped_frontiers frontier points grouped into clusters
 * @return for each frontier group, get unknown points closest to it
 */
std::vector<cv::Point> getClosestUnknowns(cv::Mat &occupancy_unknown_img,
                                          std::vector<std::vector<cv::Point> > grouped_frontiers);


/**
 *
 * @param occupancy_unknown_img image containing occupied pixels and unknown unoccupied images
 * @param unknown_points unknown point from each frontier cluster
 * @return floodfills each frontier cluster
 */
std::vector<std::vector<cv::Point> > expandUnknowns(cv::Mat &occupancy_unknown_img,
                                                    std::vector<cv::Point> unknown_points);

void get_frontier_info(const boost::shared_ptr<hector_exploration_planner::HectorExplorationPlanner> planner,
                       const boost::shared_ptr<hector_exploration_planner::CustomCostmap2DROS> costmap_2d_ros,
                       Pose2D robotPose,
                       double map_resolution,
                       cv::Size desired_size,
                       cv::Mat &costmap_output,
                       std::vector<cv::Rect> &bounding_box_output,
                       std::vector<std::vector<Pose2D>> &frontier_clusters);

std::pair<int, int> pointToPair(cv::Point point);

} // namespace frontier_analysis
} // namespace hector_exploration_planner


#endif //HECTOR_NAVIGATION_FRONTIER_ANALYSIS_H

