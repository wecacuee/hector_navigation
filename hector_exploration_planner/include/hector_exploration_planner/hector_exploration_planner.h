//=================================================================================================
// Copyright (c) 2012, Mark Sollweck, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef HECTOR_EXPLORATION_PLANNER_H___
#define HECTOR_EXPLORATION_PLANNER_H___

#include <ros/ros.h>
#include <hector_exploration_planner/custom_costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <dynamic_reconfigure/server.h>

#include <hector_exploration_planner/exploration_transform_vis.h>
//#include <hector_exploration_planner/frontier_vis.h>
#include <hector_exploration_planner/frontier_analysis.h>
#include <path_smoothing_ros/cubic_spline_interpolator.h>


#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/atomic.hpp>

#include <opencv2/core/core.hpp>

namespace hector_exploration_planner{

// forward declarations of test classes
class FrontiersTest;

class ExplorationPlannerConfig;
class InfoGainClient;

class HectorExplorationPlanner {
public:
  // allow for unit tests
//  friend class FrontiersTest;

  HectorExplorationPlanner();
  ~HectorExplorationPlanner();
  HectorExplorationPlanner(std::string name, hector_exploration_planner::CustomCostmap2DROS *costmap_ros);
  void initialize(std::string name,hector_exploration_planner::CustomCostmap2DROS *costmap_ros);

  void dynRecParamCallback(hector_exploration_planner::ExplorationPlannerConfig &config, uint32_t level);

  /**
    * Given a start point, finds a frontier between known and unknown space and generates a plan to go there
    * @param start The start point
    * @param plan The plan to explore into unknown space
    */
  bool doExploration(const geometry_msgs::PoseStamped &start,std::vector<geometry_msgs::PoseStamped> &plan);

  void updateFrontiers();

  float angleDifferenceWall(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal);

  /**
   * @return image with frontier points (with size and resolution of costmap)
   */
  cv::Mat getFrontierImg() { return frontiers_img_; }

  /**
   * @return clustered frontier points in world coords
   */
  std::vector<std::vector<geometry_msgs::PoseStamped>> getClusteredFrontierPoints() 
  { 
    boost::mutex::scoped_lock lock(frontiers_mutex_);
    return frontier_clusters_;
  }

  std::vector<geometry_msgs::PoseStamped> getFrontierClusterCenters()
  {
    boost::mutex::scoped_lock lock(frontiers_mutex_);
    return frontier_cluster_centers_;
  }

  costmap_2d::Costmap2D* getCostMap() { return costmap_;}

private:
  /**
   * Updates costmap data and resizes internal data structures if costmap size has changed. Should be called once before every planning command
   */
  void setupMapData();
  void deleteMapData();
  bool buildobstacle_trans_array_(bool use_inflated_obstacles);
  bool buildexploration_trans_array_(const geometry_msgs::PoseStamped &start,
                                     const std::vector<int>& info_gains,
                                     const std::vector<std::vector<int>>& goals,
                                     bool use_cell_danger = true);

  unsigned int max_valid_value(boost::shared_array<unsigned int> & array, int size);
  unsigned int min_value(boost::shared_array<unsigned int> & array, int size);

  /**
   * convert a exploration array into color map
   * @param exploration_array
   * @param max
   * @return
   */
  cv::Mat trans_array_to_image(boost::shared_array<unsigned int> & exploration_array);

  bool propagate_trans_cost(std::queue<int> init_queue, boost::shared_array<unsigned int>& array, bool use_cell_danger);

  int getTransDelta(int src_pt, int dst_pt);
  bool getTrajectory(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan);
  unsigned int cellDanger(int point);
  unsigned int angleDanger(float angle);

  void resetMaps();
  void clearFrontiers();
  bool isValid(int point);
  bool isFree(int point);
  bool isFrontier(int point);
  float angleDifference(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal);
  float getDistanceWeight(const geometry_msgs::PoseStamped &point1, const geometry_msgs::PoseStamped &point2);
  double getYawToUnknown(int point);
  bool isFrontierReached(int point);
  bool isSameFrontier(int frontier_point1,int frontier_point2);

  int getFrontierClusterCenter(const std::vector<int> &cluster);
  std::vector<int> getFrontierClusterCenters(const std::vector<std::vector<int>> &clusters);

  /**
   * @brief cluster frontiers, but just using index to represent a frontier
   * @param all_frontiers input frontiers
   * @param frontier_clusters output clusters
   * @return fail or success
   */
  bool clusterFrontiers(const std::vector<int>& all_frontiers,
                        std::vector<std::vector<int>>& frontier_clusters);

  /**
   * @brief cluster frontiers and remove small clusters
   * @param all_frontiers input frontiers
   * @param frontier_clusters output clusters
   * @return
   */
  bool clusterFrontiersRemoveSmall(const std::vector<int>& all_frontiers,
                                   std::vector<std::vector<int>>& frontier_clusters);

  void getStraightPoints(int point, int points[]);
  void getDiagonalPoints(int point, int points[]);
  void getAdjacentPoints(int point, int points[]);
  int left(int point);
  int upleft(int point);
  int up(int point);
  int upright(int point);
  int right(int point);
  int downright(int point);
  int down(int point);
  int downleft(int point);

  void getNeighbors(int point, int points[], int dis); // get all neighbors with distance < dis
  int getNeigoborsNumber(int dis);  // get how many neighbors it has given distance < dis as neighbors

  // construct a PoseStamped structure from int index
  bool constructFrontier(int point, geometry_msgs::PoseStamped& frontier);
  bool constructFrontiers(const std::vector<int> &points, std::vector<geometry_msgs::PoseStamped> &frontiers);
  bool constructFrontiers(const std::vector<std::vector<int>> &points,
                          std::vector<std::vector<geometry_msgs::PoseStamped>> &frontiers);

  bool findAllFrontiers(std::vector<int>& allFrontiers);
  bool findFrontiers(std::vector<int> &frontiers);

  int centerPoint(const std::vector<int>& frontier_clusters);
  int maxObstaclePoint(const std::vector<int>& frontier_cluster);

  void visualizeFrontiers(std::vector<geometry_msgs::PoseStamped> &frontiers);

  ros::Publisher observation_pose_pub_;
  ros::Publisher goal_pose_pub_;

  ros::Publisher visualization_pub_;
  ros::ServiceClient path_service_client_;
  hector_exploration_planner::CustomCostmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;

//  const unsigned char* occupancy_grid_array_;
  unsigned char* occupancy_grid_array_ = nullptr;
  boost::shared_array<unsigned int> exploration_trans_array_;
  boost::shared_array<unsigned int> obstacle_trans_array_;
  boost::shared_array<unsigned int> exploration_trans_array_info_gain_;
  boost::shared_array<int> frontier_map_array_;
  boost::shared_array<bool> is_goal_array_;

  std::vector<int> current_info_gain_;

  bool initialized_;
  int previous_goal_;

  std::string name;
  unsigned int map_width_;
  unsigned int map_height_;
  unsigned int num_map_cells_;

  // Parameters
  bool p_plan_in_unknown_;
  bool p_explore_close_to_path_;
  bool p_use_inflated_obs_;
  int p_goal_angle_penalty_;
  int p_min_obstacle_dist_;
  int p_min_frontier_size_;
  double p_alpha_;
  double p_dist_for_goal_reached_;
  double p_same_frontier_dist_;
  double p_obstacle_cutoff_dist_;
  bool p_use_observation_pose_calculation_;
  double p_observation_pose_desired_dist_;

  int p_min_dist_frontier_to_obstacle_;
  int p_frontier_neighbor_dist_;
  int p_min_frontier_cluster_size_;

  // use information gain or not
  bool use_information_gain_;

  // path smoothing params
  bool smoothing_enabled_;
  int smoothed_points_per_unit_;
  int smoothed_points_throttle_;
  bool smoothed_use_end_conditions_;
  bool smoothed_use_middle_conditions_;
  boost::shared_ptr<path_smoothing::CubicSplineInterpolator> path_smoother_;
  boost::mutex path_smoother_mutex_;

  double p_cos_of_allowed_observation_pose_angle_;
  double p_close_to_path_target_distance_;

  boost::shared_ptr<dynamic_reconfigure::Server<hector_exploration_planner::ExplorationPlannerConfig> > dyn_rec_server_;

  boost::shared_ptr<ExplorationTransformVis> vis_;
  boost::shared_ptr<ExplorationTransformVis> obstacle_vis_;
  boost::shared_ptr<ExplorationTransformVis> info_gain_vis_;

//  boost::shared_ptr<FrontierVis> frontier_vis_;

  std::vector<geometry_msgs::PoseStamped> frontiers_;
  std::vector<geometry_msgs::PoseStamped> frontier_cluster_centers_; ///< the center for each frontier cluster
  std::vector<std::vector<geometry_msgs::PoseStamped>> frontier_clusters_; ///< for all frontier points in clusters
  std::vector<std::vector<int>> frontier_index_clusters_;

  boost::atomic_bool is_frontiers_found_;
  boost::mutex frontiers_mutex_;
  cv::Mat frontiers_img_;

  boost::shared_ptr<boost::thread> frontiers_thread_;

  boost::shared_ptr<hector_exploration_planner::InfoGainClient> info_gain_client_;

};
}

#endif


