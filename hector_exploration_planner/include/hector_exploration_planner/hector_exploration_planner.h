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
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <dynamic_reconfigure/server.h>

#include <hector_exploration_planner/exploration_transform_vis.h>
#include <hector_exploration_planner/frontier_vis.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/atomic.hpp>

#include <opencv2/core/core.hpp>

namespace hector_exploration_planner{

class ExplorationPlannerConfig;

class HectorExplorationPlanner {
public:
  HectorExplorationPlanner();
  ~HectorExplorationPlanner();
  HectorExplorationPlanner(std::string name,costmap_2d::Costmap2DROS *costmap_ros);
  void initialize(std::string name,costmap_2d::Costmap2DROS *costmap_ros);

  void dynRecParamCallback(hector_exploration_planner::ExplorationPlannerConfig &config, uint32_t level);

  /**
   * Plans from start to given goal. If orientation quaternion of goal is all zeros, calls exploration instead. This is a hacky workaround that
   * has to be refactored.
   * @param start The start point
   * @param goal The goal point (Use orientation quaternion all 0 to let exploration find goal point)
   * @param plan The generated plan
   */
  bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &original_goal, std::vector<geometry_msgs::PoseStamped> &plan);

  /**
    * Given a start point, finds a frontier between known and unknown space and generates a plan to go there
    * @param start The start point
    * @param plan The plan to explore into unknown space
    */
  bool doExploration(const geometry_msgs::PoseStamped &start,std::vector<geometry_msgs::PoseStamped> &plan);

  /**
    * This can be used if there are no frontiers to unknown space left in the map. The robot will retrieve it's path travelled so far via a service
    * and try to go to places having a large distance to this path.
    * @param start The start point
    * @param plan The plan to explore into unknown space
    */
  bool doInnerExploration(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan);

  bool getObservationPose(const geometry_msgs::PoseStamped& observation_pose, const double desired_distance, geometry_msgs::PoseStamped& new_observation_pose);

  bool doAlternativeExploration(const geometry_msgs::PoseStamped &start,std::vector<geometry_msgs::PoseStamped> &plan, std::vector<geometry_msgs::PoseStamped> &oldplan);
  bool findFrontiers(std::vector<geometry_msgs::PoseStamped> &frontiers, std::vector<geometry_msgs::PoseStamped> &noFrontiers);
  bool findFrontiersCloseToPath(std::vector<geometry_msgs::PoseStamped> &frontiers);
  bool findFrontiers(std::vector<geometry_msgs::PoseStamped> &frontiers);
  bool findInnerFrontier(std::vector<geometry_msgs::PoseStamped> &innerFrontier);

  void updateFrontiers();
  
  float angleDifferenceWall(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal);
  bool exploreWalls(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &goals);

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
    return all_frontiers_clustered; 
  }

private:

  enum LastMode{
    FRONTIER_EXPLORE,
    INNER_EXPLORE
  } last_mode_;

  /**
   * Updates costmap data and resizes internal data structures if costmap size has changed. Should be called once before every planning command
   */
  void setupMapData();
  void deleteMapData();
  bool buildobstacle_trans_array_(bool use_inflated_obstacles);
  bool buildexploration_trans_array_(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> goals,bool useAnglePenalty, bool use_cell_danger = true);
  bool getTrajectory(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> goals, std::vector<geometry_msgs::PoseStamped> &plan);
  bool recoveryMakePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,std::vector<geometry_msgs::PoseStamped> &plan);
  unsigned int cellDanger(int point);
  unsigned int angleDanger(float angle);

  void saveMaps(std::string path);
  void resetMaps();
  void clearFrontiers();
  bool isValid(int point);
  bool isFree(int point);
  bool isFreeFrontiers(int point);
  bool isFrontier(int point);
  float angleDifference(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal);
  float getDistanceWeight(const geometry_msgs::PoseStamped &point1, const geometry_msgs::PoseStamped &point2);
  double getYawToUnknown(int point);
  bool isFrontierReached(int point);
  bool isSameFrontier(int frontier_point1,int frontier_point2);

  /**
   * @brief clusters frontiers into blobs
   * @param allFrontiers vector of all frontier cells (input argument)
   * @param frontiers same as the frontiers param of findFrontiers method (output argument)
   * @return whether the frontiers is empty
   */
  bool clusterFrontiers(std::vector<int> &allFrontiers,
                        std::vector<geometry_msgs::PoseStamped> &frontiers);


  /**
   * @brief clusters frontiers into blobs
   * @param allFrontiers vector of all frontier cells (input argument)
   * @param frontiers same as the frontiers param of findFrontiers method (output argument)
   * @param noFrontiers same as the frontiers param of findFrontiers method (output argument)
   * @param frontierPoints frontiers in map (image) coordinates
   * @return whether the frontiers is empty
   */
//  bool clusterFrontiers(std::vector<int> &allFrontiers,
//                        std::vector<geometry_msgs::PoseStamped> &frontiers,
//                        std::vector<geometry_msgs::PoseStamped> &noFrontiers);
//

  /**
   * @brief cluster frontiers, but just using index to represent a frontier
   * @param all_frontiers
   * @param frontier_clusters
   * @return
   */
  bool clusterFrontiers(std::vector<int>& all_frontiers, std::vector<std::vector<int>>& frontier_clusters);

  bool clusterFrontiers(std::vector<int>& all_frontiers,
                        std::vector<geometry_msgs::PoseStamped> &frontiers,
                        std::vector<std::vector<geometry_msgs::PoseStamped>>& frontier_clusters);

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
  bool findAllFrontiers_index(std::vector<int>& allFrontiers);
  bool findFrontiers_index(std::vector<int>& frontiers);
//  bool clusterFrontiers_index(std::vector<int>& allFrontiers, std::vector<int>& frontiers);
  bool centerOfFrontierCluster(std::vector<int>& frontier_clusters,
                               geometry_msgs::PoseStamped& frontiers);

  bool max_obs_point_of_cluster(std::vector<int>& frontier_cluster,
                                                               geometry_msgs::PoseStamped& frontiers);

  void visualizeFrontiers(std::vector<geometry_msgs::PoseStamped> &frontiers);


  ros::Publisher observation_pose_pub_;
  ros::Publisher goal_pose_pub_;

  ros::Publisher visualization_pub_;
  ros::ServiceClient path_service_client_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;

  const unsigned char* occupancy_grid_array_;
  boost::shared_array<unsigned int> exploration_trans_array_;
  boost::shared_array<unsigned int> obstacle_trans_array_;
  boost::shared_array<int> frontier_map_array_;
  boost::shared_array<bool> is_goal_array_;

  cv::Mat exploration_trans_img_;
  static void drawExplorationTransform(const boost::shared_array<unsigned int> exploration_transform_array,
                                       const costmap_2d::Costmap2D& costmap,
                                       cv::Mat &img);

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
  // frontier parameters
  int neighbor_distance = 5;
  int cluster_min_number = 25;

  double p_cos_of_allowed_observation_pose_angle_;
  double p_close_to_path_target_distance_;

  boost::shared_ptr<dynamic_reconfigure::Server<hector_exploration_planner::ExplorationPlannerConfig> > dyn_rec_server_;

  boost::shared_ptr<ExplorationTransformVis> vis_;
  boost::shared_ptr<ExplorationTransformVis> close_path_vis_;
  boost::shared_ptr<ExplorationTransformVis> inner_vis_;
  boost::shared_ptr<ExplorationTransformVis> obstacle_vis_;

  boost::shared_ptr<FrontierVis> frontier_vis_;

  std::vector<geometry_msgs::PoseStamped> frontiers_;

  std::vector<geometry_msgs::PoseStamped> clustered_frontiers_; ///< the center for each frontier cluster
  std::vector<std::vector<geometry_msgs::PoseStamped>> all_frontiers_clustered; ///< for all frontier points in clusters

  boost::atomic_bool is_frontiers_found_;
  boost::mutex frontiers_mutex_;
  cv::Mat frontiers_img_;

  boost::shared_ptr<boost::thread> frontiers_thread_;

};
}

#endif


