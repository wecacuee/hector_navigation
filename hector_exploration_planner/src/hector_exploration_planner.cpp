//=================================================================================================
// Copyright (c) 2012, Mark Sollweck, Stefan Kohlbrecher, Florian Berz TU Darmstadt
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

#include <hector_exploration_planner/hector_exploration_planner.h>
#include <hector_exploration_planner/info_gain_client.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <hector_nav_msgs/GetRobotTrajectory.h>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <hector_exploration_planner/ExplorationPlannerConfig.h>

#define STRAIGHT_COST 100
#define DIAGONAL_COST 141

//#define STRAIGHT_COST 3
//#define DIAGONAL_COST 4

using namespace hector_exploration_planner;

/* definition to expand macro then apply to pragma message */
#define VALUE_TO_STRING(x) #x
#define VALUE(x) VALUE_TO_STRING(x)
#define VAR_NAME_VALUE(var) #var "="  VALUE(var)

#pragma message(VAR_NAME_VALUE(USE_CUSTOM_POSE))


HectorExplorationPlanner::HectorExplorationPlanner()
: costmap_ros_(0)
, costmap_(0)
, initialized_(false)
, map_width_(0)
, map_height_(0)
, num_map_cells_(0)
{
#if USE_CUSTOM_POSE
  ROS_INFO("[hector_exploration_planner] Using custom pose");
#else
  ROS_INFO("[hector_exploration_planner] Using tf pose");
#endif
}

HectorExplorationPlanner::~HectorExplorationPlanner(){
  this->deleteMapData();
}

HectorExplorationPlanner::HectorExplorationPlanner(std::string name, hector_exploration_planner::CustomCostmap2DROS *costmap_ros_in) :
costmap_ros_(NULL), initialized_(false), is_frontiers_found_(false) {
  HectorExplorationPlanner::initialize(name, costmap_ros_in);
}

void HectorExplorationPlanner::initialize(std::string name, hector_exploration_planner::CustomCostmap2DROS *costmap_ros_in){
  // unknown: 255, obstacle 254, inflated: 253, free: 0

  if(initialized_){
    ROS_ERROR("[hector_exploration_planner] HectorExplorationPlanner is already initialized_! Please check why initialize() got called twice.");
    return;
  }

  ROS_INFO("[hector_exploration_planner] Initializing HectorExplorationPlanner");

  // initialize costmaps
  this->costmap_ros_ = costmap_ros_in;
  this->setupMapData();

  // initialize parameters
  ros::NodeHandle private_nh_("~/" + name);
  ros::NodeHandle nh;
  visualization_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);

  observation_pose_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("observation_pose", 1, true);
  goal_pose_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("goal_pose", 1, true);

  dyn_rec_server_.reset(new dynamic_reconfigure::Server<hector_exploration_planner::ExplorationPlannerConfig>(ros::NodeHandle("~/hector_exploration_planner")));

  dyn_rec_server_->setCallback(boost::bind(&HectorExplorationPlanner::dynRecParamCallback, this, _1, _2));

  path_service_client_ = nh.serviceClient<hector_nav_msgs::GetRobotTrajectory>("trajectory");

  ROS_INFO("[hector_exploration_planner] Parameter set. security_const: %f, min_obstacle_dist: %d, plan_in_unknown: %d, use_inflated_obstacle: %d, p_goal_angle_penalty_:%d , min_frontier_size: %d, p_dist_for_goal_reached_: %f, same_frontier: %f", p_alpha_, p_min_obstacle_dist_, p_plan_in_unknown_, p_use_inflated_obs_, p_goal_angle_penalty_, p_min_frontier_size_,p_dist_for_goal_reached_,p_same_frontier_dist_);
  //p_min_obstacle_dist_ = p_min_obstacle_dist_ * STRAIGHT_COST;

  this->name = name;
  this->initialized_ = true;
  this->previous_goal_ = -1;

  vis_.reset(new ExplorationTransformVis("exploration_transform"));
  obstacle_vis_.reset(new ExplorationTransformVis("obstacle_transform"));
//  frontier_vis_.reset(new FrontierVis("frontier_img"));

  info_gain_client_ = boost::make_shared<InfoGainClient>(nh, this, costmap_ros_in);

  // frontiers_thread_.reset(
  //   new boost::thread([this]() {
  //     while (ros::ok()) {
  //       updateFrontiers();
  //       ros::Duration(0.5).sleep();
  //     }
  //   })
  // );
  // frontiers_thread_->detach();
}

void HectorExplorationPlanner::dynRecParamCallback(hector_exploration_planner::ExplorationPlannerConfig &config, uint32_t level)
{
  p_plan_in_unknown_ = config.plan_in_unknown;
  p_explore_close_to_path_ = config.explore_close_to_path;
  p_use_inflated_obs_ = config.use_inflated_obstacles;
  p_goal_angle_penalty_ = config.goal_angle_penalty;
  p_alpha_ = config.security_constant;
  p_dist_for_goal_reached_ = config.dist_for_goal_reached;
  p_same_frontier_dist_ = config.same_frontier_distance;
  p_min_frontier_size_ = config.min_frontier_size;
  p_min_obstacle_dist_ = config.min_obstacle_dist * STRAIGHT_COST;
  p_obstacle_cutoff_dist_ = config.obstacle_cutoff_distance;

  p_use_observation_pose_calculation_ = config.use_observation_pose_calculation;
  p_observation_pose_desired_dist_ = config.observation_pose_desired_dist;
  double angle_rad = config.observation_pose_allowed_angle * (M_PI / 180.0);
  p_cos_of_allowed_observation_pose_angle_ = cos(angle_rad);
  p_close_to_path_target_distance_ = config.close_to_path_target_distance;

  {
    boost::mutex::scoped_lock lock(path_smoother_mutex_);

    smoothing_enabled_ = config.smoothing_enabled;
    smoothed_points_per_unit_ = config.smoothed_points_per_unit;
    smoothed_points_throttle_ = config.smoothed_points_throttle;
    smoothed_use_end_conditions_ = config.smoothed_use_end_conditions;
    smoothed_use_middle_conditions_ = config.smoothed_use_middle_conditions;

    path_smoother_.reset(new path_smoothing::CubicSplineInterpolator(
      smoothed_points_per_unit_, smoothed_points_throttle_, smoothed_use_end_conditions_,
      smoothed_use_middle_conditions_
    ));
  }
}

void HectorExplorationPlanner::updateFrontiers()
{
  boost::mutex::scoped_lock lock(frontiers_mutex_);

  is_frontiers_found_ = false;

  std::vector<int> frontiers;

  this->setupMapData();

  // setup maps and goals
  resetMaps();
  clearFrontiers();

  // create obstacle transform
  buildobstacle_trans_array_(p_use_inflated_obs_);

  is_frontiers_found_ = findFrontiers(frontiers);

  std::vector<std::vector<int>> frontier_clusters;
  clusterFrontiersRemoveSmall(frontiers, frontier_clusters);
  std::vector<int> frontier_cluster_centers = getFrontierClusterCenters(frontier_clusters);

  frontiers_.clear();
  clustered_frontiers_.clear();
  all_frontiers_clustered.clear();

  constructFrontiers(frontiers, frontiers_);
  constructFrontiers(frontier_cluster_centers,clustered_frontiers_);
  constructFrontiers(frontier_clusters,all_frontiers_clustered);

  bool visualization_requested = (visualization_pub_.getNumSubscribers() > 0);
  if (visualization_requested)
  {
    visualizeFrontiers(clustered_frontiers_);
  }
}

bool HectorExplorationPlanner::doExploration(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan)
{
  std::vector<geometry_msgs::PoseStamped> goals;

  updateFrontiers();
  {
    boost::mutex::scoped_lock lock(frontiers_mutex_);
    if (!frontiers_.empty()) {
      goals = frontiers_;
    }
  }

  plan.clear();
  bool frontiers_found = is_frontiers_found_;

  if(frontiers_found) {
    ROS_INFO("[hector_exploration_planner] exploration: found %u frontiers!", (unsigned int) goals.size());
  }

  auto info_gains = info_gain_client_->getInfoGain();
  // TODO: use the info gain in exploration transform

  // make plan
  if(!buildexploration_trans_array_(start,goals,true)){
    return false;
  }

  if(!getTrajectory(start,goals,plan)) {
    ROS_INFO("[hector_exploration_planner] exploration: could not plan to frontier, fail");
    return false;
  }

  // update previous goal
  if(!plan.empty()){
    geometry_msgs::PoseStamped thisgoal = plan.back();
    unsigned int mx,my;
    costmap_->worldToMap(thisgoal.pose.position.x,thisgoal.pose.position.y,mx,my);
    previous_goal_ = costmap_->getIndex(mx,my);
  }

  ROS_INFO("[hector_exploration_planner] exploration: plan to a frontier has been found! plansize: %u", (unsigned int)plan.size());
  return true;
}

float HectorExplorationPlanner::angleDifferenceWall(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal){
  // setup start positions
  unsigned int mxs,mys;
  costmap_->worldToMap(start.pose.position.x,start.pose.position.y,mxs,mys);

  unsigned int gx,gy;
  costmap_->worldToMap(goal.pose.position.x,goal.pose.position.y,gx,gy);

  int goal_proj_x = gx-mxs;
  int goal_proj_y = gy-mys;

  float start_angle = tf::getYaw(start.pose.orientation);
  float goal_angle = std::atan2(goal_proj_y,goal_proj_x);

  float both_angle = 0;
  if(start_angle > goal_angle){
    both_angle = start_angle - goal_angle;
  } else {
    both_angle = goal_angle - start_angle;
  }

  return both_angle;
}

void HectorExplorationPlanner::setupMapData()
{

#ifdef COSTMAP_2D_LAYERED_COSTMAP_H_
  costmap_ = costmap_ros_->getCostmap();
#else
  if (costmap_) delete costmap_;
  costmap_ = new costmap_2d::Costmap2D;
  costmap_ros_->getCostmapCopy(*costmap_);
#endif

  //Below code can be used to guarantee start pose is cleared. Somewhat risky.
  //@TODO: Make available through dynamic reconfigure
  /*
  std::vector<geometry_msgs::Point> points;
  costmap_ros_->getOrientedFootprint(points);

  bool filled = costmap_->setConvexPolygonCost(points, costmap_2d::FREE_SPACE);

  //std::vector<geometry_msgs::Point> points = costmap_ros_->getRobotFootprint();
  for (size_t i = 0; i < points.size(); ++i)
    std::cout << points[i];
  if (filled)
    ROS_INFO("Set costmap to free");
  else
    ROS_INFO("Failed to set costmap free");
  */

  if ((this->map_width_ != costmap_->getSizeInCellsX()) || (this->map_height_ != costmap_->getSizeInCellsY())){
    map_width_ = costmap_->getSizeInCellsX();
    map_height_ = costmap_->getSizeInCellsY();
    num_map_cells_ = map_width_ * map_height_;

    // initialize exploration_trans_array_, obstacle_trans_array_, goalMap and frontier_map_array_
    unsigned int *exploration_trans_array = new unsigned int[num_map_cells_];
    unsigned int *obstacle_trans_array = new unsigned int[num_map_cells_];
    bool *is_goal_array = new bool[num_map_cells_];
    int *frontier_map_array = new int[num_map_cells_];

    std::fill_n(exploration_trans_array, num_map_cells_, 0);
    std::fill_n(obstacle_trans_array, num_map_cells_, 0);
    std::fill_n(is_goal_array, num_map_cells_, false);
    std::fill_n(frontier_map_array, num_map_cells_, 0);

    exploration_trans_array_.reset(exploration_trans_array);
    obstacle_trans_array_.reset(obstacle_trans_array);
    is_goal_array_.reset(is_goal_array);
    frontier_map_array_.reset(frontier_map_array);
    frontiers_img_ = cv::Mat(map_height_, map_width_, CV_8UC1, cv::Scalar(0));
    clearFrontiers();
    resetMaps();
  }

  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  if(occupancy_grid_array_)
    delete []occupancy_grid_array_;

  occupancy_grid_array_ = new unsigned char[num_map_cells_];
  std::memcpy(occupancy_grid_array_, costmap_->getCharMap(), num_map_cells_);

//  occupancy_grid_array_ = costmap_->getCharMap();
}

void HectorExplorationPlanner::deleteMapData()
{
  if(occupancy_grid_array_)
    delete []occupancy_grid_array_;

  exploration_trans_array_.reset();
  obstacle_trans_array_.reset();
  is_goal_array_.reset();
  frontier_map_array_.reset();
}


bool HectorExplorationPlanner::buildexploration_trans_array_(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> goals, bool useAnglePenalty, bool use_cell_danger){

  ROS_DEBUG("[hector_exploration_planner] buildexploration_trans_array_");

  // reset exploration transform
  std::fill_n(exploration_trans_array_.get(), num_map_cells_, UINT_MAX);
  std::fill_n(is_goal_array_.get(), num_map_cells_, false);

  std::queue<int> myqueue;

  size_t num_free_goals = 0;

  // initialize goals
  for(unsigned int i = 0; i < goals.size(); ++i){
    // setup goal positions
    unsigned int mx,my;

    if(!costmap_->worldToMap(goals[i].pose.position.x,goals[i].pose.position.y,mx,my)){
      //ROS_WARN("[hector_exploration_planner] The goal coordinates are outside the costmap!");
      continue;
    }

    int goal_point = costmap_->getIndex(mx,my);

    // Ignore free goal for the moment, check after iterating over all goals if there is not valid one at all
    if(!isFree(goal_point)){
      continue;
    }else{
      ++num_free_goals;
    }

    unsigned int init_cost = 0;
    if(false){
      init_cost = angleDanger(angleDifference(start,goals[i])) * getDistanceWeight(start,goals[i]);
    }

    exploration_trans_array_[goal_point] = init_cost;

    // do not punish previous frontiers (oscillation)
    if(false && isValid(previous_goal_)){
      if(isSameFrontier(goal_point, previous_goal_)){
        ROS_DEBUG("[hector_exploration_planner] same frontier: init with 0");
        exploration_trans_array_[goal_point] = 0;
      }
    }

    if (exploration_trans_array_[goal_point]) {
      ROS_DEBUG("[hector_exploration_planner] Goal init cost: %d, point: %d", exploration_trans_array_[goal_point], goal_point);
    }
    is_goal_array_[goal_point] = true;
    frontiers_img_.data[goal_point] = 255;
    myqueue.push(goal_point);
  }

  if (num_free_goals == 0){
    ROS_WARN("[hector_exploration_planner] All goal coordinates for exploration transform invalid (occupied or out of bounds), aborting.");
    return false;
  }

  // exploration transform algorithm
  if (use_cell_danger){
    while(myqueue.size()){
      int point = myqueue.front();
      myqueue.pop();

      unsigned int minimum = exploration_trans_array_[point];

      int straightPoints[4];
      getStraightPoints(point,straightPoints);
      int diagonalPoints[4];
      getDiagonalPoints(point,diagonalPoints);

      // calculate the minimum exploration value of all adjacent cells
      for (int i = 0; i < 4; ++i) {
        if (isFree(straightPoints[i])) {
          unsigned int neighbor_cost = minimum + STRAIGHT_COST + cellDanger(straightPoints[i]);

          if (exploration_trans_array_[straightPoints[i]] > neighbor_cost) {
            exploration_trans_array_[straightPoints[i]] = neighbor_cost;
            myqueue.push(straightPoints[i]);
          }
        }

        if (isFree(diagonalPoints[i])) {
          unsigned int neighbor_cost = minimum + DIAGONAL_COST + cellDanger(diagonalPoints[i]);

          if (exploration_trans_array_[diagonalPoints[i]] > neighbor_cost) {
            exploration_trans_array_[diagonalPoints[i]] = neighbor_cost;
            myqueue.push(diagonalPoints[i]);
          }
        }
      }
    }
  }else{
    while(myqueue.size()){
      int point = myqueue.front();
      myqueue.pop();

      unsigned int minimum = exploration_trans_array_[point];

      int straightPoints[4];
      getStraightPoints(point,straightPoints);
      int diagonalPoints[4];
      getDiagonalPoints(point,diagonalPoints);

      // calculate the minimum exploration value of all adjacent cells
      for (int i = 0; i < 4; ++i) {
        if (isFree(straightPoints[i])) {
          unsigned int neighbor_cost = minimum + STRAIGHT_COST;

          if (exploration_trans_array_[straightPoints[i]] > neighbor_cost) {
            exploration_trans_array_[straightPoints[i]] = neighbor_cost;
            myqueue.push(straightPoints[i]);
          }
        }

        if (isFree(diagonalPoints[i])) {
          unsigned int neighbor_cost = minimum + DIAGONAL_COST;

          if (exploration_trans_array_[diagonalPoints[i]] > neighbor_cost) {
            exploration_trans_array_[diagonalPoints[i]] = neighbor_cost;
            myqueue.push(diagonalPoints[i]);
          }
        }
      }
    }
  }

  ROS_DEBUG("[hector_exploration_planner] END: buildexploration_trans_array_");

  // drawExplorationTransform(exploration_trans_array_, *costmap_, exploration_trans_img_);
  vis_->publishVisOnDemand(*costmap_, exploration_trans_array_.get());
  return true;
}

void HectorExplorationPlanner::drawExplorationTransform(const boost::shared_array<unsigned int> exploration_array,
                                                        const costmap_2d::Costmap2D& costmap,
                                                        cv::Mat &img)
{
  unsigned int size_x = costmap.getSizeInCellsX();
  unsigned int size_y = costmap.getSizeInCellsY();
  unsigned int size = size_x * size_y;

  img = cv::Mat(size_y, size_x, CV_8UC1);

  unsigned int max = 0;
#define MAX_TRANSFORM 10000

  for (size_t i = 0; i < size; ++i) {
    if ((exploration_array[i] < INT_MAX) && (exploration_array[i] > max)) {
      max = exploration_array[i];
    }
    if (max > MAX_TRANSFORM) {
      max = MAX_TRANSFORM;
      break;
    }
  }

  for (size_t i = 0; i < size; ++i) {
    if (exploration_array[i] < INT_MAX) {
      unsigned int transform = exploration_array[i];
      if (transform > MAX_TRANSFORM) {
        transform = MAX_TRANSFORM;
      }
      img.data[i] = static_cast<uint8_t>(static_cast<float>(transform) / max * 255);
    } else {
      img.data[i] = 255;
    }
  }
}

bool HectorExplorationPlanner::buildobstacle_trans_array_(bool use_inflated_obstacles){
  ROS_DEBUG("[hector_exploration_planner] buildobstacle_trans_array_");
  std::queue<int> myqueue;

  // init obstacles
  for(unsigned int i=0; i < num_map_cells_; ++i){
    if(occupancy_grid_array_[i] == costmap_2d::LETHAL_OBSTACLE){
      myqueue.push(i);
      obstacle_trans_array_[i] = 0;
    } else if(use_inflated_obstacles){
      if(occupancy_grid_array_[i] == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        myqueue.push(i);
        obstacle_trans_array_[i] = 0;
      }
    }
  }

  unsigned int obstacle_cutoff_value = static_cast<unsigned int>((p_obstacle_cutoff_dist_ / costmap_->getResolution()) * STRAIGHT_COST + 0.5);

  // obstacle transform algorithm
  while(myqueue.size()){
    int point = myqueue.front();
    myqueue.pop();

    unsigned int minimum = obstacle_trans_array_[point];
    if (minimum > obstacle_cutoff_value) continue;

    int straightPoints[4];
    getStraightPoints(point,straightPoints);
    int diagonalPoints[4];
    getDiagonalPoints(point,diagonalPoints);

    // check all 8 directions
    for(int i = 0; i < 4; ++i){
      if (isValid(straightPoints[i]) && (obstacle_trans_array_[straightPoints[i]] > minimum + STRAIGHT_COST)) {
        obstacle_trans_array_[straightPoints[i]] = minimum + STRAIGHT_COST;
        myqueue.push(straightPoints[i]);
      }
      if (isValid(diagonalPoints[i]) && (obstacle_trans_array_[diagonalPoints[i]] > minimum + DIAGONAL_COST)) {
        obstacle_trans_array_[diagonalPoints[i]] = minimum + DIAGONAL_COST;
        myqueue.push(diagonalPoints[i]);
      }
    }
  }

  ROS_DEBUG("[hector_exploration_planner] END: buildobstacle_trans_array_");

  obstacle_vis_->publishVisOnDemand(*costmap_, obstacle_trans_array_.get());
  return true;
}

bool HectorExplorationPlanner::getTrajectory(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> goals, std::vector<geometry_msgs::PoseStamped> &plan){

  ROS_DEBUG("[hector_exploration_planner] getTrajectory");

  // setup start positions
  unsigned int mx,my;

  if(!costmap_->worldToMap(start.pose.position.x,start.pose.position.y,mx,my)){
    ROS_WARN("[hector_exploration_planner] The start coordinates are outside the costmap!");
    return false;
  }

  int currentPoint = costmap_->getIndex(mx,my);
  int nextPoint = currentPoint;

  geometry_msgs::PoseStamped trajPoint;
  std::string global_frame = costmap_ros_->getGlobalFrameID();
  trajPoint.header.frame_id = global_frame;

  if (is_goal_array_[currentPoint]){
    ROS_INFO("Already at goal point position. No pose vector generated.");
    return true;
  }

  while(!is_goal_array_[currentPoint]){
    int thisDelta;
    int adjacentPoints[8];
    getAdjacentPoints(currentPoint,adjacentPoints);

    int maxDelta = 0;

    for(int i = 0; i < 8; ++i){
      if(isFree(adjacentPoints[i])){
        thisDelta = exploration_trans_array_[currentPoint] - exploration_trans_array_[adjacentPoints[i]];
        if(thisDelta > maxDelta){
          maxDelta = thisDelta;
          nextPoint = adjacentPoints[i];
        }
      }
    }

    // This happens when there is no valid exploration transform data at the start point for example
    if(maxDelta == 0){
      ROS_WARN("[hector_exploration_planner] No path to the goal could be found by following gradient!");
      return false;
    }


    // make trajectory point
    unsigned int sx,sy,gx,gy;
    costmap_->indexToCells((unsigned int)currentPoint,sx,sy);
    costmap_->indexToCells((unsigned int)nextPoint,gx,gy);
    double wx,wy;
    costmap_->mapToWorld(sx,sy,wx,wy);

    trajPoint.pose.position.x = wx;
    trajPoint.pose.position.y = wy;
    trajPoint.pose.position.z = 0.0;

    // assign orientation
    int dx = gx-sx;
    int dy = gy-sy;
    double yaw_path = std::atan2(dy,dx);
    trajPoint.pose.orientation.x = 0.0;
    trajPoint.pose.orientation.y = 0.0;
    trajPoint.pose.orientation.z = sin(yaw_path*0.5f);
    trajPoint.pose.orientation.w = cos(yaw_path*0.5f);

    plan.push_back(trajPoint);

    currentPoint = nextPoint;
  }

  if (smoothing_enabled_ && path_smoother_ && plan.size() > smoothed_points_per_unit_)
  {
    nav_msgs::Path original_path;
    original_path.header.frame_id = global_frame;
    original_path.header.stamp = ros::Time::now();
    original_path.poses = plan;

    auto check_path_consistency_lambda = [this](nav_msgs::Path path) {
      if (path.poses.size() >= 2)
      {
        auto ultimate_pose = path.poses.back();
        auto penultimate_pose = path.poses[path.poses.size() - 2];
        return (std::hypot(ultimate_pose.pose.position.x - penultimate_pose.pose.position.x,
                           ultimate_pose.pose.position.y - penultimate_pose.pose.position.y)
                < 2.0 / smoothed_points_per_unit_);
      }
      else
      {
        return true;
      }
    };

    if (!check_path_consistency_lambda(original_path))
    {
      ROS_WARN("[hector_exploration_planner] raw path has inconsistent end point");
    }

    nav_msgs::Path smoothed_path;
    {
      boost::mutex::scoped_lock lock(path_smoother_mutex_);
      path_smoother_->interpolatePath(original_path, smoothed_path);
    }


    if (!check_path_consistency_lambda(smoothed_path))
    {
      ROS_WARN("[hector_exploration_planner] smoothed path has inconsistent end point, using raw path end point");
      if (!smoothed_path.poses.empty() && !original_path.poses.empty())
      {
        smoothed_path.poses.back() = original_path.poses.back();
      }
    }

    plan = smoothed_path.poses;
  }

  ROS_DEBUG("[hector_exploration_planner] END: getTrajectory. Plansize %u", (unsigned int)plan.size());
  return !plan.empty();
}

bool HectorExplorationPlanner::constructFrontier(int point, geometry_msgs::PoseStamped& frontier)
{
  if(this->isValid(point))
  {
    double wx, wy;
    unsigned int mx, my;
    costmap_->indexToCells(point, mx, my);
    costmap_->mapToWorld(mx, my, wx, wy);
    std::string global_frame = costmap_ros_->getGlobalFrameID();
    frontier.header.frame_id = global_frame;
    frontier.pose.position.x = wx;
    frontier.pose.position.y = wy;
    frontier.pose.position.z = 0.0;

    double yaw = getYawToUnknown(costmap_->getIndex(mx, my));

    frontier.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    return true;
  }
  return false;
}

bool HectorExplorationPlanner::constructFrontiers(const std::vector<int> &points,
                                                  std::vector<geometry_msgs::PoseStamped> &frontiers)
{
  bool success = true;
  for(int p: points)
  {
    geometry_msgs::PoseStamped frontier;
    constructFrontier(p, frontier);
    frontiers.push_back(frontier);
  }
  return true;
}


bool HectorExplorationPlanner::constructFrontiers(const std::vector<std::vector<int>> &points,
                                                  std::vector<std::vector<geometry_msgs::PoseStamped>> &clusters)
{
  for (auto cluster: points)
  {
    std::vector<geometry_msgs::PoseStamped> frontier_cluster;
    constructFrontiers(cluster, frontier_cluster);
    clusters.push_back(frontier_cluster);
  }
}

bool HectorExplorationPlanner::findAllFrontiers(std::vector<int>& allFrontiers)
{
  allFrontiers.clear();
  // check for all cells in the occupancy grid whether or not they are frontier cells
  for (unsigned int i = 0; i < num_map_cells_; ++i)
  {
    if (isFrontier(i))
    {
      allFrontiers.push_back(i);
    }
  }

  return (allFrontiers.size() > 0);
}

bool HectorExplorationPlanner::findFrontiers(std::vector<int> &frontiers)
{
  frontiers.clear();

  // check for all cells in the occupancy grid whether or not they are frontier cells
  for (unsigned int i = 0; i < num_map_cells_; ++i)
  {
    if (isFrontier(i) && !isFrontierReached(i))
    {
      frontiers.push_back(i);
    }
  }

  return (frontiers.size() > 0);
}

int HectorExplorationPlanner::centerPoint(const std::vector<int>& frontier_cluster)
{
  if (frontier_cluster.size() <= 0)
    return false;
  float x=0, y=0;
  for(int i = 0; i < frontier_cluster.size(); i++)
  {
    unsigned int tx, ty;
    costmap_->indexToCells(frontier_cluster[i],tx,ty);
    x += tx;
    y += ty;
  }
  int fx = (int) (x / frontier_cluster.size());
  int fy = (int) (y / frontier_cluster.size());
  int index = costmap_->getIndex(fx, fy);
  return index;
}

int HectorExplorationPlanner::maxObstaclePoint(const std::vector<int>& frontier_cluster)
{
  int max_obs_idx = frontier_cluster[0];
  for(int cur: frontier_cluster)
  {
      if (obstacle_trans_array_[cur] > obstacle_trans_array_[max_obs_idx])
          max_obs_idx = cur;
  }
  return max_obs_idx;
}

int HectorExplorationPlanner::getFrontierClusterCenter(const std::vector<int> &cluster)
{
  assert(cluster && cluster.size() != 0);
  return maxObstaclePoint(cluster);
}

std::vector<int> HectorExplorationPlanner::getFrontierClusterCenters(
    const std::vector<std::vector<int>> &clusters)
{
  std::vector<int> cluster_centers;
  for(auto cluster: clusters)
  {
    int center = getFrontierClusterCenter(cluster);
    cluster_centers.push_back(center);
  }
  return cluster_centers;
}

bool HectorExplorationPlanner::clusterFrontiersRemoveSmall(const std::vector<int>& all_frontiers,
                                 std::vector<std::vector<int>>& frontier_clusters)
{
  frontier_clusters.clear();

  std::vector<std::vector<int>> frontier_clusters_tmp;
  bool isSuccess = clusterFrontiers(all_frontiers, frontier_clusters_tmp);
  for(std::vector<int> cluster: frontier_clusters_tmp)
  {
    if(cluster.size() >= cluster_min_number)
      frontier_clusters.push_back(cluster);
  }
  return isSuccess;
}

// group adjoining frontier points just index
bool HectorExplorationPlanner::clusterFrontiers(const std::vector<int>& all_frontiers,
                                                std::vector<std::vector<int>>& frontier_clusters)
{
  frontier_clusters.clear();
  if (all_frontiers.size() <= 0)
    return false;

  const int FRONTIER_VALUE = -1;
  int frontier_cluster_map[num_map_cells_] = {0};
  //boost::scoped_array<int> frontier_cluster_map(new int[num_map_cells_]);

  // set all frontiers points value as -1 in frontier_cluster_map
  for(int i = 0; i < all_frontiers.size(); i++)
  {
    frontier_cluster_map[all_frontiers[i]] = FRONTIER_VALUE;
  }

  // cluster_id begins from 1 and add 1 when there is a new cluster
  int cluster_id = 1;

  for(int i = 0; i < all_frontiers.size(); i++)
  {
    int cur_idx = all_frontiers[i];
    // if current frontier has not been clustered into an existing cluster
    if(frontier_cluster_map[cur_idx] == FRONTIER_VALUE)
    {
      std::queue<int> neighbors;  // used as an queue to find all conjoint frontier points
      std::vector<int> single_cluster; // save all frontier points in a cluster
      neighbors.push(cur_idx);
      frontier_cluster_map[cur_idx] = cluster_id; //set value as cluster_id if the point has been visited

      while(neighbors.size() > 0)
      {
        int idx = neighbors.front();
        neighbors.pop();
        single_cluster.push_back(idx);

        // consider neighborhoods with distance = this->neighbor_distance
        int neighbor_num = this->getNeigoborsNumber(this->neighbor_distance);
//        std::cout << "neighbor_num: " << neighbor_num << std::endl;

        int adjacentPoints[neighbor_num];
        getNeighbors(idx, adjacentPoints, this->neighbor_distance);
        for(int j = 0; j < neighbor_num; j++)
        {
          // if it is valid points and its value is FRONTIER_VALUE and it has not been visited (not positive value)
          if(isValid(adjacentPoints[j]) && frontier_cluster_map[adjacentPoints[j]] == FRONTIER_VALUE)
          {
            neighbors.push(adjacentPoints[j]);
            frontier_cluster_map[adjacentPoints[j]] = cluster_id; //set value as cluster_id if the point has been visited
          }
        }
      }
      frontier_clusters.push_back(single_cluster);
      cluster_id ++;
    }
  }

  int total_size = 0;
  for(auto cluster: frontier_clusters)
  {
    total_size += cluster.size();
  }

  std::cout << "Frontier size in clusters and original one: " << total_size
            << " vs " << all_frontiers.size() << std::endl;

  return !frontier_clusters.empty();
}

void HectorExplorationPlanner::visualizeFrontiers(std::vector<geometry_msgs::PoseStamped>& clusteredfrontiers)
{
  static visualization_msgs::MarkerArray markers;

  for (auto &marker: markers.markers) {
    marker.action = visualization_msgs::Marker::DELETE;
  }

  visualization_pub_.publish(markers);
  markers.markers.clear();

  int id = 1;
  for(int i = 0; i < clusteredfrontiers.size(); i++)
  {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time();
      marker.ns = "hector_exploration_planner";
      marker.id = id++;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::DELETE;
      visualization_pub_.publish(marker);

      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = clusteredfrontiers[i].pose.position.x;
      marker.pose.position.y = clusteredfrontiers[i].pose.position.y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation = clusteredfrontiers[i].pose.orientation;
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.color.a = 1.0;

      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.lifetime = ros::Duration(0); // (50,0);
      markers.markers.push_back(marker);
  }

  visualization_pub_.publish(markers);
}

/*
 * checks if a given point is a frontier cell. a frontier cell is a cell in the occupancy grid
 * that seperates known from unknown space. Therefore the cell has to be free but at least three
 * of its neighbours need to be unknown
 */
bool HectorExplorationPlanner::isFrontier(int point){
  if(isFreeFrontiers(point)){

    int adjacentPoints[8];
    getAdjacentPoints(point,adjacentPoints);

    for(int i = 0; i < 8; ++i){
      if(isValid(adjacentPoints[i]) && occupancy_grid_array_[adjacentPoints[i]] == costmap_2d::NO_INFORMATION){
        int no_inf_count = 0;
        int noInfPoints[8];
        getAdjacentPoints(adjacentPoints[i],noInfPoints);
        for(int j = 0; j < 8; j++){
          if( isValid(noInfPoints[j]) && occupancy_grid_array_[noInfPoints[j]] == costmap_2d::NO_INFORMATION){
            ++no_inf_count;

            if(no_inf_count > 1)
              return true;
          }
        }
      }
    }
  }

  return false;
}


void HectorExplorationPlanner::resetMaps(){
  std::fill_n(exploration_trans_array_.get(), num_map_cells_, UINT_MAX);
  std::fill_n(obstacle_trans_array_.get(), num_map_cells_, UINT_MAX);
  std::fill_n(is_goal_array_.get(), num_map_cells_, false);
}

void HectorExplorationPlanner::clearFrontiers(){
  std::fill_n(frontier_map_array_.get(), num_map_cells_, 0);
}

inline bool HectorExplorationPlanner::isValid(int point){
  return (point>=0);
}

bool HectorExplorationPlanner::isFree(int point){

  if(isValid(point)){
    // if a point is not inscribed_inflated_obstacle, leathal_obstacle or no_information, its free

    if(occupancy_grid_array_[point] == costmap_2d::FREE_SPACE){
      return true;
    }
//    if(p_use_inflated_obs_){
//      if(occupancy_grid_array_[point] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
//        return true;
//      }
//    } else {
//      if(occupancy_grid_array_[point] <= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
//        return true;
//      }
//    }

    if(p_plan_in_unknown_){
      if(occupancy_grid_array_[point] == costmap_2d::NO_INFORMATION){
        return true;
      }
    }
  }
  return false;
}

bool HectorExplorationPlanner::isFreeFrontiers(int point){

  if(isValid(point)){
    // if a point is not inscribed_inflated_obstacle, leathal_obstacle or no_information, its free

    if(occupancy_grid_array_[point] == costmap_2d::FREE_SPACE){
        return true;
    }

//    if(p_use_inflated_obs_){
//      if(occupancy_grid_array_[point] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
//        return true;
//      }
//    } else {
//      if(occupancy_grid_array_[point] <= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
//        return true;
//      }
//    }
  }
  return false;
}

bool HectorExplorationPlanner::isFrontierReached(int point){

  tf::Stamped<tf::Pose> robotPose;

  if(!costmap_ros_->getRobotPose(robotPose)) {
    ROS_WARN("[hector_exploration_planner]: Failed to get RobotPose");
  }
  geometry_msgs::PoseStamped robotPoseMsg;
  tf::poseStampedTFToMsg(robotPose, robotPoseMsg);

  unsigned int fx,fy;
  double wfx,wfy;
  costmap_->indexToCells(point,fx,fy);
  costmap_->mapToWorld(fx,fy,wfx,wfy);


  double dx = robotPoseMsg.pose.position.x - wfx;
  double dy = robotPoseMsg.pose.position.y - wfy;

  if ( (dx*dx) + (dy*dy) < (p_dist_for_goal_reached_*p_dist_for_goal_reached_)) {
    ROS_DEBUG("[hector_exploration_planner]: frontier is within the squared range of: %f", p_dist_for_goal_reached_);
    return true;
  }
  return false;

}

bool HectorExplorationPlanner::isSameFrontier(int frontier_point1, int frontier_point2){
  unsigned int fx1,fy1;
  unsigned int fx2,fy2;
  double wfx1,wfy1;
  double wfx2,wfy2;
  costmap_->indexToCells(frontier_point1,fx1,fy1);
  costmap_->indexToCells(frontier_point2,fx2,fy2);
  costmap_->mapToWorld(fx1,fy1,wfx1,wfy1);
  costmap_->mapToWorld(fx2,fy2,wfx2,wfy2);

  double dx = wfx1 - wfx2;
  double dy = wfy1 - wfy2;

  if((dx*dx) + (dy*dy) < (p_same_frontier_dist_*p_same_frontier_dist_)){
    return true;
  }
  return false;
}

inline unsigned int HectorExplorationPlanner::cellDanger(int point){

  if ((int)obstacle_trans_array_[point] <= p_min_obstacle_dist_){
    return static_cast<unsigned int>(p_alpha_ * std::pow(p_min_obstacle_dist_ - obstacle_trans_array_[point], 2) + .5);
  }
  //ROS_INFO("%d", (int)obstacle_trans_array_[point] );
  //return 80000u - std::min(80000u, obstacle_trans_array_[point]*40);

  //return (2000u - std::min(2000u, obstacle_trans_array_[point])) / 500u;
  //std::cout << obstacle_trans_array_[point] << "\n";

  return 0;
}

float HectorExplorationPlanner::angleDifference(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal){
  // setup start positions
  unsigned int mxs,mys;
  costmap_->worldToMap(start.pose.position.x,start.pose.position.y,mxs,mys);

  unsigned int gx,gy;
  costmap_->worldToMap(goal.pose.position.x,goal.pose.position.y,gx,gy);

  int goal_proj_x = gx-mxs;
  int goal_proj_y = gy-mys;

  float start_angle = tf::getYaw(start.pose.orientation);
  float goal_angle = std::atan2(goal_proj_y,goal_proj_x);

  float both_angle = 0;
  if(start_angle > goal_angle){
    both_angle = start_angle - goal_angle;
  } else {
    both_angle = goal_angle - start_angle;
  }

  if(both_angle > M_PI){
    both_angle = (M_PI - std::abs(start_angle)) + (M_PI - std::abs(goal_angle));
  }

  return both_angle;
}

// Used to generate direction for frontiers
double HectorExplorationPlanner::getYawToUnknown(int point){
  int adjacentPoints[8];
  getAdjacentPoints(point,adjacentPoints);

  int max_obs_idx = 0;
  unsigned int max_obs_dist = 0;

  for(int i = 0; i < 8; ++i){
    if(isValid(adjacentPoints[i])){
      if(occupancy_grid_array_[adjacentPoints[i]] == costmap_2d::NO_INFORMATION){
        if(obstacle_trans_array_[adjacentPoints[i]] > max_obs_dist){
          max_obs_idx = i;
          max_obs_dist = obstacle_trans_array_[adjacentPoints[i]];
        }
      }
    }
  }

  int orientationPoint = adjacentPoints[max_obs_idx];
  unsigned int sx,sy,gx,gy;
  costmap_->indexToCells((unsigned int)point,sx,sy);
  costmap_->indexToCells((unsigned int)orientationPoint,gx,gy);
  int x = gx-sx;
  int y = gy-sy;
  double yaw = std::atan2(y,x);

  return yaw;

}

unsigned int HectorExplorationPlanner::angleDanger(float angle){
  float angle_fraction = std::pow(angle,3);///M_PI;
  unsigned int result = static_cast<unsigned int>(p_goal_angle_penalty_ * angle_fraction);
  return result;
}

float HectorExplorationPlanner::getDistanceWeight(const geometry_msgs::PoseStamped &point1, const geometry_msgs::PoseStamped &point2){
  float distance = std::sqrt(std::pow(point1.pose.position.x - point2.pose.position.x,2) + std::pow(point1.pose.position.y - point2.pose.position.y,2));
  if(distance < 0.5){
    return 5.0;
  } else {
    return 1;
  }
}

/*
 These functions calculate the index of an adjacent point (left,upleft,up,upright,right,downright,down,downleft) to the
 given point. If there is no such point (meaning the point would cause the index to be out of bounds), -1 is returned.
 */
inline void HectorExplorationPlanner::getStraightPoints(int point, int points[]){

  points[0] = left(point);
  points[1] = up(point);
  points[2] = right(point);
  points[3] = down(point);

}

inline void HectorExplorationPlanner::getDiagonalPoints(int point, int points[]){

  points[0] = upleft(point);
  points[1] = upright(point);
  points[2] = downright(point);
  points[3] = downleft(point);

}

void HectorExplorationPlanner::getNeighbors(int point, int points[], int dis)
{
  unsigned int mx, my;
  int idx = 0;
  costmap_->indexToCells(point, mx, my);
  for(int dx = -dis; dx <= dis; dx++)
  {
    for(int dy = -dis; dy <= dis; dy++)
    {
      if(dx == 0 && dy == 0)
        continue;
      else
      {
        int x = mx + dx;
        int y = my + dy;
        if(x < 0 || x >= map_width_ || y < 0 || y >= map_height_)
          points[idx++] = -1;
        else
          points[idx++] = costmap_->getIndex(x, y);
      }
    }
  }
}

int HectorExplorationPlanner::getNeigoborsNumber(int dis)
{
  int l = dis * 2 + 1;
  return l*l - 1;
}

inline void HectorExplorationPlanner::getAdjacentPoints(int point, int points[]){

  points[0] = left(point);
  points[1] = up(point);
  points[2] = right(point);
  points[3] = down(point);
  points[4] = upleft(point);
  points[5] = upright(point);
  points[6] = downright(point);
  points[7] = downleft(point);

}

inline int HectorExplorationPlanner::left(int point){
  // only go left if no index error and if current point is not already on the left boundary
  if((point % map_width_ != 0)){
    return point-1;
  }
  return -1;
}
inline int HectorExplorationPlanner::upleft(int point){
  if((point % map_width_ != 0) && (point >= (int)map_width_)){
    return point-1-map_width_;
  }
  return -1;

}
inline int HectorExplorationPlanner::up(int point){
  if(point >= (int)map_width_){
    return point-map_width_;
  }
  return -1;
}
inline int HectorExplorationPlanner::upright(int point){
  if((point >= (int)map_width_) && ((point + 1) % (int)map_width_ != 0)){
    return point-map_width_+1;
  }
  return -1;
}
inline int HectorExplorationPlanner::right(int point){
  if((point + 1) % map_width_ != 0){
    return point+1;
  }
  return -1;

}
inline int HectorExplorationPlanner::downright(int point){
  if(((point + 1) % map_width_ != 0) && ((point/map_width_) < (map_height_-1))){
    return point+map_width_+1;
  }
  return -1;

}
inline int HectorExplorationPlanner::down(int point){
  if((point/map_width_) < (map_height_-1)){
    return point+map_width_;
  }
  return -1;

}
inline int HectorExplorationPlanner::downleft(int point){
  if(((point/map_width_) < (map_height_-1)) && (point % map_width_ != 0)){
    return point+map_width_-1;
  }
  return -1;

}

