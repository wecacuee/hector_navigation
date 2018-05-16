//
// Created by rakesh on 14/05/18.
//

#include <hector_exploration_planner/frontier_analysis.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <queue>
#include <map>

#include <ros/ros.h>

namespace hector_exploration_planner
{
namespace frontier_analysis
{

void preprocessFrontierImg(cv::Mat &frontier_img_in, cv::Mat &frontier_img_out)
{
  cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                               cv::Size( 2*FRONTIER_OPENING_KERNEL + 1, 2*FRONTIER_OPENING_KERNEL+1 ),
                                               cv::Point( FRONTIER_OPENING_KERNEL, FRONTIER_OPENING_KERNEL ) );

  cv::Mat temp_img = frontier_img_in.clone();

  // closing (hole filling)
  cv::dilate(frontier_img_in, temp_img, element);
  cv::erode(temp_img, frontier_img_out, element);

// opening (cleaning)
//  element = cv::getStructuringElement( cv::MORPH_RECT,
//                                       cv::Size( 2*FRONTIER_HOLE_FILLING_KERNEL + 1, 2*FRONTIER_HOLE_FILLING_KERNEL+1 ),
//                                       cv::Point( FRONTIER_HOLE_FILLING_KERNEL, FRONTIER_HOLE_FILLING_KERNEL ) );
//  cv::dilate(frontier_img_out, temp_img, element);
//  cv::erode(temp_img, frontier_img_out, element);

}

std::vector< std::vector<cv::Point> > groupFrontiers(cv::Mat &frontier_img,
                                                     std::vector<cv::Point> clustered_frontier_points)
{
//  std::map<cv::Point, bool> already_grouped;
//
//  std::vector< std::vector<cv::Point> > grouped_frontiers;
//
//  for (const auto &frontier_point: clustered_frontier_points)
//  {
//    if (!frontier_img.at<uint8_t>(frontier_point))
//    {
//      std::cout << "CLUSTERED POINT NOT IN FRONTIER" << std::endl;
//      continue;
//    }
//    cv::Mat temp_img = frontier_img.clone();
//    cv::floodFill(temp_img, frontier_point, cv::Scalar(255), nullptr, cv::Scalar(0), cv::Scalar(0));
//    cv::Mat non_zero_coordinates;
//    cv::findNonZero(temp_img, non_zero_coordinates);
//    // TODO: check if they've already been grouped into some other group
//    std::vector<cv::Point> group;
//    for (size_t i = 0; i < non_zero_coordinates.total(); i++)
//    {
//      group.push_back(non_zero_coordinates.at<cv::Point>(i));
//    }
//    grouped_frontiers.push_back(group);
//  }
//
//  return grouped_frontiers;

  std::vector< std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(frontier_img, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

  std::vector< std::vector<cv::Point> > filtered_contours;
  std::copy_if(
    contours.begin(), contours.end(),
    std::back_inserter(filtered_contours),
    [](std::vector<cv::Point> group) {
      return group.size() > MIN_FRONTIER_CLUSTER_SIZE;
    }
  );
  return filtered_contours;
}

void colorFrontiers(cv::Mat &frontier_img,
                    std::vector< std::vector<cv::Point> > grouped_frontiers,
                    cv::RNG &rng,
                    cv::Mat &colored_frontier_img)
{
//  colored_frontier_img = cv::Mat(frontier_img.size(), CV_8UC3);
  for (size_t i = 0; i < grouped_frontiers.size(); i++)
  {
    cv::Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    cv::drawContours(colored_frontier_img, grouped_frontiers, i, color, 2);
  }
}

void loadStageWorld(std::string bitmap,
                    cv::Size2f size,
                    const costmap_2d::Costmap2D& costmap,
                    cv::Mat &output_img)
{
  cv::Mat resized_img;
  cv::Mat output_img_flipped;
  cv::Mat raw_img = cv::imread(bitmap, CV_LOAD_IMAGE_UNCHANGED);
  // fill borders
  raw_img.colRange(0, raw_img.cols).rowRange(0, 1) = cv::Scalar(0);
  raw_img.colRange(0, raw_img.cols).rowRange(raw_img.rows-1, raw_img.rows) = cv::Scalar(0);
  raw_img.rowRange(0, raw_img.rows).colRange(0, 1) = cv::Scalar(0);
  raw_img.rowRange(0, raw_img.rows).colRange(raw_img.cols-1, raw_img.cols) = cv::Scalar(0);

  auto resolution = costmap.getResolution();
  cv::Size output_size(
    static_cast<int>(std::round(size.width  / resolution)),
    static_cast<int>(std::round(size.height / resolution))
  );
  cv::resize(raw_img, resized_img, output_size);
  resized_img = 255 - resized_img;

  output_img_flipped = cv::Mat::zeros(cv::Size(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()), CV_8UC1);
  cv::Size costmap_center(std::round(costmap.getSizeInCellsX()/(float)2), std::round(costmap.getSizeInCellsY()/(float)2));
  cv::Size bitmap_center(std::round(resized_img.cols/(float)2), std::round(resized_img.rows/(float)2));

  // TODO: fix the funny offset business
  resized_img.copyTo(
    output_img_flipped
      .colRange(costmap_center.width - bitmap_center.width + 8, costmap_center.width + bitmap_center.width + 8)
      .rowRange(costmap_center.height - bitmap_center.height - 8, costmap_center.height + bitmap_center.height - 8)
  );

  cv::flip(output_img_flipped, output_img, 0);
}

//std::vector<cv::Point> getClosestUnknowns(cv::Mat &occupancy_unknown_img,
//                                          std::vector< std::vector<cv::Point> > grouped_frontiers)
//{
//  std::vector<cv::Point> closest_unknowns;
//  closest_unknowns.reserve(grouped_frontiers.size());
//
//  for (const auto &frontier_group: grouped_frontiers)
//  {
//    cv::Point closest_point;
//    if (frontier_group.empty())
//    {
//      ROS_ERROR("Frontier group empty");
//    }
//
//    bool is_found = false;
//    auto point = frontier_group.front();
//    std::queue<cv::Point> queue;
//    queue.push(point);
//
//    while (!queue.empty())
//    {
//      for (size_t i =0; i < )
//    }
//
//  }
//}

} // namespace frontier_analysis
} // namespace hector_exploration_planner