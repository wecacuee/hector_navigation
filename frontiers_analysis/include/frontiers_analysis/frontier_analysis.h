//
// Created by rakesh on 14/05/18.
//

#ifndef HECTOR_NAVIGATION_FRONTIER_ANALYSIS_H
#define HECTOR_NAVIGATION_FRONTIER_ANALYSIS_H

// preprocessing operations on frontiers
#define FRONTIER_HOLE_FILLING_KERNEL 1
#define FRONTIER_OPENING_KERNEL 1

#include <opencv2/core/core.hpp>

#include <vector>

namespace hector_exploration_planner
{
namespace frontier_analysis
{

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
std::vector< std::vector<cv::Point> > groupFrontiers(cv::Mat &frontier_img,
                                                     std::vector<cv::Point> clustered_frontier_points);

/**
 *
 * @brief color the frontier image based on clusters
 * @param frontier_img input frontier binary image (single channeled)
 * @param grouped_frontiers coordinates of the frontiers grouped into clusters
 * @param rng random number generator
 * @param colored_frontier_img output image (three channeled)
 */
void colorFrontiers(cv::Mat &frontier_img,
                    std::vector< std::vector<cv::Point> > grouped_frontiers,
                    cv::RNG &rng,
                    cv::Mat &colored_frontier_img);

} // namespace frontier_analysis
} // namespace hector_exploration_planner

#endif //HECTOR_NAVIGATION_FRONTIER_ANALYSIS_H
