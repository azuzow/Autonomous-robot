//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

using namespace std;

namespace slam {

struct Pose {
  Eigen::Vector2f loc;
  float angle;
  float log_likelihood = 0.0;
};


class SLAM {
 public:
  // Default Constructor.
  SLAM();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;
  void makeProbTable(Eigen::Vector2f point);
  void construct_obs_prob_table();
  Pose CorrelativeScanMatching(const vector<float>& ranges, float angle_min, float angle_max);
  void add_new_points_in_map(Pose current_best_pose, const vector<float>& ranges, float angle_min, float angle_max);
  Eigen::Vector2f rotation( Eigen::Vector2f local_frame_loc, float local_frame_angle, Eigen::Vector2f point_in_local_frame );
  Eigen::Vector2f convert_scan_prev_pose(Pose particle_pose, Eigen::Vector2f laser_point);
  void motion_model(float distance, float angle,float x_translation_error_stddev, float y_translation_error_stddev,float rotation_error_stddev);

 private:

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;
  bool odom_observed;
  int num_points_in_final_plot = 1000;

  Pose current_best_pose;

  // Keep symetric in x and y direction
  int min_x_val = -10;
  int max_x_val = 10;
  int min_y_val = -10;
  int max_y_val = 10;

  // Observation likelihood table
  int obs_prob_table_width;
  int obs_prob_table_height;
  float delta_distance = 0.05;
  vector< vector<float> > obs_prob_table;
  float std_obs_likelihood = 0.01;

  //CorrelativeScanMatching
  float obs_weight = 3.0/1000;
  float motion_weight = 1.0/3;

  // Constructed map to plot
  vector<Eigen::Vector2f> constructed_map;

  float current_angle;
  Eigen::Vector2f current_loc;
  Eigen::Vector2f last_likelihood_scan_loc;
  float last_likelihood_scan_angle;
  bool calculate_likelihoods;
  bool obs_prob_table_init = false;
  bool use_laser = false;
  std::vector<Pose> poses;
  int x_resolution;
  int y_resolution;
  int theta_resolution;
};
}  // namespace slam

#endif   // SRC_SLAM_H_
