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
\file    slam.cc
\brief   SLAM Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "slam.h"

#include "vector_map/vector_map.h"

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

namespace slam {

SLAM::SLAM() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = current_best_pose.loc;
  *angle = current_best_pose.angle;
}


Pose SLAM::CorrelativeScanMatching(const vector<float>& ranges, float angle_min, float angle_max)
{
  Pose new_pose;
  float best_likelihood = -100000;

  float angle_diff = (angle_max - angle_min) / ranges.size();
  float current_angle;

  for( unsigned int i=0; i<poses.size(); i++ )
  {
    float obs_log_likelihood = 0.0;
    current_angle = angle_min;

    for( unsigned int j=0; j<ranges.size(); j++ )
    {
      Eigen::Vector2f current_point_in_new_base_link;
      current_point_in_new_base_link.x() = ranges[j] * cos(current_angle) + 0.2;
      current_point_in_new_base_link.y() = ranges[j] * sin(current_angle);

      Eigen::Vector2f query_location = convert_scan_prev_pose( poses[i], current_point_in_new_base_link );

      obs_log_likelihood += obs_prob_table[ int((query_location.x() - min_x_val) / delta_distance) ][ int( (query_location.y() - min_y_val) / delta_distance
 ) ];

      current_angle += angle_diff;
    }

    float cur_particle_likelihood = obs_weight * obs_log_likelihood + motion_weight * poses[i].log_likelihood;
    if(cur_particle_likelihood > best_likelihood)
    {
      best_likelihood = cur_particle_likelihood;
      new_pose = poses[i];
    }
    // Need to add code which computes log likelihood of this pose with max and updates accordingly

  }

  return new_pose;
}


Eigen::Vector2f SLAM::convert_scan_prev_pose(Pose particle_pose, Eigen::Vector2f laser_point)
{
    Eigen::Rotation2Df rotation_matrix_new_link_old_link( AngleDiff( particle_pose.angle, current_best_pose.angle ) );
    Eigen::Rotation2Df rotation_matrix_map_old_link( -current_best_pose.angle );
    return rotation_matrix_map_old_link * (particle_pose.loc - current_best_pose.loc) + rotation_matrix_new_link_old_link * laser_point;
}


void SLAM::construct_obs_prob_table()
{
  obs_prob_table_width = (max_x_val - min_x_val) / delta_distance;
  obs_prob_table_height = (max_y_val - min_y_val) / delta_distance;

  obs_prob_table.resize(obs_prob_table_width);
  for(int i=0; i<obs_prob_table_width; i++)
  {
    obs_prob_table[i].resize( obs_prob_table_height );
    for(int j=0; j<obs_prob_table_height; j++) obs_prob_table[i][j] = -100000;
  }
}


void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max)
{
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.

  if( !odom_observed ) return;

  current_best_pose = CorrelativeScanMatching( ranges, angle_min, angle_max );

  // Change point cloud according to current_best_pose
  add_new_points_in_map(current_best_pose, ranges, angle_min, angle_max );

  construct_obs_prob_table();

  float angle_diff = (angle_max - angle_min) / ranges.size();
  float current_angle = angle_min;

  for(unsigned int i=0; i<ranges.size(); i++)
  {
    Eigen::Vector2f current_point;
    current_point.x() = ranges[i] * cos(current_angle) + 0.2;
    current_point.y() = ranges[i] * sin(current_angle);
    makeProbTable(current_point);
    current_angle += angle_diff;
  }
}


void SLAM::makeProbTable(Eigen::Vector2f point)
{
  int ind_x = (point.x() - min_x_val) / delta_distance;
  int ind_y = (point.y() - min_y_val) / delta_distance;

  int iter_x_min = min(ind_x - 20, 0);
  int iter_y_min = min(ind_y - 20, 0);
  int iter_x_max = max(ind_x + 20, obs_prob_table_width - 1);
  int iter_y_max = max(ind_y + 20, obs_prob_table_height - 1);

  for(int x_iter = iter_x_min; x_iter <= iter_x_max; x_iter++)
  {
    for(int y_iter = iter_y_min; y_iter <= iter_y_max; y_iter++)
    {
      float cur_ll = - ( pow( (ind_x - x_iter)*delta_distance , 2) + pow(  (ind_y - y_iter)*delta_distance , 2 ) );
      obs_prob_table[ x_iter][ y_iter ] = max(obs_prob_table[ x_iter][ y_iter ], cur_ll);
    }
  }
}



Vector2f SLAM::rotation( Eigen::Vector2f local_frame_loc, float local_frame_angle, Eigen::Vector2f point_in_local_frame )
{
  Eigen::Vector2f new_location(0.0, 0.0);

  new_location.x() = local_frame_loc.x() + point_in_local_frame.x()*cos(local_frame_angle) + point_in_local_frame.y() * sin( -local_frame_angle ) ;
  new_location.y() = local_frame_loc.y() + point_in_local_frame.x()*sin(local_frame_angle) + point_in_local_frame.y() * cos( local_frame_angle ) ;

  return new_location;
}



void SLAM::add_new_points_in_map(Pose current_best_pose, const vector<float>& ranges, float angle_min, float angle_max)
{
  float angle_diff = (angle_max - angle_min) / ranges.size();
  float current_angle = angle_min + current_best_pose.angle;
  int i=0;
  while( angle_max > current_angle )
  {
    Eigen::Vector2f range_point(ranges[i], 0.0);
    Eigen::Vector2f new_point = rotation( current_best_pose.loc, current_angle, range_point );
    constructed_map.push_back(new_point);
    current_angle += angle_diff;
    i++;
  }
  return;
}

vector<Vector2f> SLAM::GetMap()
{
  vector<Vector2f> plotting_map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  for(int i=0; i<num_points_in_final_plot; i++)
  {
      plotting_map.push_back( constructed_map[ int( (i * (constructed_map.size() - 1) ) / (num_points_in_final_plot - 1) ) ] );
  }
  return plotting_map;
}


void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle)
{
  odom_observed = true;
  if (!odom_initialized_)
  {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    return;
  }
  // Keep track of odometry to estimate how far the robot has moved between
  // poses.



  // Set odom_observed true when we call motion model here
}


}  // namespace slam
