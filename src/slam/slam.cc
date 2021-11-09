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
    odom_initialized_(false),
    x_resolution(3),
    y_resolution(3),
    theta_resolution(30) {}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = current_pose.loc;
  *angle = current_pose.angle;
}


Pose SLAM::CorrelativeScanMatching(const vector<float>& ranges, float angle_min, float angle_max)
{
  Pose new_pose;
  float best_likelihood = -100000;

  float angle_diff = (angle_max - angle_min) / ranges.size();
  float cur_angle;
   // std::cout << "checkpoint 1 " << poses.size() << std::endl;
  for( unsigned int i=0; i<poses.size(); i++ )
  {
    float obs_log_likelihood = 0.0;
    cur_angle = angle_min;

    unsigned int j = 0;
    // std::cout << j << " " << ranges.size() << std::endl;
    // std::cout << "checkpoint 1 inside first " << i << " " << ranges.size() << std::endl;
    while(j < ranges.size())
    {
      // std::cout << j << " start ";
      if(ranges[j] > 9)
      {
        cur_angle += angle_diff*skip_scans;
        j += 1*skip_scans;
        continue;
      }
      Eigen::Vector2f current_point_in_new_base_link;
      current_point_in_new_base_link.x() = ranges[j] * cos(cur_angle) + 0.2;
      current_point_in_new_base_link.y() = ranges[j] * sin(cur_angle);

      Eigen::Vector2f query_location = convert_scan_prev_pose( poses[i], current_point_in_new_base_link );


      if(obs_prob_table_init)
      {
        obs_log_likelihood += obs_prob_table[ int((query_location.x() - min_x_val) / delta_distance) ][ int( (query_location.y() - min_y_val) / delta_distance
      ) ];

      }

      cur_angle += angle_diff*skip_scans;
      j += 1*skip_scans;
      // std::cout << j << std::endl;
    }

    // std::cout << "checkpoint 1 inside " << i << std::endl;

    float cur_particle_likelihood = obs_weight * obs_log_likelihood + motion_weight * poses[i].log_likelihood;
    // float cur_particle_likelihood = motion_weight * poses[i].log_likelihood;
    if(cur_particle_likelihood > best_likelihood)
    {
      best_likelihood = cur_particle_likelihood;
      new_pose = poses[i];
    }
    // Need to add code which computes log likelihood of this pose with max and updates accordingly

  }
  // std::cout << "checkpoint 2" << std::endl;
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

  // std::cout << "ObserveLaser Start" << std::endl;

  if( !odom_observed ) return;
  if(calculate_likelihoods && use_laser)
  {

  current_best_pose = CorrelativeScanMatching( ranges, angle_min, angle_max );

  // Change point cloud according to current_best_pose
  add_new_points_in_map(current_best_pose, ranges, angle_min, angle_max );

  construct_obs_prob_table();
  float angle_diff = (angle_max - angle_min) / ranges.size();
  float cur_angle = angle_min;



  for(unsigned int i=0; i<ranges.size(); i++)
  {
    Eigen::Vector2f current_point;
    current_point.x() = ranges[i] * cos(cur_angle) + 0.2;
    current_point.y() = ranges[i] * sin(cur_angle);
    // std::cout << "checkpoint in " << i << " " << obs_prob_table_width << " " << obs_prob_table_height << " " << current_point.x() << " " << ranges.size() << std::endl;
    makeProbTable(current_point);
    cur_angle += angle_diff;
  }
  obs_prob_table_init = true;
  use_laser = false;
  rotation_matrix = Eigen::Rotation2Df(current_best_pose.angle - prev_odom_angle_);
  }
  // std::cout << "end of ObserveLaser" << std::endl;


}


void SLAM::makeProbTable(Eigen::Vector2f point)
{
  int ind_x = (point.x() - min_x_val) / delta_distance;
  int ind_y = (point.y() - min_y_val) / delta_distance;

  int iter_x_min = max(ind_x - 10, 0);
  int iter_y_min = max(ind_y - 10, 0);
  int iter_x_max = min(ind_x + 10, obs_prob_table_width - 1);
  int iter_y_max = min(ind_y + 10, obs_prob_table_height - 1);

  // std::cout << iter_x_min << " " << iter_x_max << " " << iter_y_min << " " << iter_y_max << endl;

  for(int x_iter = iter_x_min; x_iter <= iter_x_max; x_iter++)
  {
    for(int y_iter = iter_y_min; y_iter <= iter_y_max; y_iter++)
    {
      float cur_ll = - ( pow( (ind_x - x_iter)*delta_distance , 2) + pow(  (ind_y - y_iter)*delta_distance , 2 ) ) / obs_variance;
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
  float cur_angle = angle_min + current_best_pose.angle;
  int i=0;
  while( angle_max > cur_angle )
  {
    Eigen::Vector2f range_point(ranges[i], 0.0);
    Eigen::Vector2f new_point = rotation( current_best_pose.loc, cur_angle, range_point );
    constructed_map.push_back(new_point);
    cur_angle += angle_diff;
    i++;
  }
  return;
}


vector<Vector2f> SLAM::GetMap()
{
  std::cout << "GetMap Start" << std::endl;
  vector<Vector2f> plotting_map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  for(int i=0; i<num_points_in_final_plot; i++)
  {
      if(constructed_map.size() > 0){
        plotting_map.push_back( constructed_map[ int( (i * (constructed_map.size() - 1) ) / (num_points_in_final_plot - 1) ) ] );
      }
  }
  std::cout << "GetMap End" << std::endl;
  return plotting_map;
}




void SLAM::motion_model(float distance, float angle, float x_translation_error_stddev,float y_translation_error_stddev,float rotation_error_stddev){

  //returns table of log likelihoods from motion model

  //k1 : translation error from translation
  //k2 : rotation error from translation
  //k3 : rotation error from rotation
  //k4 : translation error from rotation
  // angle : angle diff from current and previous pose
  // distance : magnitude of transform from current and previous pose

  //double x_translation_error_stdev= (k1)*magnitude_of_transform+ (k2)*magnitude_of_rotation;
  //double y_translation_error_stdev= k1*magnitude_of_transform+ k2*magnitude_of_rotation;
  //double rotation_error_stdev= k3*magnitude_of_transform+ k4*magnitude_of_rotation;


  // iterate through every cell in 3d motion model lookup table

  for(int i=0;i<x_resolution;i++){
    for(int j=0; j<y_resolution;j++){
      for(int k=0; k<theta_resolution; k++){



        float x_delta=x_translation_error_stddev*(2*i/(x_resolution-1)-1);
        float y_delta = y_translation_error_stddev*(2*j/(y_resolution-1)-1);
        float angle_delta= rotation_error_stddev*(2*k/(theta_resolution-1)-1);

        float x_likelihood = -(pow(x_delta/x_translation_error_stddev, 2));
        float y_likelihood = -(pow(y_delta/y_translation_error_stddev, 2));
        float angle_likelihood = -(pow(angle_delta/rotation_error_stddev, 2));
        //Eigen::Vector2f distance_noise(x_translation_error_stdev,y_translation_error_stdev);
        Eigen::Vector2f angle_rotation(cos(angle),sin(angle));
        //whatever current location variable is assumed its called current_loc and current_angle
        //pose(0): x_loc pose(1) , y_loc , pose(2) theta , log likelihood



        float log_likelihood= x_likelihood+y_likelihood+angle_likelihood;
        Eigen::Vector4d temp( current_pose.loc.x() + x_delta*cos(current_pose.angle) - y_delta*sin(current_pose.angle), current_pose.loc.y() + x_delta*sin(current_pose.angle) + y_delta*cos(current_pose.angle), current_pose.angle+angle_delta,log_likelihood);
        Pose new_pose;
        new_pose.log_likelihood=temp(3);
        new_pose.loc= Eigen::Vector2f(temp(0),temp(1));
        new_pose.angle= temp(2);
        //all poses for current time step
        poses.push_back(new_pose);

        // std::cout << i << " " << j << " " << k << " " << new_pose.loc << " " << current_loc << std::endl;

      }
    }
  }

}


void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  odom_observed = true;

  std::cout << "ObserveOdometry Start" << std::endl;

  if (!odom_initialized_)
  {
    current_angle = odom_angle;
    current_loc = odom_loc;
    odom_initialized_ = true;
    last_likelihood_scan_loc=odom_loc;
    last_likelihood_scan_angle=odom_angle;
    prev_odom_angle_ = current_angle;
    prev_odom_loc_ = current_loc;
    use_laser = true;
    return;
  }
  // Keep track of odometry to estimate how far the robot has moved between
  // poses.
  // prev_odom_angle_ = current_angle;
  //   prev_odom_loc_ = current_loc;
    current_angle=odom_angle;
    current_loc= odom_loc;

    // Not sure about these two lines
    // current_best_pose.loc = current_best_pose.loc + Eigen::Rotation2Df( odom_angle - prev_odom_angle_ ) * (odom_loc - prev_odom_loc_ );
    // current_best_pose.angle = AngleDiff(current_best_pose.angle, AngleDiff(odom_angle, prev_odom_angle_));

    current_pose.loc = current_best_pose.loc + rotation_matrix * (current_loc - prev_odom_loc_);
    current_pose.angle = fmod(current_best_pose.angle + AngleDiff(odom_angle, prev_odom_angle_) + M_PI,  2*M_PI) - M_PI;



    float distance_to_compute_scan=0.1;
    float angle_to_compute_scan=M_PI/24;

    if(((prev_odom_loc_-current_loc).norm()>=distance_to_compute_scan) || (abs(AngleDiff(prev_odom_angle_, current_angle))>angle_to_compute_scan)){
      calculate_likelihoods=true;
      last_likelihood_scan_loc=current_loc;
      last_likelihood_scan_angle=current_angle;
      }
    else{
        calculate_likelihoods=false;
      }


    if(calculate_likelihoods)
    {


    double distance = (current_loc-prev_odom_loc_).norm();
    float angle = AngleDiff(current_angle,prev_odom_angle_);
    double k1 = 0.8;
    double k2 = 0.5;
    double k3 = 0.1;
    double k4 = 2.0;
    double magnitude_of_rotation = abs(angle);
    double x_translation_error_stdev= k1*distance+ k2*magnitude_of_rotation;
    double y_translation_error_stdev= k1*distance+ k2*magnitude_of_rotation;
    double rotation_error_stdev= k3*distance+ k4*magnitude_of_rotation;
    poses.clear();
    motion_model(distance,angle,x_translation_error_stdev,y_translation_error_stdev,rotation_error_stdev);

    use_laser = true;


        prev_odom_loc_ = odom_loc;
        prev_odom_angle_ = odom_angle;
    }
    std::cout << "ObserveOdometry End" << std::endl;

}




}  // namespace slam
