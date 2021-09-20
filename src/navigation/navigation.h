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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================




/*
Acutal width and length of the car:
width: 21 cm
length: 47 cm

*/


#include <vector>
#include <iostream>
#include<deque>
#include <algorithm>

#include "eigen3/Eigen/Dense"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  float score;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

  // Update speed by using current velocity
  void updateSpeed(PathOption optimal_path);

  void DrawCar();

  // Draw car
  // void calculate_distance_to_target();

  Eigen::Vector2f get_robot_loc();

  float get_robot_angle();

  void updatePointCloudToGlobalFrame();

  float findDistanceofPointfromCurve(float x, float y, float curvature);

  //returns distance of nearest point
  float findNearestPoint(float curvature, float angle);

  //return vector of nearest point
  Eigen::Vector2f  findVectorOfNearestPoint(float curvature, float angle);

  std::pair<float, float> distanceAlongPath(float x, float y, float curvature);

  void computeAllPaths();

  void scoreFunction();

  bool isClockwise(float x, float y);

  // returns all possible paths the car might take
  void initialize_trajectories(const float curvature, const float distance, const float clearance);

  float check_if_collision(float curvature, Eigen::Vector2f& target_point, float inner_radius, float mid_radius, float outer_radius);

  std::pair<float, float> free_path_length_function(float curvature);
  // std::pair<float, float> findFreePathLengthAlongACurvature(float curvature);

  float findBestCurvature(unsigned int& total_curves, float& min_curve);

  bool checkPoint(float angle, float curvature, float x, float y);

  Eigen::Vector2f latency_compensation(const float& latency, unsigned int iterations);

  bool checkPointinSector(float x, float y, float percent, float radius );

  PathOption find_optimal_path(unsigned int total_curves, float min_curve,const Eigen::Vector2f target_point);

  //min = -pi/2
  float min_curve = -M_PI_2;
  //max = pi/2
  float max_curve = M_PI_2;

  float car_length = 0.47;
  float car_width = 0.21;
  float car_base_length = 0.35;
  float margin = 0.2;


 private:

  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;
  bool point_cloud_set;
  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;
  // speed calculated from velocity
  float speed;
  //cruise speed
  float max_speed;
  float max_acceleration_magnitude;
  float max_deceleration_magnitude;
  // previous velocity
  float previous_velocity;
  std::deque<float> previous_speeds;
  std::deque<Eigen::Vector2f> previous_velocities;
  std::deque<Eigen::Vector2f> previous_locations;
  std::deque<float> previous_omegas;
  std::deque<float> previous_angles;
  PathOption best_path;
};

}  // namespace navigation

#endif  // NAVIGATION_H
