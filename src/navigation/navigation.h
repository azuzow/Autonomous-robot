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
#include "eigen3/Eigen/Geometry"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "simple_queue.h"
#include "vector_map/vector_map.h"
#include "amrl_msgs/VisualizationMsg.h"


#ifndef NAVIGATION_H
#define NAVIGATION_H

/*      */
#define X_MAX 1000
#define X_STEP 20
#define Y_MAX 500
#define Y_STEP 20
/*      */

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

struct Edge {
  pair<int,int> id;
  float weight;
  Eigen::Vector2i neighbor_ind;
  int neighbor_num;
};


struct Node {
  Eigen::Vector2f loc;
  pair<int,int> parent_id; 
  pair<int,int> id;
  Eigen::Vector2i index;
  float g;
  float h;
  float f;
  std::vector<Edge> neighbors;
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

  float checkPoint(float angle, float curvature, float x, float y);

  Eigen::Vector2f latency_compensation(const float& latency, unsigned int iterations);

  bool checkPointinSector(float x, float y, float percent, float radius );

  PathOption find_optimal_path(unsigned int total_curves, float min_curve,const Eigen::Vector2f target_point);

  
/******************************************************************************/
/************************Public : Global Planning********************************/
double calculateHeuristic(Eigen::Vector2f node, Eigen::Vector2f target);

void aStarPathFinder(Eigen::Vector2f global_target_loc);

Edge NeighborSetup(Eigen::Vector2i loc_index, int neighbor_number);

std::vector<Edge> findEightNeighbors(Node node);

std::vector<geometry::line2f> findMargins(geometry::line2f line_edge);

bool isValid(Eigen::Vector2f node_loc, Eigen::Vector2i node_index, Eigen::Vector2i neighbor_ind);

std::vector<Node> construct_path(Node destination);

Node findTheCarrot(Eigen::Vector2f current_loc);

void initialization(Eigen::Vector2f starting_loc);

Node nodeSetup(Node node, int neighbor_num);

Eigen::Vector2i neighborhoodLookup(int index);

void recalculate_path(Eigen::Vector2f destination_loc);

void drawNavigationPath(amrl_msgs::VisualizationMsg &msg);

void drawOpenList(amrl_msgs::VisualizationMsg &msg);






  /******************************************************************************/
  //min = -pi/2
  float min_curve = -M_PI_2;
  //max = pi/2
  float max_curve = M_PI_2;

  float car_length = 0.45;
  float car_width = 0.27;
  float car_base_length = 0.32;
  float margin = 0.12;


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

  /******************************************************/
  /*************Private :Global Planning*****************/
    float D1;
    float D2;
    Eigen::Vector2f goal; //global navigation target
    std::vector<Node> path_navigation; //navigation path to destination
    uint64_t idCounter = 0;
    std::map<pair<int, int>, Node> node_map;
    float resolution = 0.25;
    float diagonal_movement = 1.414 * resolution;
    float straight_movement = 1 * resolution;
    float minimum_radius = 2.0;
    float buffer = 0.25;
    bool NEED_TO_RECALCULATE_PATH = false;
    bool DESTINATION_REACHED = false;
    Eigen::Vector2f destinationLoc;
    Eigen::Matrix2f rotateMaptoBase;
    // Map of the environment.
    vector_map::VectorMap map_;
    bool found_path;  
  /*****************************************************/

};

}  // namespace navigation

#endif  // NAVIGATION_H