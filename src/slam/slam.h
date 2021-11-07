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
#include "vector_map/vector_map.h"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

#define TABLE_WIDTH 30
#define TABLE_HEIGHT 30

namespace slam {

struct Pose {
  Eigen::Vector2f loc;
  float angle;
};

struct Pose_Likelihood{
  Pose pose;
  float log_likelihood;
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

  std::vector<Eigen::Vector2f>* getPointCloud(int num_ranges, float range_min, float range_max, float angle_min, float angle_max);


  Eigen::Vector2f convertToGlobalFrame(Eigen::Vector2f local_frame_loc, float local_frame_angle, Eigen::Vector2f point_in_local_frame);

  void pointCloudTrim(std::vector<Eigen::Vector2f>* scan, int offset);

  void buildLookUpTable(Eigen::Vector2f loc);
  Pose correlativeScanMatching( int num_ranges, float range_min, float range_max, float angle_min, float angle_max);

  Eigen::Vector2f transformLatestScan(const Eigen::Vector2f scan_loc, Pose pose_cur);
  void MotionModel(Eigen::Vector2f loc, float angle, float dist_traveled, float angle_diff);
  void reconstructMap(const std::vector<float>& ranges, float angle_min, float angle_max, Pose pose);
  // LookupTable containing log probabilities for lidar observations at each pose
  std::vector< std::vector<float> > lookupTable; 
  void runCSM(const std::vector<float>& ranges, float range_min, float range_max, float angle_min, float angle_max);


 private:

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;
  bool update_flag = false;
  float angle_offset = M_PI /24;
  float var_obs_ = 0.5;
  float short_distance = 0.5;
  float long_distance = 0.5;
  float gamma = 1.0;
  int ratio = 10;
  float lin_thresh = 0.15;
  // Map of the environment.
  std::vector<Eigen::Vector2f> map_;
  Pose m_pose;
  Eigen::Rotation2Df m_Rot_Odom;
  Pose curr_pose;
  std::vector<Pose_Likelihood> poses_;
  bool lookup_initialized = false;  
};
}  // namespace slam

#endif   // SRC_SLAM_H_