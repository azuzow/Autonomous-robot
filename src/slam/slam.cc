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
using math_util::AngleDiff;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;
using geometry::line2f;

namespace slam {

SLAM::SLAM() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

//TODO: Get Current Pose Estimate from Odometry
void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = Vector2f(0, 0);
  *angle = 0;
}


//TODO: Needs to be revisited
vector<Vector2f>* SLAM::getPointCloud(
    int num_ranges,
    float range_min,
    float range_max,
    float angle_min,
    float angle_max) {


  static vector<Vector2f> laserScan(size_t(num_ranges), Vector2f(0,0));

    
    float angle_range = angle_max-angle_min;
    float angle_increment= angle_range/float(num_ranges);
    laserScan.resize(size_t(num_ranges));

    float current_ray_angle = angle_min;
    for (size_t i = 0; i < size_t(num_ranges); i++)
    {
        Eigen::Vector2f lazer_offset(cos(current_ray_angle),sin(current_ray_angle));
        Eigen::Vector2f lazer_loc=lazer_offset*0.2;
        laserScan[i] = lazer_loc;
        current_ray_angle += angle_increment;
    }
    return &laserScan;
  }

  
Vector2f SLAM::convertToGlobalFrame(Vector2f local_frame_loc, float local_frame_angle, Vector2f point_in_local_frame)
  {
    Eigen::Vector2f lazer_offset(cos(local_frame_angle), sin(local_frame_angle));
    Eigen::Vector2f global_location(0.0, 0.0);

    global_location.x() = local_frame_loc.x() + point_in_local_frame.x()*cos(local_frame_angle);
    global_location.y() = local_frame_loc.y() + point_in_local_frame.x()*sin(local_frame_angle);

    return global_location;
  }


Pose SLAM::correlativeScanMatching( int num_ranges,
    float range_min,
    float range_max,
    float angle_min,
    float angle_max) {
    Pose csm_Pose;

    return csm_Pose;
}



void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.

  if(odom_initialized_ == false)
  {
    return;
  }

}





/*void SLAM::TransformParticle(Particle* particle_pointer,const Eigen::Vector2f& transform, const float& rotation,const float& k1,const float& k2,const float& k3,const float& k4){
  //k1 : translation error from translation
  //k2 : rotation error from translation
  //k3 : rotation error from rotation
  //k4 : translation error from rotation

  //k1 and k2 should be larger than k3 and k4 to create a oval that is longer along the x axis

  Particle& particle = *particle_pointer;

  // std::cout << "Transform:" << transform << " rotation:" << rotation << std::endl;

    float magnitude_of_transform = sqrt((transform.x()*transform.x())+(transform.y()*transform.y()) );
    float magnitude_of_rotation = abs(rotation);
    //adding some constant to k1 and k2 to make the the probability density contour more oval like
    float x_translation_error_stdev= (k1)*magnitude_of_transform+ (k2)*magnitude_of_rotation;
    float y_translation_error_stdev= k1*magnitude_of_transform+ k2*magnitude_of_rotation;
    float rotation_error_stdev= k3*magnitude_of_transform+ k4*magnitude_of_rotation;

    // std::cout<<magnitude_of_transform<<magnitude_of_rotation<<x_translation_error_stdev<<y_translation_error_stdev<<rotation_error_stdev<<std::endl;
    float epsilon_x= rng_.Gaussian(0.0, x_translation_error_stdev);
    float epsilon_y= rng_.Gaussian(0.0, y_translation_error_stdev);
    float epsilon_theta=rng_.Gaussian(0.0, rotation_error_stdev);

    particle.loc.x() += transform.x()+ epsilon_x;
    particle.loc.y() += transform.y()+ epsilon_y;
    // not sure if this is correct either
    particle.angle+= rotation+epsilon_theta;

    // std::cout << particle.loc.x() << " in TransformParticle " << particle.loc.y() << std::endl;

  } */



void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    update_flag = true;
    return;
  }


}



void SLAM::buildLookUpTable(Vector2f point){



 int x = point.x();
 int y = point.y();
 Eigen::Vector2f lazer_loc_local_frame(0.2, 0.0);
 Eigen::Vector2f lazer_loc;
 
 vector<float> ranges;
/*****Parameters need tuning *********/
 //float xMax = 0;
 //float yMax = 0;
 //float xMin = 0;
 //float yMin = 0;
 /************************************/
 //float tableRangeX = xMax - xMin;
 //float tableRangeY = yMax - yMin;
 //float tableSizeX = float(tableRangeX/ TABLE_WIDTH );
 //float tableSizeY = float(tableRangeY/ TABLE_HEIGHT );

 for(int i = 0; i < TABLE_WIDTH ; i++){
  for(int j = 0; j < TABLE_HEIGHT; j++){
    float diff = pow(i * TABLE_WIDTH,2) + pow(j * TABLE_HEIGHT, 2);
    float likelihood = exp(-(diff / (var_obs_ * var_obs_)));
    if((x+i) < TABLE_WIDTH && (y+ j) < TABLE_HEIGHT){
      if( (x + i >= 0) && (y + i >= 0)){
      lookupTable[x+i][y+i] = std::max(likelihood, lookupTable[x+i][y+j]);
      }
    }
    if((x-i) < TABLE_WIDTH && (y - j) < TABLE_HEIGHT){
      if( (x - i >= 0) && (y - i >= 0)){
      lookupTable[x-i][y - i] = std::max(likelihood, lookupTable[x-i][y-j]);
      }
    }
  }
 }


}



vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.

  return map;
}

}  // namespace slam