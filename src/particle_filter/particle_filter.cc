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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
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
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using math_util::AngleDiff;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 30, "Number of particles");

namespace particle_filter {

  config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

  ParticleFilter::ParticleFilter() :
  prev_odom_loc_(0, 0),
  prev_odom_angle_(0),
  odom_initialized_(false) {}

  void ParticleFilter::GetParticles(vector<Particle>* particles) const {
    *particles = particles_;
  }


  void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
    const float angle,
    int num_ranges,
    float range_min,
    float range_max,
    float angle_min,
    float angle_max,
    vector<Vector2f>* scan_ptr) {


    vector<Vector2f>& scan = *scan_ptr;

  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
    scan.resize(num_ranges);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...

  // .norm() of vector is magnitude
    // float magnitude = loc.norm();
    Eigen::Vector2f lazer_offset(cos(angle),sin(angle));
    lazer_offset=lazer_offset*0.2;
    Eigen::Vector2f lazer_loc = loc+lazer_offset;
    float angle_range = angle_max-angle_min;
    float angle_increment= angle_range/float(num_ranges);
    float current_ray_angle = angle + angle_min;

    // TODO: Global frame vs Local frame

    for (size_t i = 0; i < scan.size(); ++i)
    {

      Eigen::Vector2f ray_start(cos(current_ray_angle),sin(current_ray_angle));
      Eigen::Vector2f ray_end(cos(current_ray_angle),sin(current_ray_angle));

      //TODO: We need to add lidar location here
      ray_start= ray_start*range_min + lazer_loc;
      ray_end= ray_end*range_max + lazer_loc;
      line2f current_ray(ray_start.x(), ray_start.y(), ray_end.x(), ray_end.y());

      // The line segments in the map are stored in the `map_.lines` variable. You
      // can iterate through them as:

      Eigen::Vector2f closest_point = ray_end;
      Eigen::Vector2f intersection_point(0,0);

      for (size_t j = 0; j < map_.lines.size(); ++j)
      {
        const line2f map_line = map_.lines[j];
        bool intersects = map_line.Intersection(current_ray, &intersection_point);
        if (intersects)
        {
          // printf("Intersection\n" );
          //TODO: Shouldn't we need to find distance with location and not lazer location.
          float distance = sqrt(pow(intersection_point.x()-lazer_loc.x(),2)+pow(intersection_point.y()-lazer_loc.y(),2));
          float closest_point_distance = sqrt(pow(intersection_point.x()-closest_point.x(),2)+pow(intersection_point.y()-closest_point.y(),2));
          if(distance<closest_point_distance)
          {
            closest_point=intersection_point;
          }
        }
        else
        {
          // printf("No intersection\n");
        }

      }
      scan[i]=closest_point;
   // scan[i] = Vector2f(0, 0);
      current_ray_angle+=angle_increment;
    }
  }


  Vector2f ParticleFilter::convertToGlobalFrame(Vector2f local_frame_loc, float local_frame_angle, Vector2f point_in_local_frame)
  {
    Eigen::Vector2f lazer_offset(cos(local_frame_angle), sin(local_frame_angle));
    Eigen::Vector2f global_location(0.0, 0.0);

    global_location.x() = local_frame_loc.x() + point_in_local_frame.x()*cos(local_frame_angle);
    global_location.y() = local_frame_loc.y() + point_in_local_frame.x()*sin(local_frame_angle);

    return global_location;
  }

  void ParticleFilter::Update(const vector<float>& ranges,
    float range_min,
    float range_max,
    float angle_min,
    float angle_max,
    Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.

    // void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
    // const float angle,
    // int num_ranges,
    // float range_min,
    // float range_max,
    // float angle_min,
    // float angle_max,
    // vector<Vector2f>* scan_ptr)


    Particle& particle = *p_ptr;
    // std::cout<<particle.loc.x()<<" "<< particle.loc.y()<<std::endl;
    std::vector<Eigen::Vector2f> predicted_pointCloud;
    GetPredictedPointCloud(particle.loc,particle.angle,ranges.size(),range_min,range_max,angle_min,angle_max,&predicted_pointCloud);

    unsigned int i = 0;
    double distance = 0;

    Eigen::Vector2f lazer_loc_local_frame(0.2, 0.0);

    Eigen::Vector2f lazer_loc = convertToGlobalFrame( particle.loc , particle.angle, lazer_loc_local_frame );
    double log_prob = 0;
    double prob = 0;


    for (i=0; i < predicted_pointCloud.size(); i++)
    {
      distance = sqrt( pow(predicted_pointCloud[i].x()-lazer_loc.x(),2) + pow(predicted_pointCloud[i].y()-lazer_loc.y(),2) );
      if(ranges[i] < range_min)
      {
        continue;
      }
      else if(ranges[i] > range_max)
      {
        continue;
      }
      else if(ranges[i] < distance - short_distance )
      {
        prob = exp( - ( short_distance*short_distance ) / ( std_update_weight * std_update_weight ) );
      }
      else if( ranges[i] > distance + long_distance )
      {
        prob = exp( - ( long_distance*long_distance ) / ( std_update_weight * std_update_weight ) );
      }
      else
      {
        prob = exp( - ( pow( distance - ranges[i] , 2) ) / (std_update_weight * std_update_weight) );
      }
      log_prob += log(prob);
    }

    particle.log_weight += log_prob;
    // particle.weight += exp(log_prob);
  }


void ParticleFilter::Resample()
{
    // Resample the particles, proportional to their weights.
    // The current particles are in the `particles_` variable.
    // Create a variable to store the new particles, and when done, replace the
    // old set of particles:
    // vector<Particle> new_particles';
    // During resampling:
    //    new_particles.push_back(...)
    // After resampling:
    // particles_ = new_particles;

    // You will need to use the uniform random number generator provided. For
    // example, to generate a random number between 0 and 1:

    //compute sum of all particle weights
    float totalWeightSum = 0;
    double max_log_prob = -10000000.0;

    for(unsigned int i=0; i<particles_.size(); i++)
    {
      max_log_prob = std::max(max_log_prob, particles_[i].log_weight);
    }

    // std::cout << "after finding max_log_prob" << particles_.size() << std::endl;

    for(unsigned int i=0; i<particles_.size(); i++)
    {
      particles_[i].log_weight -= max_log_prob;
      particles_[i].weight = exp(particles_[i].log_weight);
      // std::cout << i << " " << particles_[i].weight << std::endl;
      totalWeightSum += particles_[i].weight;
    }

    // std::cout << "after finding total prob" << particles_.size() << std::endl;


    // for(auto& particle: particles_)
    // {
    //   totalWeightSum += particle.weight;
    // }


    // use vectors to encode the width of each bin.
    // Width is equal to the weight of particle[i] proportional to the total sum of particle weights
    unsigned int total_particles=FLAGS_num_particles;
    float weightSum = 0;
    std::vector<Vector2f> binSet;
    for(unsigned int i=0; i< total_particles; ++i)
    {
        float start = weightSum / totalWeightSum;
        weightSum += particles_[i].weight;
        float end = weightSum / totalWeightSum;
        // std::cout << i << " " << particles_[i].weight << " " << start << " " << end << std::endl;
        Vector2f bin = Vector2f(start,end);
        binSet.push_back(bin);
    }

    //new particle set
    vector<Particle> newParticles_;
    unsigned int j =0, i=0;
    double avg_bin_size = 1.0 / total_particles;

  //select a random value between 0 and 1 and  determine which bin it belongs to.
  //Corresponding particle at bin will be resampled in the new particle set
  // Run this step N (number of particles in set) times

    float randNum = rng_.UniformRandom(0, 1);
    std::cout << particles_.size() << std::endl;

    while( j < total_particles )
    {
      while(binSet[i].x() < randNum && randNum <= binSet[i].y() )
      {
        newParticles_.push_back(particles_[i]);
        // std::cout << i << " " << particles_[i].loc << " " << particles_[i].weight << " " << particles_[i].log_weight << std::endl;
        randNum += avg_bin_size;
        j++;
        if( j == total_particles )
        {
          break;
        }
        if (randNum > 1)
        {
          randNum -= 1;
        }
      }
      i++;
      if( i == total_particles )
      {
        i = 0;
      }
      // std::cout << i << " " << j << " " << randNum << " " << binSet[i].x() << " " << binSet[i].y() << std::endl;
    }


   /* float r = rng_.UniformRandom(0, 1/total_particles);
    double c = particles_[0];
    x = 1;
    for(unsigned int m = 0; m < total_particles ; m++){
        double U = r + (m -1) * (1/total_particles);
        while( U > c)
        {
          x += 1;
          c += particles_[x];
        }
        newParticles_.push_back(particles_[x]);
    } */



   //   printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
    //   x);

    particles_ = newParticles_;
}


void ParticleFilter::ObserveLaser(const vector<float>& ranges,
    float range_min,
    float range_max,
    float angle_min,
    float angle_max)
{
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.

  if(odom_initialized_ == false)
  {
    return;
  }

  // std::cout << odom_initialized_ << " In update " << std::endl;

  double distance_from_last_update = (prev_odom_loc_ - last_update).norm();

  if (distance_from_last_update < 0.1)
  {
    return;
  }

    unsigned int i = 0;
    std::cout << odom_initialized_ << " before update " << std::endl;
    for(i=0; i < particles_.size(); i++)
    {
      Update( ranges, range_min, range_max, angle_min, angle_max, &particles_[i] );
    }
    updateCount++;
std::cout << odom_initialized_ << " after update " << updateCount << std::endl;

    //resample less often: control how many times resample is called based on number of updates ; default is 1 (resample all the time)
    if(updateCount % modOperator == 0){
      std::cout << odom_initialized_ << " Going in Resample " << std::endl;
      Resample();
    }
    last_update = prev_odom_loc_;
    std::cout << odom_initialized_ << " after Resample " << std::endl;

  }


  void ParticleFilter::TransformParticle(Particle* particle_pointer,const Eigen::Vector2f& transform, const float& rotation,const float& k1,const float& k2,const float& k3,const float& k4){
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

  }

  void ParticleFilter::Predict(const Vector2f& odom_loc,
   const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.

    if(odom_initialized_)
    {
      if( (odom_loc-prev_odom_loc_).norm() > 1.0 )
      {
        prev_odom_loc_ = odom_loc;
        prev_odom_angle_ = odom_angle;
      }
      else
      {
        float deltaTransformAngle = AngleDiff(odom_angle, prev_odom_angle_);
         for(auto& particle: particles_)
         {

            Eigen::Rotation2Df rotation( particle.angle -prev_odom_angle_ );
            Eigen::Vector2f deltaTransformBaseLink =  rotation * (odom_loc-prev_odom_loc_) ;

            TransformParticle(&particle, deltaTransformBaseLink, deltaTransformAngle, 0.4, 0.02, 0.2, 0.4 );
          }
          prev_odom_loc_ = odom_loc;
          prev_odom_angle_ = odom_angle;
      }
    }
    else
    {
      odom_initialized_ = true;
      prev_odom_loc_ = odom_loc;
      prev_odom_angle_ = odom_angle;
      updateCount = 0;
    }

  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  // float x = rng_.Gaussian(0.0, 2.0);
  // printf("Random number drawn from Gaussian distribution with 0 mean and " "standard deviation of 2 : %f\n", x);
}

void ParticleFilter::Initialize(const string& map_file,
  const Vector2f& loc,
  const float angle)
  {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.

  unsigned int total_particles=FLAGS_num_particles;
  odom_initialized_ = false;

  std::cout << "In initialization" << std::endl;

  for(unsigned int i=0; i< total_particles; ++i){

    Particle particle;

    particle.loc.x() = loc.x()+ rng_.Gaussian(0.0, 0.05);
    particle.loc.y() = loc.y()+ rng_.Gaussian(0.0, 0.05);
      //angle within theta of 30
    particle.angle = angle+rng_.Gaussian(0.0, M_PI/32);
    particle.weight = (1.0)/FLAGS_num_particles;
    particle.log_weight = log( particle.weight );
    particles_.push_back(particle);
  }
  map_.Load(map_file);
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr,
 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:

  double max_log_prob = 0.0, totalWeightSum = 0.0, p_weights[int(FLAGS_num_particles)];

  for(unsigned int i=0; i<particles_.size(); i++)
  {
    max_log_prob = std::max(max_log_prob, particles_[i].log_weight);
  }

  for(unsigned int i=0; i<particles_.size(); i++)
  {
    p_weights[i] = exp(particles_[i].log_weight - max_log_prob);
    // p_weights[i] = particles_[i].weight;
    totalWeightSum += p_weights[i];
  }

  Eigen::Vector2f next_robot_loc(0.0, 0.0);
  double next_robot_angle = 0.0;

  for(unsigned int i=0; i<particles_.size(); i++)
  {
    next_robot_loc += ( p_weights[i] / totalWeightSum )*particles_[i].loc;
    next_robot_angle += ( p_weights[i] / totalWeightSum )*particles_[i].angle;
  }
  loc = next_robot_loc;
  angle = next_robot_angle;

  // std::cout << loc << " location and angle in get location " << angle << " " << particles_.size() << std::endl;

}


}  // namespace particle_filter
