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
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
//  Version 3 in the file COPYING that came with this distribution.
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include <algorithm>
#include <cmath>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include <limits>


#define MAX_NEIGHBORS 9



using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using geometry::line2f;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;

// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {


Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    speed(0),
    max_speed(1),
    max_acceleration_magnitude(4),
    max_deceleration_magnitude(4),
    D1(1),
    D2(1.414),
    found_path(true),
    found_target(false){
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
  map_.Load(map_file);
  if(map_file.empty()){
    std::cout << "No Map" << std::endl;
  }
  else{
    std::cout << "Map Loaded:" << map_file << std::endl;
  }

}


void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  std::cout << "Enter SetNavGoal()" << std::endl;
  destinationLoc = loc;
  //start position
  //Eigen::Vector2f starting_location = Vector2f(-26.3, 8.3); 
  //destinationLoc = Vector2f(-13.44, 13.59);
  std::cout << "Starting Location : " <<  robot_loc_ << std::endl;
  initialization(robot_loc_);
  // find path
  std::cout << "Destination Loc : " <<  destinationLoc << std::endl;
  aStarPathFinder(destinationLoc);
  found_path = false;
  found_target = false;
  //initMap();
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;

  rotateMaptoBase << cos(robot_angle_), -sin(robot_angle_), sin(robot_angle_), cos(robot_angle_);
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

Eigen::Vector2f Navigation::get_robot_loc()
{
  //std::cout << robot_loc_ << std::endl;
  return robot_loc_;
}

float Navigation::get_robot_angle()
{
  return robot_angle_;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud, double time) {
  point_cloud_ = cloud;
}

// void Navigation::calculate_distance_to_target(){
//   float min_distance = -1000000;
//   if (point_cloud_.size() == 0) {
//     // return -1;
// }
//   for (unsigned int i=0; i < point_cloud_.size(); i++)
//   {
//     float distance = sqrt( pow(point_cloud_[i][0], 2) + pow(point_cloud_[i][1], 2) );

//     if ( abs(point_cloud_[i][1]) < car_length + margin ) // Checking if i-th point is in straight line or not.
//     {
//       // std::cout << "Information about minimum point: Distance " << distance << " index: " << i << std::endl;
//       min_distance = distance;
//     }
//   }
//   // return min_distance;
// }


void Navigation::updateSpeed(PathOption optimal_path){
  //float x=robot_vel_.x();
  //float y=robot_vel_.y();
  // speed= sqrt(x*x + y*y);
  speed = drive_msg_.velocity;
  float distance = optimal_path.free_path_length;
  float distance2goal = (robot_loc_ - destinationLoc).norm();
  /*std::cout<< "=================================================" << std::endl;
  std::cout << " Distance2Goal = " << dist_to_goal << std::endl;
  std::cout<< "=================================================" << std::endl;*/
  // time_needed_to_stop= (speed*speed)/max_deceleration_magnitude;
  //std::cout<<"==================="<<std::endl;
  float distance_needed_to_stop= (max_speed*max_speed)/(2*max_deceleration_magnitude);
  /*std::cout<< "=================================================" << std::endl;
  std::cout << " Distance Needed to Stop = " << distance_needed_to_stop << std::endl;
  std::cout<< "=================================================" << std::endl;*/
  //std::cout<<"distance needed to stop "<<distance_needed_to_stop<<std::endl;
  //std::cout<<"==================="<<std::endl;
  // distance_needed_to_cruise=(speed*speed)/(2*max_acceleration_magnitude);
  //std::cout<<"distance remaining "<<distance<<std::endl;
  //std::cout<<speed<<" "<<max_speed<<" "<<distance<< " " << robot_omega_ << " " << x << " " << y << std::endl;
  if( (speed<=0)  && (distance_needed_to_stop>=distance*.9) ){
    // std::cout<<"stopped"<<std::endl;
    drive_msg_.velocity=0;
    exit(0);
  }
  else if (distance_needed_to_stop>=0.9*distance)
  {
    // decelerate
    // std::cout<<speed<<"decelerating"<<std::endl;
    drive_msg_.velocity=speed-(max_deceleration_magnitude*1/20);
    // exit(0);
  }
  else if(speed<max_speed && distance>0  ){

    // std::cout<<"accelerating"<<std::endl;
    drive_msg_.velocity=speed+max_acceleration_magnitude;
    if (drive_msg_.velocity > max_speed)
    {
      drive_msg_.velocity = speed+ (max_acceleration_magnitude*1/20);
    }
  }
  else if (distance2goal <= (10*distance_needed_to_stop)){
    found_target = true;
  }

  else
  {
  // otherwise keep going max speed
    //std::cout<<"cruising"<<std::endl;
  drive_msg_.velocity=max_speed;
  }
}

  //Distnace of  point from curvature is equal to the difference of the distance of the point from origin and the radius of curvature
float Navigation::findDistanceofPointfromCurve(float x, float y, float curvature){
  float radius = 1 / curvature;
  //use difference of distance formula of point from origin and radius to find distance of point from curve
  return abs(sqrt(pow(x,2) + pow(y,2)) - radius);
}

bool Navigation::isClockwise(float x, float y){
  float counterX = - y;
  float counterY = x;
  return (-x * counterY + y * counterX > 0);
}



std::pair<float, float> Navigation::distanceAlongPath(float x, float y, float curvature){
  float xCoord = x;
  float yCoord = y;
  std::pair<float, float> length_and_angle;
  float radius = 1 / curvature;
  //radiusfromPoint = abs(sqrt(pow(xCoord,2) + pow((yCoord - radius),2)));
  float distancebetweenPoints = abs(sqrt(pow(xCoord,2) + pow((yCoord),2)));
  float theta = acos(1 - (pow(distancebetweenPoints, 2)/ (2 * pow(radius,2))));
  float length = radius * theta;
  length_and_angle.first = length;
  length_and_angle.second = theta;
  return length_and_angle;
}

Eigen::Vector2f Navigation::latency_compensation(const float& latency, unsigned int iterations)
{
    previous_velocities.push_back(robot_vel_);
    previous_locations.push_back(robot_loc_);
    previous_omegas.push_back(robot_omega_);
    previous_speeds.push_back(speed);
    previous_angles.push_back(robot_angle_);

    Eigen::Vector2f predicted_location(robot_loc_);

    if (previous_omegas.size()>iterations){
      previous_omegas.pop_front();
      previous_locations.pop_front();
      previous_velocities.pop_front();
      previous_speeds.pop_front();
      previous_angles.pop_front();
    }

    if (previous_omegas.size()== iterations){
    for (unsigned int i=0; i < iterations; i++)
    {
      predicted_location.x()= predicted_location.x()+ previous_speeds[i] * cos( previous_angles[i] )/20;
      predicted_location.y()= predicted_location.y() + previous_speeds[i] * sin( previous_angles[i] )/20;
    }
    // predicted_location=predicted_location*latency;
    //std::cout<<"predicted_location"<<predicted_location.x()<<" "<<predicted_location.y()<<std::endl;
    //std::cout<<"actual location"<<robot_loc_.x()<<" "<<robot_loc_.y()<<std::endl;
    }
    visualization::DrawCross(robot_loc_, 0.4, 0x32a852,global_viz_msg_);
    visualization::DrawCross(predicted_location, 0.2, 0xFF0000 ,global_viz_msg_);
    return predicted_location;
}


void Navigation::updatePointCloudToGlobalFrame()
{
  float x_p, y_p;
  unsigned int i;
  for (i=0; i < point_cloud_.size(); i++)
  {
    //std::cout << point_cloud_[i] << "before" << std::endl;
    x_p = point_cloud_[i][0] * cos( -robot_angle_ ) - point_cloud_[i][1] * sin( -robot_angle_ ) - robot_loc_[0];
    y_p = point_cloud_[i][0] * sin( -robot_angle_ ) + point_cloud_[i][1] * cos( -robot_angle_ ) - robot_loc_[1];
    point_cloud_[i][0] = x_p;
    point_cloud_[i][1] = y_p;
    //std::cout << "\n" << point_cloud_[i] << "after" << std::endl;
  }
  //std::cout << robot_angle_ << std::endl;
}


// Change this function to give if collision is in innner part or outer part
float Navigation::check_if_collision(float curvature, Eigen::Vector2f& target_point, float inner_radius, float mid_radius, float outer_radius)
{
  // Returns 0 if no collision, 1 if collision on inner side of car, 2 if collision is on outer side of circle
  float x= 0;
  float y=1/curvature;
  // NOT SURE IF CENTER OF CIRCLE IS ALWAYS (0,R)
  float target_x=target_point[0];
  float target_y=target_point[1];
  float distance_from_center = sqrt(pow((x-target_x),2) + pow((y-target_y),2));
  if ( (distance_from_center<inner_radius) || (distance_from_center>outer_radius) )
  {
    return 0;
  }
  else if ( (distance_from_center >= inner_radius) && (distance_from_center <= mid_radius) )
  {
    return 1;
  }
  else if ( (distance_from_center >= mid_radius) && (distance_from_center <= outer_radius) )
  {
    return 2;
  }
  else
  {
    printf("Error in comparing distances");
    exit(0);
    return -1;
  }
}

float Navigation::checkPoint(float angle, float curvature, float x, float y)
{
  float r = 1.0 / curvature;

  float distance=0;

    float x_point = r*cos(angle);
    float y_point = r*sin(angle);
    distance = sqrt(pow(x_point-x,2)+pow(y_point-y,2));




  return distance;
}


// find nearest point in point cloud
float Navigation::findNearestPoint(float curvature, float angle)
{
  if (point_cloud_.size() == 0) return {};

  float minimumDistance = 10000, distance = 0.0;

  for(unsigned int i = 0; i < point_cloud_.size(); i++)
  {

  distance = checkPoint(angle*.1,curvature,point_cloud_[i][0],point_cloud_[i][1]);
  if (distance< minimumDistance){
    minimumDistance=distance;
  }

}
return minimumDistance;
}



Eigen::Vector2f  Navigation::findVectorOfNearestPoint(float curvature, float angle){
  if (point_cloud_.size() == 0) return {};
  float radius = 1 /curvature;
  Eigen::Vector2f  nearestPoint;
  float minimumDistance = 10000;
  float innerRadius = .5 * radius;
  float outerRadius = 1.5 * radius;

    for(unsigned int i = 0; i < point_cloud_.size(); i++){
        float isInsideRange = check_if_collision(curvature, point_cloud_[i], innerRadius, (innerRadius + outerRadius)/2, outerRadius);
        if(isInsideRange == 0){
          if(checkPoint(angle, curvature, point_cloud_[i][0], point_cloud_[i][1])){
            float distance = findDistanceofPointfromCurve(point_cloud_[i][0] , point_cloud_[i][1], curvature);
            if(distance < minimumDistance){

              minimumDistance = distance;
              nearestPoint.x() = point_cloud_[i][0];
              nearestPoint.y() = point_cloud_[i][1];
          }
        }
    }
  }
  return nearestPoint;
}

  //check if point lies in percentage of area in a sector of a circle starting at 0 degrees
bool Navigation::checkPointinSector(float x, float y, float percent, float radius ){
  float angleStart = 0;
  float angleEnd = 350/percent + angleStart;

  //get polar co-ordinates
  float polarRadius = sqrt (x*x + y*y);
  float angle = atan(y /x );

  return (angle >= angleStart && angle <= angleEnd && polarRadius < radius);
}

std::pair<float, float> Navigation::free_path_length_function(float curvature)
{
      //   find inner radius and outer radius
    const float MAX_ANGLE = 2;
    float r=1/curvature;
    float inner_radius = abs(r) - car_width/2 - margin;
    float outer_radius = sqrt( pow( abs(r)+ margin + car_width/2, 2) + pow( car_base_length + (car_length - car_base_length)/2 + margin, 2 ) );
    float mid_radius = sqrt( pow( abs(r) - car_width/2 - margin , 2) + pow( car_base_length + (car_length - car_base_length)/2 + margin, 2 ) );
    float collision = 0, collision_point_angle, total_angle, free_path_angle=0.0, free_path_length=0.0, min_free_path_length, min_free_path_angle=20.0;
    float x, y;
    min_free_path_length = 1000000;

    for (unsigned int i=0; i < point_cloud_.size(); i++)
    {
      x = point_cloud_[i][0], y = point_cloud_[i][1];
      // std::cout << x << " " << y << std::endl;
      if( y * r < 0 )
      {
        // point on opposite side of turning
        continue;
      }
      // if (x < 0) continue; // points with angle>pi/2
      collision = check_if_collision(curvature, point_cloud_[i], inner_radius, mid_radius, outer_radius);
      if( collision == 0)
      {
        // No collision
        continue;
      }
      else if(collision == 1)
      {
        // std::cout << "In inner collision\n";
        //inner collision
        collision_point_angle = acos( (( abs(r) - car_width/2 - margin ) / ( sqrt( pow(x, 2) + pow( y - r, 2 ) ) ) ) );
        total_angle = acos( ( (y-r)*(y-r) + r*r - y*y ) / ( 2*sqrt( x*x + (y-r)*(y-r) )*abs(r) ) );
        free_path_angle = total_angle - collision_point_angle;
        free_path_length = free_path_angle * abs(r);
        // min_free_path_length = std::min( min_free_path_length, free_path_length );
      }

      else if(collision == 2)
      {
        //outer collision
        collision_point_angle = asin( ( ( car_base_length + ( car_length  - car_base_length )/2 + margin ) / ( sqrt( pow( x, 2 ) + pow( y - r, 2 ) ) ) ) );
        total_angle = acos( ( (y-r)*(y-r) + r*r - y*y ) / ( 2*sqrt( x*x + (y-r)*(y-r) )*abs(r) ) );
        free_path_angle = total_angle - collision_point_angle;
        free_path_length = free_path_angle * abs(r);

        // std::cout << x << " " << y << " " << free_path_length << " " << min_free_path_length << " In outer collision\n";
        // min_free_path_length = std::min( min_free_path_length, free_path_length );
      }


      if (min_free_path_length > free_path_length)
      {
        min_free_path_length  = free_path_length;
        min_free_path_angle = free_path_angle;
      }
      // std::cout << "Minimum free path length" << min_free_path_length << " " <<  collision << " " << point_cloud_[i] << " " << i << " " << collision_point_angle << " " << total_angle << std::endl;
    }
    // std::cout<<"===================="<<std::endl;
    // std::cout<<"collision "<<collision<<std::endl;
    // std::cout<<"===================="<<std::endl;
    // std::cout << "Minimum free path length" << min_free_path_length << " " << min_free_path_angle << std::endl;
    if(min_free_path_angle > MAX_ANGLE){
      min_free_path_length = MAX_ANGLE * abs(r);
      min_free_path_angle = MAX_ANGLE;
    }

    if(min_free_path_length > 10)
    {
      min_free_path_length = 10;
    }
    // if (min_free_path_length < 0)
    std::pair<float, float> min_free_path_variables;
    // if (min_free_path_angle > 0.75)
    // {
    //   min_free_path_angle = 0.75;
    //   min_free_path_length = 0.75 * abs(r);
    // }
    min_free_path_variables.first = min_free_path_length;
    min_free_path_variables.second = min_free_path_angle;
    return min_free_path_variables;
}



/*
float return_path_length()
{
  if collision at inner part
    find angle beta and free path length
  else if collision at outer part
    find angle beta and free path length
}
*/


void Navigation::DrawCar()
{
  Eigen::Vector2f front_left = Vector2f( car_base_length + (car_length - car_base_length) / 2 + margin, car_width/2 + margin );
  Eigen::Vector2f back_left = Vector2f( - (car_length - car_base_length) / 2 - margin, car_width/2 + margin );

  Eigen::Vector2f front_right = Vector2f( car_base_length + (car_length - car_base_length) / 2 + margin, -car_width/2 - margin );
  Eigen::Vector2f back_right = Vector2f( - (car_length - car_base_length) / 2 - margin, -car_width/2 - margin );

  visualization::DrawLine( front_left, back_left, 0x32a852,local_viz_msg_);
  visualization::DrawLine( front_right, back_right, 0x32a852,local_viz_msg_);

  visualization::DrawLine( front_left, front_right, 0x32a852,local_viz_msg_);
  visualization::DrawLine( back_left, back_right, 0x32a852,local_viz_msg_);
}


// float findBestCurvature()
// {
//   float best_curvature = 1;
//   free_path_length_function(best_curvature);
//   return best_curvature;
// }


PathOption Navigation::find_optimal_path(unsigned int total_curves, float min_curve, const Eigen::Vector2f target_point)
{

  float current_curvature=-1000.0;
  float current_free_path_length=-1000.0;
  float current_clearance=-1000.0;
  float distance_needed_to_stop= (max_speed*max_speed)/(2*max_deceleration_magnitude);
  //float current_free_path_angle=-1000.0;
  // float current_distance_score=-10000;
  float max_score = -1000000.0; // total_weights;
  //float optimal_angle = 0;
  //int path_num = 1000;

  float current_score=0, curvature_score;
  std::pair<float, float> free_path_length_angle;


  PathOption optimal_path;
  for(unsigned int i =0; i<total_curves;i++)
  {
    current_curvature =  min_curve + i*0.2;
    //std::cout<<"curves "<<current_curvature<<std::endl;
    std::pair<float,float>free_path_pair= free_path_length_function( current_curvature );
    // first is length second is angle

    current_free_path_length = free_path_pair.first;
    //float current_free_path_length_score=0;
    if (current_free_path_length<distance_needed_to_stop || current_free_path_length<.3){
      //std::cout<<"free path < distance needed to stop "<<current_curvature<<std::endl;
      //current_free_path_length_score=-100000000;
      current_free_path_length=-10;
    }

    // if(current_free_path_length < 0.3)
    // {
    //   continue;
    // }
    //current_free_path_angle = free_path_pair.second;

    
    curvature_score = abs(current_curvature);
    // current_free_path_angle = free_path_length_angle.second;


    //// first is length second is angle
    // current_length_and_angle = distanceAlongPath(target_point.x(), target_point.y(), current_curvature);
    // current_length_and_angle.second *= .9;


    /* Find Angle  */

    float theta = current_free_path_length/ (1/current_curvature);
    float x = (1/current_curvature) * sin((theta));
    float y = (1/current_curvature) * (1 - (cos(theta)));
    //float y = (1/current_curvature) * sin(current_free_path_angle);
    Eigen::Vector2f point = Vector2f(x,y);
    

    //distance to goal
    float diff = (point - target_point).norm();


    current_clearance = findNearestPoint( current_curvature, free_path_pair.second );
    if(current_clearance>3|| current_free_path_length<.3){
      current_clearance=0;
    }
    // if (current_clearance < 0.1)`
    // {
    //   continue;
    // }

    // current_distance_score= findDistanceofPointfromCurve(target_point.x(),target_point.y(),current_curvature);

    current_score = 1 * current_free_path_length + 4 * curvature_score + 3 * current_clearance - diff *8;
    //std::cout<< "=================================================" << std::endl;
    //std::cout << "Path #" << i <<" Score terms: current score" << current_score << " current free path length: " << 5*current_free_path_length << " current_clearance: " << 3*current_clearance << " Curvature score: " << 4*curvature_score << " Distance2Goal: " << diff <<std::endl;
   //std::cout<< "=================================================" << std::endl;
    //current_score = 5 * current_free_path_length + 3 * current_clearance + diff * 6;
     // std::cout << " score terms: current score" << current_score << " current free path length: " << 5*current_free_path_length << " current_clearance: " << 3*current_clearance << " Curvature score: " << 4*curvature_score << std::endl;
    // std::cout << "Max score: " << max_score << " " << current_score << "\n" << std::endl;
   if ( max_score < current_score )
    {
      //std::cout << i << std::endl;
      optimal_path.curvature=current_curvature;
      optimal_path.clearance=current_clearance;
      optimal_path.free_path_length=current_free_path_length;
      //new
      //path_num = i;
     // optimal_angle = current_free_path_angle;
      //optimal_path.angle=optimal_path.angle;
      optimal_path.score=current_score;
      max_score = current_score;
    }



    visualization::DrawPathOption(current_curvature, current_free_path_length, current_clearance, local_viz_msg_);
    visualization::DrawPoint(point, 0xff0000, local_viz_msg_);
  }

  /** for(unsigned int i =0; i<total_curves;i++)
  {
    current_curvature =  min_curve + i*0.05;
    current_score = paths[i].score;
    //std::cout << current_score << "\n";
    total_weights = 1;

    if(int(i) - 1 >= 0)
    {
      current_score += 0.4 * paths[i-1].score;
      total_weights += 0.4;
    }
    else if( int(i)-2 >= 0)
    {
      current_score += 0.1 * paths[i-2].score;
      total_weights += 0.1;
    }
    else if( int(i)-3 >= 0)
    {
      current_score += 0.025 * paths[i-3].score;
      total_weights += 0.025;
    }
    else if(i + 1 < total_curves)
    {
      current_score += 0.4 * paths[i+1].score;
      total_weights += 0.4;
    }
    else if(i + 2 < total_curves)
    {
      current_score += 0.1 * paths[i+2].score;
      total_weights += 0.1;
    }
    else if( i+3 < total_curves)
    {
      current_score += 0.025 * paths[i+3].score;
      total_weights += 0.025;
    }
    current_score = current_score / total_weights;
    //std::cout << i << " " << max_score << " " << current_score << std::endl;
    if ( max_score < current_score )
    {
      //std::cout << i << std::endl;
      optimal_path.curvature=current_curvature;
      optimal_path.clearance=current_clearance;
      optimal_path.free_path_length=current_free_path_length;
      optimal_path.score=current_score;
      max_score = current_score;
    }
  }
   ***/
  //float x = (1/optimal_path.curvature) * cos(-(optimal_angle+ robot_angle_)) ;
  //float y = (1/optimal_path.curvature) * sin(optimal_angle + robot_angle_) ;
  //Eigen::Vector2f point = Vector2f(x,y);
  //visualization::DrawCross(target_point, 0.5, 0xff0000, global_viz_msg_);


  //visualization::DrawCross(point, 0.5, 0xff0000, global_viz_msg_);
  //std::cout<< "=================================================" << std::endl;

  //std::cout<<" OPTIMAL CURVE "<<optimal_path.curvature<< " Path Num: " << path_num << std::endl;
  //std::cout<< "=================================================" << std::endl;

  if(optimal_path.free_path_length == -1000)
  {
    exit(0);

  }
  return optimal_path;
}



/******************************************************************************/
/***************************Global Planning************************************/

double Navigation::calculateHeuristic(Eigen::Vector2f node_loc, Eigen::Vector2f target_loc){
  //std::cout << "calculateHeuristic:: Enter" << std::endl;
  double heuristic;
  double dx = abs(node_loc.x() - target_loc.x());
  double dy = abs(node_loc.y() - target_loc.y());
  heuristic = D1 * (dx + dy) + (D2 - 2 * D1) * std::min(dx,dy);
  //std::cout << "calculateHeuristic:: Exit()" << std::endl;
  return heuristic;
}



void Navigation::aStarPathFinder(Eigen::Vector2f destination_loc){

 
SimpleQueue< std::pair<int, int> , double > openList;

  bool foundDestination = false;
  std::pair <int, int> current_node_id;
  
  /**** HARD CODED DESTINATION FOR DEBUGGING ****/
  //destination_loc.x() = 6.0;
  //destination_loc.y() = 16.0;
  /**** HARD CODED DESTINATION FOR DEBUGGING ****/

  pair<int, int> start_id = {0, 0};
  destinationLoc = destination_loc;
  float  goal_bounds = 1;


  int num_of_iterations = 0;
  
  openList.Push(start_id, 1000000);
 //current_node_id = start_id;
  Node current;


  while(!openList.Empty()){
  
    current_node_id = openList.Pop();
    current = node_map[current_node_id];

   /* std::cout<< "========================================" << std::endl;
    std::cout<< "Current Node Loc: " << current.loc << std::endl;
    //std::cout<< "neighbor id first: " << next_neighbor.id.first << std::endl;
    //std::cout<< "neighbor id second: " << next_neighbor.id.second << std::endl;
    std::cout<< "Current Node X: " << current.index.x() << std::endl;
    std::cout<< "Current Node Y: " << current.index.y() << std::endl;
    std::cout<< "Current Node Cost (g): " << current.g << std::endl;          
    std::cout<< "========================================" << std::endl;*/

    float diff = (destination_loc - current.loc).norm();

    if(diff < goal_bounds){
      foundDestination = true;
      break;
    }

    for(Edge neighbor : current.neighbors){
      
      /*std::cout<< "========================================" << std::endl;
      std::cout<< "neighbor num: " << neighbor.neighbor_num << std::endl;
      std::cout<< "neighbor id first: " << neighbor.id.first << std::endl;
      std::cout<< "neighbor id second: " << neighbor.id.second << std::endl;
      std::cout<< "neighbor index X: " << neighbor.neighbor_ind.x() << std::endl;
      std::cout<< "neighbor index Y: " << neighbor.neighbor_ind.y() << std::endl;
      std::cout<< "neighbor weight: " << neighbor.weight << std::endl;
      std::cout<< "========================================" << std::endl;*/
      
      double totalWeight = current.g + neighbor.weight;
      //std::cout<< "TW =  " << totalWeight << " Current Node g = " << current.g << " NW = " << neighbor.weight <<  std::endl;


      if(!node_map.count(neighbor.id) ){
        Node a_node;
        a_node = nodeSetup(current, neighbor.neighbor_num);
        float heuristic = calculateHeuristic( a_node.loc, destination_loc);
        //std::cout<< "f =  " << (totalWeight + heuristic) <<  std::endl;
        openList.Push(neighbor.id, totalWeight + heuristic);
        }
      else{
        if(totalWeight < node_map[neighbor.id].g){
          node_map[neighbor.id].parent_id = current.id;
          node_map[neighbor.id].g = totalWeight;
          float heuristic = calculateHeuristic(node_map[neighbor.id].loc, destination_loc);
          //std::cout<< "f =  " << (totalWeight + heuristic) <<  std::endl;
          openList.Push(neighbor.id, totalWeight + heuristic);   
        }  
      }
    }
    num_of_iterations++;
    //std::cout << "Number of Iterations: " << num_of_iterations <<std::endl;
  }
    if(foundDestination){
      std::cout << "Found Destination" << std::endl;
      path_navigation = construct_path(current);
    }
    else{
      //std::cout << "Failed to find destination" << std::endl;
    }
    //std::cout << "aStarPathFinder :: Exit()" << std::endl;
  }


Eigen::Vector2i Navigation::neighborhoodLookup(int index){

Eigen::Vector2i directional_moves[9] = { 
    Eigen::Vector2i(-1, 1), //NorthWest
    Eigen::Vector2i(0,1),   // North
    Eigen::Vector2i(1, 1),  //NorthEast
    Eigen::Vector2i(-1, 0),  //West
    Eigen::Vector2i(0,0),    //center = ignore
    Eigen::Vector2i(1,0),    //east
    Eigen::Vector2i(-1,-1),   // SouthWest
    Eigen::Vector2i(0,-1),   // South
    Eigen::Vector2i(1,-1)   //SouthEast
  };

  return directional_moves[index];
}

Edge Navigation::NeighborSetup(Eigen::Vector2i loc_index, int neighbor_number){
  Edge neighbor;
  neighbor.neighbor_ind = loc_index + neighborhoodLookup(neighbor_number);
  neighbor.neighbor_num = neighbor_number;
  neighbor.weight = (neighbor_number % 2 == 0) ? diagonal_movement : straight_movement;
  neighbor.id.first = neighbor.neighbor_ind.x();
  neighbor.id.second = neighbor.neighbor_ind.y();
  return neighbor;
}

 // find each neighbor starting from the North-West Direction thru South-East 
 vector<Edge> Navigation::findEightNeighbors(Node node){

  vector<Edge> neighbors;

  for(int i = 0; i < MAX_NEIGHBORS; i++){

    if(i != 4){
      Edge neighbor = NeighborSetup(node.index, i);
      if(isValid(node.loc, node.index, neighbor.neighbor_ind)){
          neighbors.push_back(neighbor);
      }
    }
  }   
      return neighbors;  
}


vector<line2f> Navigation::findMargins(line2f line_edge){
    vector<line2f> margins;

    Vector2f line_edge_vector = line_edge.p1 + 
      ((line_edge.p1 - line_edge.p0)/ (line_edge.p1 - line_edge.p0).norm()) * buffer;

    Vector2f normalized_line_edge = line_edge.UnitNormal() * buffer;

    Vector2f boundary_side_one = line_edge.p0 + normalized_line_edge;
    Vector2f boundary_side_two = line_edge_vector + normalized_line_edge;
    Vector2f boundary_side_three = line_edge.p0 - normalized_line_edge;
    Vector2f boundary_side_four = line_edge_vector - normalized_line_edge;

    margins.push_back(line2f(boundary_side_one, boundary_side_two));
    margins.push_back(line2f(boundary_side_three, boundary_side_four));
    margins.push_back(line2f(boundary_side_one, boundary_side_three));
    margins.push_back(line2f(boundary_side_two, boundary_side_four));

    return margins;
  }

  bool Navigation::isValid(Eigen::Vector2f node_loc, Eigen::Vector2i node_index, Eigen::Vector2i neighbor_ind){
    bool validNeighbor = true; 

    int deltaX = node_index.x() - neighbor_ind.x();
    int deltaY = node_index.y() - neighbor_ind.y();

    if(!(abs(deltaX) == 1 || abs(deltaY) == 1)){
      validNeighbor = false;
      return validNeighbor;
    }

    float dX = resolution * deltaX;
    float dY = resolution * deltaY;

    Vector2f deltaVector(dX, dY);
    Vector2f neighborLoc = node_loc +  deltaVector;

    line2f line_edge(node_loc, neighborLoc);
    vector<line2f> margins = findMargins(line_edge);

    for (size_t i = 0; i < map_.lines.size(); ++i)
      {
        const line2f line = map_.lines[i];

        //TODO: revisit; check for correctness
        bool isCrossingCheckOne = line.Intersects(node_loc, neighborLoc);
        for(line2f margin : margins){
            bool isCrossingCheckTwo = line.Intersects(margin);
            if(isCrossingCheckOne || isCrossingCheckTwo){
              validNeighbor = false;
              return validNeighbor;
            }
        }
      }

      return validNeighbor;
  }

  vector<Node> Navigation::construct_path(Node destination){
    vector<Node> path;
    pair<int, int> id = destination.id;
    pair<int, int> start_id = {0, 0};

    std::cout << "Construct Path: Destination" << std::endl;
    while(id != start_id){
        path_navigation.push_back(node_map[id]);
        id = node_map[id].parent_id; 
    }

    std::reverse(path_navigation.begin(), path_navigation.end());
    path = path_navigation; 
    return path;
  }


  /* Get closest node to path */
  Node Navigation::findTheCarrot(Eigen::Vector2f current_loc){
    std::cout << "find the carrot:: Enter()" << std::endl;
    Node carrot;
    Node closestNode;
    size_t closestNodeIndex = 0;
    size_t carrotIndex = 0;
    float minDistance = std::numeric_limits<float>::max();

    //get closest node in path
    int i = 0;
    for(Node node: path_navigation){

      float diff = (current_loc - node.loc).norm();

      if(diff < minDistance){
        minDistance = diff;
        closestNodeIndex = i;
        closestNode = path_navigation[i];
      }
      
      i++;
    }

    std::cout << "The Closest Node: " << closestNode.loc << std::endl;

    if(minDistance > minimum_radius){
      NEED_TO_RECALCULATE_PATH = true;
      return closestNode;
    }

    // TODO: get next nearest node
    size_t j = closestNodeIndex;

    while(j < path_navigation.size()){
      carrot = path_navigation[j];
      float diff =  (current_loc - carrot.loc).norm();
      if(diff > minimum_radius){
        carrotIndex = j;
        break;
      }

      j++;
    }

    size_t k = carrotIndex;

    while(k > closestNodeIndex){
      Vector2f node_loc_ = path_navigation[k].loc;

      if(!(map_.Intersects(current_loc, node_loc_))){
        carrot = path_navigation[k];
        return carrot;
      }

      if(k < closestNodeIndex + 5){
        NEED_TO_RECALCULATE_PATH = true;
        //carrot = path_navigation[k];
        return carrot;
      }

      k--;
    }

    std::cout << "find the carrot:: End()" << std::endl;
    return carrot;
  }




void Navigation::initialization(Eigen::Vector2f starting_loc){
  std::cout << "Initalization :: Enter()" << std::endl;
  node_map.clear();
  path_navigation.clear();

  Node start;
  
  //start.h = std::numeric_limits<float>::max();
  start.g = 0;
  start.parent_id.first = 0;
  start.parent_id.second = 0;
  start.id.first = 0;
  start.id.second = 0;
  start.loc = starting_loc;
  start.index = Eigen::Vector2i(starting_loc.x() / resolution, starting_loc.y()/resolution);
  start.neighbors = findEightNeighbors(start);

  node_map[start.id] = start;
  std::cout << "Initalization :: Exit()" << std::endl;
}

Node Navigation::nodeSetup(Node node, int neighbor_num){
  Node buildNode;

  Eigen::Vector2i neighbor_i = neighborhoodLookup(neighbor_num);

  buildNode.loc = node.loc + Vector2f(neighbor_i.x(), neighbor_i.y()) * resolution;
  buildNode.index = node.index + neighbor_i;
  buildNode.g = node.g + ((node.loc - buildNode.loc).norm());
  buildNode.parent_id.first = node.id.first;
  buildNode.parent_id.second = node.id.second;
  buildNode.id.first = buildNode.index.x();
  buildNode.id.second = buildNode.index.y();
  buildNode.neighbors = findEightNeighbors(buildNode);

  node_map[buildNode.id] = buildNode;

  return buildNode;
}

void Navigation::recalculate_path(Vector2f destination_loc){
  std::cout << "recalculate_path(): Recalculating path...." << std::endl;

  initialization(robot_loc_);
  std::cout << "recalculate_path(): Node Loc - " << destinationLoc <<std::endl;
  aStarPathFinder(destinationLoc);

}

void Navigation::drawNavigationPath(amrl_msgs::VisualizationMsg &msg){

  Vector2f start = node_map[path_navigation.front().id].loc;
  Vector2f goal = node_map[path_navigation.back().id].loc;
  visualization::DrawCross(start, 0.5, 0xff0000, msg);
  visualization::DrawCross(goal, 0.5, 0xff0000, msg);


  for(Node point : path_navigation){

    Vector2f start_location = point.loc;
    Vector2f end_location = node_map[point.parent_id].loc; 
    visualization::DrawLine(start_location, end_location, 0x009c08, msg);
  }

}


/******************************************************************************/


void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  //std::cout << "\n \n \n New iteration of run" << std::endl;

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  Eigen::Vector2f test_cross1 = Vector2f( 1, 1 );
  Eigen::Vector2f test_cross2 = Vector2f( 2, 2 );

  visualization::DrawLine( test_cross1, test_cross2, 0x32a852, local_viz_msg_);
  // The control iteration goes here.
  // Feel free to make helper functions to structure the control appropriately.

  // The latest observed point cloud is accessible via "point_clouds_"

  // Eventually, you will have to set the control values to issue drive commands:
  // curvature=0;

  // predict location for when commands are executed

  //uncomment when we actually know how to transform points based on new location
  // Eigen::Vector2f predicted_location= latency_compensation(0.3, 6);
  // reconstruct point cloud based on predicted location and rotation

  // find best path based predicted location
  
  if(found_path || found_target){
    //std::cout<<"Run(): Reached Destination - Navigation is complete!!!" << std::endl;
    ros::Duration(0.01).sleep();
  }
  else{


    //Eigen::Vector2f target_point{10,0};
    Node carrot = findTheCarrot(robot_loc_);

    Eigen::Vector2f carrot_point = rotateMaptoBase.transpose()*(carrot.loc - robot_loc_);
    std::cout << "Carrot : " << carrot_point << std::endl;
    std::cout << "Carrot Global : " << carrot.loc << std::endl;
    visualization::DrawCross(carrot.loc, 1, 0x0000FF,local_viz_msg_);

    best_path= find_optimal_path(20, -2.02, carrot_point);

    // decide wether to speed up stay the same or slow down based on distance to target
    updateSpeed(best_path);
    // set trajectory for future time step
    drive_msg_.curvature = best_path.curvature;
    // std::cout << "Robot variables:" << robot_loc_ << "\n Robot velocity: " << robot_vel_ << robot_angle_ << std::endl;
    // std::cout << "Odom variables:" << odom_loc_ << "\n Odom angle: "  << odom_angle_ << "\n Odom start angle:" << odom_start_angle_ << odom_start_loc_ << std::endl;
    // if (point_cloud_set) {std::cout << "Yes, it worked" << point_cloud_.size() << std::endl;
    // }

    // updatePointCloudToGlobalFrame();
    //std::cout << "Best curvature: " << drive_msg_.curvature << std::endl;
    visualization::DrawCross(robot_loc_, 3, 0x32a852,local_viz_msg_);
    DrawCar();

    // std::cout<<robot_loc_.x()<<" "<<robot_loc_.y()<<std::endl;

    drawNavigationPath(global_viz_msg_);
    // Add timestamps to all messages.
    local_viz_msg_.header.stamp = ros::Time::now();
    global_viz_msg_.header.stamp = ros::Time::now();

    drive_msg_.header.stamp = ros::Time::now();
    // Publish messages.
    viz_pub_.publish(local_viz_msg_);
    viz_pub_.publish(global_viz_msg_);
    drive_pub_.publish(drive_msg_);


    if(NEED_TO_RECALCULATE_PATH){
      std::cout << "Run(): Recalculating Path ...." << std::endl;
      recalculate_path(destinationLoc);
      NEED_TO_RECALCULATE_PATH = false;
      ros::Duration(0.5).sleep();
    }
  }

  

  // sleep(1);
  if (drive_msg_.velocity == 0)
  {
    //exit(0);
  }
}

}  // namespace navigation