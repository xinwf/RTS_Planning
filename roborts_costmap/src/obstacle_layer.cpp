/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include "obstacle_layer_setting.pb.h"
#include "obstacle_layer.h"
#include "line_iterator.h"

namespace roborts_costmap {
using roborts_local_planner::FastLineIterator;
void ObstacleLayer::OnInitialize() {
  ros::NodeHandle nh;
  ParaObstacleLayer para_obstacle;

  std::string obstacle_map = ros::package::getPath("roborts_costmap") + \
      "/config/obstacle_layer_config.prototxt";
  roborts_common::ReadProtoFromTextFile(obstacle_map.c_str(), &para_obstacle);
  double observation_keep_time = 0.1, expected_update_rate = 10.0, min_obstacle_height = 0.2, \
 max_obstacle_height = 0.6, obstacle_range = 2.5, raytrace_range = 3.0, transform_tolerance = 0.2;
  observation_keep_time = para_obstacle.observation_keep_time();
  expected_update_rate = para_obstacle.expected_update_rate();
  transform_tolerance = para_obstacle.transform_tolerance();
  max_obstacle_height = para_obstacle.max_obstacle_height();
  min_obstacle_height = para_obstacle.min_obstacle_height();
  obstacle_range = para_obstacle.obstacle_range();
  raytrace_range = para_obstacle.raytrace_range();
  max_obstacle_height_ = max_obstacle_height;
  footprint_clearing_enabled_ = para_obstacle.footprint_clearing_enabled();
  std::string topic_string = "LaserScan", sensor_frame = "laser_frame";
  topic_string = para_obstacle.topic_string();
  sensor_frame = para_obstacle.sensor_frame();

  bool inf_is_valid = false, clearing = false, marking = true;
  inf_is_valid = para_obstacle.inf_is_valid();
  clearing = para_obstacle.clearing();
  marking = para_obstacle.marking();
  rolling_window_ = layered_costmap_->IsRollingWindow();
  bool track_unknown_space = layered_costmap_->IsTrackingUnknown();
  if (track_unknown_space) {
    default_value_ = NO_INFORMATION;
  } else {
    default_value_ = FREE_SPACE;
  }
  is_current_ = true;
  global_frame_ = layered_costmap_->GetGlobalFrameID();
  ObstacleLayer::MatchSize();
  observation_buffers_.push_back(std::shared_ptr<ObservationBuffer>(new ObservationBuffer(topic_string,
                                                                                            observation_keep_time,
                                                                                            expected_update_rate,
                                                                                            min_obstacle_height,
                                                                                            max_obstacle_height,
                                                                                            obstacle_range,
                                                                                            raytrace_range,
                                                                                            *tf_,
                                                                                            global_frame_,
                                                                                            sensor_frame,
                                                                                            transform_tolerance)));
  if (marking) {
    marking_buffers_.push_back(observation_buffers_.back());
  }
  if (clearing) {
    clearing_buffers_.push_back(observation_buffers_.back());
  } 
  reset_time_ = std::chrono::system_clock::now();
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>
  > sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, topic_string, 50));
  std::shared_ptr<tf::MessageFilter<sensor_msgs::LaserScan>
  > filter(new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50));
  if (inf_is_valid) {
    filter->registerCallback(
        boost::bind(&ObstacleLayer::LaserScanValidInfoCallback, this, _1, observation_buffers_.back()));
  } else {
    filter->registerCallback(
        boost::bind(&ObstacleLayer::LaserScanCallback, this, _1, observation_buffers_.back()));
  }
  observation_subscribers_.push_back(sub);
  observation_notifiers_.push_back(filter);
  observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
  std::vector<std::string> target_frames;
  target_frames.push_back(global_frame_);
  target_frames.push_back(sensor_frame);
  observation_notifiers_.back()->setTargetFrames(target_frames);
  GetGlassBoundPoint();
  is_enabled_ = true;
  buff_index_ = -1;
  ros_buff_sub_ = nh.subscribe("buff_index" ,10, &ObstacleLayer::BuffIndexCallBack , this);
}

void ObstacleLayer::LaserScanCallback(const sensor_msgs::LaserScanConstPtr &message,
                                      const std::shared_ptr<ObservationBuffer> &buffer) {
  sensor_msgs::PointCloud2 temp_cloud;
  temp_cloud.header = message->header;
  try {
    projector_.transformLaserScanToPointCloud(temp_cloud.header.frame_id, *message, temp_cloud, *tf_);
  }
  catch (tf::TransformException &ex) {
    projector_.projectLaser(*message, temp_cloud);
  }
  buffer->Lock();
  buffer->BufferCloud(temp_cloud);
  buffer->Unlock();
}

void ObstacleLayer::LaserScanValidInfoCallback(const sensor_msgs::LaserScanConstPtr &raw_message,
                                               const std::shared_ptr<ObservationBuffer> &buffer) {
  float epsilon = 0.0001, range;
  sensor_msgs::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++) {
    range = message.ranges[i];
    if (!std::isfinite(range) && range > 0) {
      message.ranges[i] = message.range_max - epsilon;
    }
  }
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message.header;
  try {
    projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("High fidelity enabled, but TF returned a transform exception to frame %s: %s", \
        global_frame_.c_str(), ex.what());
    projector_.projectLaser(message, cloud);
  }
  buffer->Lock();
  buffer->BufferCloud(cloud);
  buffer->Unlock();
}

void ObstacleLayer::UpdateBounds(double robot_x,
                                 double robot_y,
                                 double robot_yaw,
                                 double *min_x,
                                 double *min_y,
                                 double *max_x,
                                 double *max_y) {
  if (rolling_window_) {
    UpdateOrigin(robot_x - GetSizeXWorld() / 2, robot_y - GetSizeYWorld() / 2);
  } else if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - reset_time_) > std::chrono::seconds(2)){
    reset_time_ = std::chrono::system_clock::now();
    ResetMaps();
  }
  if (!is_enabled_) {
    ROS_ERROR("Obstacle layer is not enabled.");
    return;
  }
  UpdateStaticObsOrigin(robot_x,robot_y);
  UseExtraBounds(min_x, min_y, max_x, max_y);
  GetBuffClearingBounds(robot_x , robot_y);
  bool temp_is_current = true;
  std::vector<Observation> observations, clearing_observations;
  temp_is_current = temp_is_current && GetMarkingObservations(observations);
  temp_is_current = temp_is_current && GetClearingObservations(clearing_observations);
  is_current_ = temp_is_current;

  for (unsigned int i = 0; i < clearing_observations.size(); ++i) {
        RaytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }

  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); it++) {
    const Observation obs = *it;
    const pcl::PointCloud<pcl::PointXYZ> &cloud = *(obs.cloud_);
    double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;
    for (unsigned int i = 0; i < cloud.points.size(); ++i) {
      double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;

      // if the obstacle is too high or too far away from the robot we won't add it
      if (pz > max_obstacle_height_) {
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y)
          + (pz - obs.origin_.z) * (pz - obs.origin_.z);

      // if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_range) {
        continue;
      }



      // now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      if (!World2Map(px, py, mx, my)) {
        continue;
      }
      unsigned int index = GetIndex(mx, my);
      costmap_[index] = LETHAL_OBSTACLE;
      geometry_msgs::PointStamped buff_point[4];
      if(GetBuffClearingBounds(buff_point)){
          if(px > buff_point[0].point.x
            && px < buff_point[1].point.x
            && py > buff_point[3].point.y
            && py < buff_point[1].point.y){
                costmap_[index] = FREE_SPACE;
            }
      }
      Touch(px, py, min_x, min_y, max_x, max_y);
    }
  }
  UpdateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::UpdateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) {
  if (!is_enabled_) {
    ROS_WARN("Obstacle layer is not enabled");
    return;
  }

  if (footprint_clearing_enabled_) {
    SetConvexRegionCost(transformed_footprint_, FREE_SPACE);
  }
  combination_method_ = 1;
  switch (combination_method_) {
    case 0:  // Overwrite
      UpdateOverwriteByValid(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      UpdateOverwriteByMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

void ObstacleLayer::Activate() {
  for (size_t i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != nullptr) {
      observation_subscribers_[i]->subscribe();
    }
  }
  for (size_t i = 0; i < observation_buffers_.size(); ++i) {
    if (observation_buffers_[i] != nullptr) {
      observation_buffers_[i]->ResetLastUpdated();
    }
  }
}

void ObstacleLayer::Deactivate() {
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
    if (observation_subscribers_[i] != nullptr)
      observation_subscribers_[i]->unsubscribe();
  }
}

void ObstacleLayer::Reset() {
  Deactivate();
  ResetMaps();
  is_current_ = true;
  Activate();
}

bool ObstacleLayer::GetMarkingObservations(std::vector<Observation> &marking_observations) const {
  bool current = true;
  // get the marking observations
  for (size_t i = 0; i < marking_buffers_.size(); ++i) {
    marking_buffers_[i]->Lock();
    marking_buffers_[i]->GetObservations(marking_observations);
    current = marking_buffers_[i]->IsCurrent() && current;
    marking_buffers_[i]->Unlock();
  }
  marking_observations.insert(marking_observations.end(),
                              static_marking_observations_.begin(), static_marking_observations_.end());
  return current;
}

bool ObstacleLayer::GetClearingObservations(std::vector<Observation> &clearing_observations) const {
  bool current = true;
  // get the clearing observations
  for (unsigned int i = 0; i < clearing_buffers_.size(); ++i) {
    clearing_buffers_[i]->Lock();
    clearing_buffers_[i]->GetObservations(clearing_observations);
    current = clearing_buffers_[i]->IsCurrent() && current;
    clearing_buffers_[i]->Unlock();
  }
  clearing_observations.insert(clearing_observations.end(),
                               static_clearing_observations_.begin(), static_clearing_observations_.end());
  return current;
}

void ObstacleLayer::RaytraceFreespace(const Observation &clearing_observation,
                                      double *min_x,
                                      double *min_y,
                                      double *max_x,
                                      double *max_y) {
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  pcl::PointCloud<pcl::PointXYZ> cloud = *(clearing_observation.cloud_);

  // get the map coordinates of the origin of the sensor
  unsigned int x0, y0;
  if (!World2Map(ox, oy, x0, y0)) {
    return;
  }

  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  double origin_x = origin_x_, origin_y = origin_y_;
  double map_end_x = origin_x + size_x_ * resolution_;
  double map_end_y = origin_y + size_y_ * resolution_;

  Touch(ox, oy, min_x, min_y, max_x, max_y);

  // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
  for (unsigned int i = 0; i < cloud.points.size(); ++i) {
    double wx = cloud.points[i].x;
    double wy = cloud.points[i].y;

    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the map and scale if necessary
    double a = wx - ox;
    double b = wy - oy;

    // the minimum value to raytrace from is the origin
    if (wx < origin_x) {
      double t = (origin_x - ox) / a;
      wx = origin_x;
      wy = oy + b * t;
    }
    if (wy < origin_y) {
      double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }

    // the maximum value to raytrace to is the end of the map
    if (wx > map_end_x) {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y) {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    unsigned int x1, y1;

    // check for legality just in case
    if (!World2Map(wx, wy, x1, y1))
      continue;

    unsigned int cell_raytrace_range = World2Cell(clearing_observation.raytrace_range_);
    MarkCell marker(costmap_, FREE_SPACE);
    // and finally... we can execute our trace to clear obstacles along that line
    RaytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);

    UpdateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
  }
}

void ObstacleLayer::UpdateRaytraceBounds(double ox,
                                         double oy,
                                         double wx,
                                         double wy,
                                         double range,
                                         double *min_x,
                                         double *min_y,
                                         double *max_x,
                                         double *max_y) {
  double dx = wx - ox, dy = wy - oy;
  double full_distance = hypot(dx, dy);
  double scale = std::min(1.0, range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  Touch(ex, ey, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::UpdateFootprint(double robot_x,
                                    double robot_y,
                                    double robot_yaw,
                                    double *min_x,
                                    double *min_y,
                                    double *max_x,
                                    double *max_y) {
  if (!footprint_clearing_enabled_)
    return;
  TransformFootprint(robot_x, robot_y, robot_yaw, GetFootprint(), transformed_footprint_);

  for (size_t i = 0; i < transformed_footprint_.size(); i++) {
    Touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

void ObstacleLayer::BuffIndexCallBack(const roborts_msgs::BuffClearIndex::ConstPtr clear_buff_index){
    buff_index_ = clear_buff_index->buff_index;
}

void ObstacleLayer::GetBuffClearingBounds(float robot_x , float robot_y){
    if(buff_index_ < 0 && static_clearing_observations_.size() > 0){
        static_clearing_observations_.pop_back();
    }else if(buff_index_ > 0){
        std::vector<geometry_msgs::PointStamped> buff_point;
        float map_offset_x = 0.28;
        float map_offset_y = 0.23;
        switch(buff_index_){
            case 2:   // F2
            buff_point = GlassPoint(Point2PointStamped(5.91f + (float)map_offset_x,3.07f + (float)map_offset_y),
                                Point2PointStamped(6.45f + (float)map_offset_x,3.07f + (float)map_offset_y),
                                Point2PointStamped(6.45f + (float)map_offset_x,2.40f + (float)map_offset_y),
                                Point2PointStamped(5.91f + (float)map_offset_x,2.40f + (float)map_offset_y));
            
            break;

            case 3:  // F3
            buff_point = GlassPoint(Point2PointStamped(3.77f + (float)map_offset_x,0.685f + (float)map_offset_y),
                                Point2PointStamped(4.31f + (float)map_offset_x,0.685f + (float)map_offset_y),
                                Point2PointStamped(4.31f + (float)map_offset_x,0.205f + (float)map_offset_y),
                                Point2PointStamped(3.77f + (float)map_offset_x,0.205f + (float)map_offset_y));
            
            break;

            case 4:  // F4
            buff_point = GlassPoint(Point2PointStamped(3.77f + (float)map_offset_x,4.275f + (float)map_offset_y),
                                Point2PointStamped(4.31f + (float)map_offset_x,4.275f + (float)map_offset_y),
                                Point2PointStamped(4.31f + (float)map_offset_x,3.795f + (float)map_offset_y),
                                Point2PointStamped(3.77f + (float)map_offset_x,3.795f + (float)map_offset_y));
            
            break;

            case 5: //F5
            buff_point = GlassPoint(Point2PointStamped(1.63f + (float)map_offset_x,1.89f + (float)map_offset_y),
                                Point2PointStamped(2.17f + (float)map_offset_x,1.89f  + (float)map_offset_y),
                                Point2PointStamped(2.17f + (float)map_offset_x,1.41f + (float)map_offset_y),
                                Point2PointStamped(1.63f + (float)map_offset_x,1.41f + (float)map_offset_y));
            
            break;
        }
        sensor_msgs::PointCloud2 cloud = Point2Sensor(buff_point);
        pcl::PCLPointCloud2 pcl_pc2;
        // sensor_msgs::PointCloud2 -> pcl::PCLPointCloud2.
        pcl_conversions::toPCL(cloud, pcl_pc2);
        pcl::PointCloud < pcl::PointXYZ > pcl_cloud;
        pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
        Observation static_obs;
        static_clearing_observations_.push_back(static_obs);
        static_clearing_observations_.back().origin_.x = 0;
        static_clearing_observations_.back().origin_.y = 0;
        static_clearing_observations_.back().origin_.z = 0 ;
        static_clearing_observations_.back().obstacle_range_ = 1.0;
        static_clearing_observations_.back().raytrace_range_ = 5.0;
        pcl::PointCloud < pcl::PointXYZ > &observation_cloud = *(static_clearing_observations_.back().cloud_);
        unsigned int cloud_size = pcl_cloud.points.size();
        observation_cloud.points.resize(cloud_size);
        unsigned int point_count = 0;
        for (unsigned int i = 0; i < cloud_size; ++i)
        {
            observation_cloud.points[point_count++] = pcl_cloud.points[i];
        }
        observation_cloud.header.stamp = pcl_cloud.header.stamp;

        static_clearing_observations_.back().origin_.x = robot_x;
        static_clearing_observations_.back().origin_.y = robot_y;
    }
  }

bool ObstacleLayer::GetBuffClearingBounds(geometry_msgs::PointStamped *buff_point){
    if(buff_index_ < 0){
        return false;
    }else if(buff_index_ >=2 && buff_index_ <=5){
        float map_offset_x = 0.28;
        float map_offset_y = 0.23;
        switch(buff_index_){
            case 2:   // F2
            buff_point[0] = Point2PointStamped(5.91-0.3f + (float)map_offset_x,3.07+0.3f + (float)map_offset_y);
            buff_point[1] = Point2PointStamped(6.45+0.3f + (float)map_offset_x,3.07+0.3f + (float)map_offset_y);
            buff_point[2] = Point2PointStamped(6.45+0.3f + (float)map_offset_x,2.40f + (float)map_offset_y);
            buff_point[3] = Point2PointStamped(5.91-0.3f + (float)map_offset_x,2.40f + (float)map_offset_y);
            break;

            case 3:  // F3
            buff_point[0] = Point2PointStamped(3.77f + (float)map_offset_x,0.685+0.2f + (float)map_offset_y);
            buff_point[1] = Point2PointStamped(4.31f + (float)map_offset_x,0.685+0.2f + (float)map_offset_y);
            buff_point[2] = Point2PointStamped(4.31f + (float)map_offset_x,0.205-0.2f + (float)map_offset_y);
            buff_point[3] = Point2PointStamped(3.77f + (float)map_offset_x,0.205-0.2f + (float)map_offset_y);
            break;

            case 4:  // F4
            buff_point[0] = Point2PointStamped(3.77f + (float)map_offset_x,4.275+0.2f + (float)map_offset_y);
            buff_point[1] = Point2PointStamped(4.31f + (float)map_offset_x,4.275+0.2f + (float)map_offset_y);
            buff_point[2] = Point2PointStamped(4.31f + (float)map_offset_x,3.795-0.2f + (float)map_offset_y);
            buff_point[3] = Point2PointStamped(3.77f + (float)map_offset_x,3.795-0.2f + (float)map_offset_y);
            break;

            case 5: //F5
            buff_point[0] = Point2PointStamped(1.63-0.3f + (float)map_offset_x,1.89f + (float)map_offset_y);
            buff_point[1] = Point2PointStamped(2.17+0.3f + (float)map_offset_x,1.89f  + (float)map_offset_y);
            buff_point[2] = Point2PointStamped(2.17+0.3f + (float)map_offset_x,1.41-0.3f + (float)map_offset_y);
            buff_point[3] = Point2PointStamped(1.63-0.3f + (float)map_offset_x,1.41-0.3f + (float)map_offset_y);
            break;
        }
        return true;
    }else{
        return false;
    }
}

 Observation ObstacleLayer::GetGlassBoundPoint(){
    static_marking_observations_.clear();
    std::vector<geometry_msgs::PointStamped> all_glass_point;
    std::vector<geometry_msgs::PointStamped> temp_point;
    float map_offset_x = 0.28;
    float map_offset_y = 0.23;
    // b1
    temp_point = GlassPoint(Point2PointStamped(7.08f + (float)map_offset_x,1.2f + (float)map_offset_y),
                            Point2PointStamped(8.08f + (float)map_offset_x,1.2f + (float)map_offset_y),
                            Point2PointStamped(8.08f + (float)map_offset_x,1.0f + (float)map_offset_y),
                            Point2PointStamped(7.08f + (float)map_offset_x,1.0f + (float)map_offset_y));
    all_glass_point.insert(all_glass_point.end(),temp_point.begin(),temp_point.end());
    temp_point.clear();

    // b3
    temp_point = GlassPoint(Point2PointStamped(6.38f + (float)map_offset_x,4.48f + (float)map_offset_y),
                            Point2PointStamped(6.58f + (float)map_offset_x,4.48f + (float)map_offset_y),
                            Point2PointStamped(6.58f + (float)map_offset_x,3.48f + (float)map_offset_y),
                            Point2PointStamped(6.38f + (float)map_offset_x,3.48f + (float)map_offset_y));
    all_glass_point.insert(all_glass_point.end(),temp_point.begin(),temp_point.end());
    temp_point.clear();

    // b5
    temp_point = GlassPoint(Point2PointStamped(1.50f + (float)map_offset_x,1.0f + (float)map_offset_y),
                            Point2PointStamped(1.70f + (float)map_offset_x,1.0f + (float)map_offset_y),
                            Point2PointStamped(1.70f + (float)map_offset_x,1.0f + (float)map_offset_y),
                            Point2PointStamped(1.50f + (float)map_offset_x,0.0f + (float)map_offset_y));
    all_glass_point.insert(all_glass_point.end(),temp_point.begin(),temp_point.end());
    temp_point.clear();

    // b7
    temp_point = GlassPoint(Point2PointStamped(0.0f + (float)map_offset_x,3.48f + (float)map_offset_y),
                            Point2PointStamped(1.0f + (float)map_offset_x,3.48f + (float)map_offset_y),
                            Point2PointStamped(1.0f + (float)map_offset_x,3.28f + (float)map_offset_y),
                            Point2PointStamped(0.0f+ (float)map_offset_x,3.28f + (float)map_offset_y));
    all_glass_point.insert(all_glass_point.end(),temp_point.begin(),temp_point.end());

    temp_point = GlassPoint(Point2PointStamped(4.215-0.25f + (float)map_offset_x,2.365f + (float)map_offset_y),
                            Point2PointStamped(4.465-0.25f + (float)map_offset_x,2.365f + (float)map_offset_y),
                            Point2PointStamped(4.465-0.25f + (float)map_offset_x,2.115f + (float)map_offset_y),
                            Point2PointStamped(4.215-0.25f+ (float)map_offset_x,2.115f + (float)map_offset_y));
    all_glass_point.insert(all_glass_point.end(),temp_point.begin(),temp_point.end());

    temp_point = GlassPoint(Point2PointStamped(3.54f + (float)map_offset_x,3.345+0.2f + (float)map_offset_y),
                            Point2PointStamped(3.54+1.0f + (float)map_offset_x,3.345+0.2f + (float)map_offset_y),
                            Point2PointStamped(3.54+1.0f + (float)map_offset_x,3.345f + (float)map_offset_y),
                            Point2PointStamped(3.54f+ (float)map_offset_x,3.345f + (float)map_offset_y));
    all_glass_point.insert(all_glass_point.end(),temp_point.begin(),temp_point.end());

    temp_point = GlassPoint(Point2PointStamped(3.54f + (float)map_offset_x,0.935+0.2f + (float)map_offset_y),
                            Point2PointStamped(3.54+1.0f + (float)map_offset_x,0.935+0.2f + (float)map_offset_y),
                            Point2PointStamped(3.54+1.0f + (float)map_offset_x,0.935f + (float)map_offset_y),
                            Point2PointStamped(3.54f+ (float)map_offset_x,0.935f + (float)map_offset_y));
    all_glass_point.insert(all_glass_point.end(),temp_point.begin(),temp_point.end());
    temp_point.clear();
    sensor_msgs::PointCloud2 cloud = Point2Sensor(all_glass_point);
    pcl::PCLPointCloud2 pcl_pc2;
    // sensor_msgs::PointCloud2 -> pcl::PCLPointCloud2.
    pcl_conversions::toPCL(cloud, pcl_pc2);
    pcl::PointCloud < pcl::PointXYZ > pcl_cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
    Observation static_obs;
    static_marking_observations_.push_back(static_obs);
    static_marking_observations_.back().origin_.x = 0;
    static_marking_observations_.back().origin_.y = 0;
    static_marking_observations_.back().origin_.z = 0 ;
    static_marking_observations_.back().obstacle_range_ = 5.0;
    static_marking_observations_.back().raytrace_range_ = 3.0;
    pcl::PointCloud < pcl::PointXYZ > &observation_cloud = *(static_marking_observations_.back().cloud_);
    unsigned int cloud_size = pcl_cloud.points.size();
    observation_cloud.points.resize(cloud_size);
    unsigned int point_count = 0;
    for (unsigned int i = 0; i < cloud_size; ++i)
    {
        observation_cloud.points[point_count++] = pcl_cloud.points[i];
    }
    observation_cloud.header.stamp = pcl_cloud.header.stamp;
 }

 void ObstacleLayer::UpdateStaticObsOrigin(float robot_x , float robot_y){
     static_marking_observations_.back().origin_.x = robot_x;
     static_marking_observations_.back().origin_.y = robot_y;
 }

 geometry_msgs::PointStamped ObstacleLayer::Point2PointStamped( float x, float y){
     geometry_msgs::PointStamped point;
     point.header.frame_id = "map";
     point.point.x = x;
     point.point.y = y;
     return point;
 }

sensor_msgs::PointCloud2 ObstacleLayer::Point2Sensor(std::vector<geometry_msgs::PointStamped> point){
    sensor_msgs::PointCloud2 cl_pc2;
    sensor_msgs::PointCloud pc;
    cl_pc2.header.frame_id = "map" ;
    pc.header.frame_id = "map";

    for(auto iter = point.begin() ; iter != point.end() ; iter++){
        geometry_msgs::Point32 point;
        point.x = iter->point.x;
        point.y = iter->point.y;
        point.z = iter->point.z;
        pc.points.push_back(point);
    }
    sensor_msgs::convertPointCloudToPointCloud2(pc ,cl_pc2);
    return cl_pc2;
}


 std::vector<geometry_msgs::PointStamped> ObstacleLayer::LineIter(geometry_msgs::PointStamped begin , geometry_msgs::PointStamped end ,int keep_dim){
     std::vector<geometry_msgs::PointStamped> point_vector;
     switch (keep_dim)
     {
     case 0:
         if(begin.point.x > end.point.x){
             geometry_msgs::PointStamped temp;
             temp.point = begin.point;
             begin.point = end.point;
             end.point = temp.point;
         }
         while (begin.point.x <= end.point.x)
         {
             begin.point.x += 0.05;
             point_vector.push_back(begin);
         }
         break;
     case 1:
        if(begin.point.y > end.point.y){
             geometry_msgs::PointStamped temp;
             temp.point = begin.point;
             begin.point = end.point;
             end.point = temp.point;
         }
         while (begin.point.y <= end.point.y)
         {
             begin.point.y += 0.05;
             point_vector.push_back(begin);
         }
         break;
     default:
         break;
     }
     return point_vector;
 }

 std::vector<geometry_msgs::PointStamped> ObstacleLayer::GlassPoint(
                                            geometry_msgs::PointStamped left_up , 
                                            geometry_msgs::PointStamped right_up,
                                            geometry_msgs::PointStamped right_down,
                                            geometry_msgs::PointStamped left_down){
    std::vector<geometry_msgs::PointStamped> glass_point;
    std::vector<geometry_msgs::PointStamped> l ;

    l = LineIter(left_up , right_up , false);
    glass_point.insert(glass_point.end(),l.begin(),l.end());
    l.clear();

    l = LineIter(right_up, right_down, true);
    glass_point.insert(glass_point.end(),l.begin(),l.end());
    l.clear();

    l = LineIter(right_down, left_down, false);
    glass_point.insert(glass_point.end(),l.begin(),l.end());
    l.clear();

    l = LineIter(left_down, left_up, true);
    glass_point.insert(glass_point.end(),l.begin(),l.end());
    // geometry_msgs::PointStamped point;
    // point.header.frame_id = "map";
    // point.point.x = 1.0;
    // point.point.y = 1.0;
    // glass_point.insert(glass_point.end(),point);

    return glass_point;
}



} //namespace roborts_costmap