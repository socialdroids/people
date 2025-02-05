/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

/* Author: Wim Meeussen */

#include "people_tracking_filter/people_tracking_node.h"
#include "people_tracking_filter/tracker_particle.h"
#include "people_tracking_filter/tracker_kalman.h"
#include "people_tracking_filter/state_pos_vel.h"
#include "people_tracking_filter/rgb.h"
#include <people_msgs/msg/position_measurement.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/transform_datatypes.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <boost/thread/mutex.hpp> 
#include <Eigen/Dense>

using namespace std;
using namespace tf2;
using namespace BFL;

static const double       sequencer_delay            = 0.8; //TODO: this is probably too big, it was 0.8
static const unsigned int sequencer_internal_buffer  = 100;
static const unsigned int sequencer_subscribe_buffer = 10;
static const unsigned int num_particles_tracker      = 1000;
static const double       tracker_init_dist          = 4.0;

namespace estimation
{
PeopleTrackingNode::PeopleTrackingNode(std::shared_ptr<rclcpp::Node> node)
: rclcpp::Node(node->get_name()), robot_state_(), tracker_counter_(0)
{
  // Initialize
  meas_cloud_.points.push_back(geometry_msgs::msg::Point32());
  meas_cloud_.points[0].x = 0;
  meas_cloud_.points[0].y = 0;
  meas_cloud_.points[0].z = 0;

  // Get parameters
  node_->declare_parameter("fixed_frame", std::string("default"));
  node_->declare_parameter("freq", 1.0);
  node_->declare_parameter("start_distance_min", 0.0);
  node_->declare_parameter("reliability_threshold", 1.0);
  node_->declare_parameter("sys_sigma_pos_x", 0.0);
  node_->declare_parameter("sys_sigma_pos_y", 0.0);
  node_->declare_parameter("sys_sigma_pos_z", 0.0);
  node_->declare_parameter("sys_sigma_vel_x", 0.0);
  node_->declare_parameter("sys_sigma_vel_y", 0.0);
  node_->declare_parameter("sys_sigma_vel_z", 0.0);
  node_->declare_parameter("follow_one_person", false);

  // Advertise filter output
  people_filter_pub_ = node_->create_publisher<people_msgs::msg::PositionMeasurement>("people_tracker_filter", 10);

  // Advertise visualization
  people_filter_vis_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud>("people_tracker_filter_visualization", 10);
  people_tracker_vis_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud>("people_tracker_measurements_visualization", 10);

  // Register message sequencer
  people_meas_sub_ = node_->create_subscription<people_msgs::msg::PositionMeasurement>(
    "people_tracker_measurements", 1,
    [this](const people_msgs::msg::PositionMeasurement::SharedPtr message) {
        this->callbackRcv(message);
    });
}



// destructor
PeopleTrackingNode::~PeopleTrackingNode()
{
  // Delete trackers
  for (auto& tracker : trackers_)
    delete tracker;
}



// callback for messages
void PeopleTrackingNode::callbackRcv(const people_msgs::msg::PositionMeasurement::SharedPtr& message)
{
// Get measurement in fixed frame
  geometry_msgs::msg::Point meas_rel;
  meas_rel.x = message->pos.x;
  meas_rel.y = message->pos.y;
  meas_rel.z = message->pos.z;

  rclcpp::Time stamp = message->header.stamp;
  std::string frame_id = message->header.frame_id;

  geometry_msgs::msg::Point meas;
  try {
    
      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(fixed_frame_, frame_id, tf2::TimePointZero);
      tf2::doTransform(meas_rel, meas, transform);
  } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
  }

  // Get measurement covariance
  SymmetricMatrix cov(3);
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      cov(i + 1, j + 1) = message->covariance[3 * i + j];

  // ----- LOCKED ------
  std::lock_guard<boost::mutex> lock(filter_mutex_);

  // Update tracker if matching tracker found
  for (auto& tracker : trackers_)
    if (tracker->getName() == message->object_id)
    {
      tracker->updatePrediction(message->header.stamp.sec);
      tracker->updateCorrection(tf2::Vector3(meas.x, meas.y, meas.z), cov);
    }

  // Create new tracker if needed
  if (message->object_id == "" && message->reliability > reliability_threshold_)
  {
    double closest_tracker_dist = start_distance_min_;
    StatePosVel est;
    for (auto& tracker : trackers_)
    {
      tracker->getEstimate(est);
      double dst = sqrt(pow(est.pos_[0] - meas.x, 2) + pow(est.pos_[1] - meas.y, 2));
      if (dst < closest_tracker_dist)
        closest_tracker_dist = dst;
    }

    if (message->initialization == 1 && (closest_tracker_dist >= start_distance_min_))
    {
      geometry_msgs::msg::Point pt;
      pt.x = message->pos.x;
      pt.y = message->pos.y;
      pt.z = message->pos.z;

      geometry_msgs::msg::PoseStamped loc;
      loc.header.stamp = message->header.stamp;
      loc.header.frame_id = message->header.frame_id;
      loc.pose.position = pt;

      try {
          geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("base_link", loc.header.frame_id, tf2::TimePointZero);
          tf2::doTransform(loc, loc, transform);
      } catch (const tf2::TransformException &ex) {
          RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
      }

      float cur_dist;
      if ((cur_dist = pow(loc.pose.position.x, 2.0) + pow(loc.pose.position.y, 2.0)) < tracker_init_dist)
      {
        stringstream tracker_name;
        StatePosVel prior_sigma(tf2::Vector3(sqrt(cov(1, 1)), sqrt(cov(2, 2)), sqrt(cov(3, 3))),
                                tf2::Vector3(0.0000001, 0.0000001, 0.0000001));
        tracker_name << "person " << tracker_counter_++;
        Tracker* new_tracker = new TrackerKalman(tracker_name.str(), sys_sigma_);
        tf2::Vector3 meas_vec(meas.x, meas.y, meas.z); 
        new_tracker->initialize(meas_vec, prior_sigma, message->header.stamp.sec);
        trackers_.push_back(new_tracker);
      }
    }
  }

  // Visualize measurement
  meas_cloud_.points[0].x = meas.x;  
  meas_cloud_.points[0].y = meas.y;  
  meas_cloud_.points[0].z = meas.z;  

  meas_cloud_.header.frame_id = message->header.frame_id;
  people_tracker_vis_pub_->publish(meas_cloud_);
}


// // callback for dropped messages
// void PeopleTrackingNode::callbackDrop(const people_msgs::msg::PositionMeasurement::ConstPtr& message)
// {
//   ROS_INFO("DROPPED PACKAGE for %s from %s with delay %f !!!!!!!!!!!",
//            message->object_id.c_str(), message->name.c_str(), (ros::Time::now() - message->header.stamp).toSec());

// }




// filter loop
void PeopleTrackingNode::spin()
{
  RCLCPP_INFO(node_->get_logger(), "People tracking manager started.");

  while (rclcpp::ok())
  {
    std::lock_guard<boost::mutex> lock(filter_mutex_);

    // Visualization variables
    vector<geometry_msgs::msg::Point> filter_visualize(trackers_.size());
    vector<float> weights(trackers_.size());
    sensor_msgs::msg::ChannelFloat32 channel;

    unsigned int i = 0;
    for (auto& tracker : trackers_)
    {
      tracker->updatePrediction(rclcpp::Clock().now().seconds() - sequencer_delay);

      // Publish filter result
      people_msgs::msg::PositionMeasurement est_pos;
      tracker->getEstimate(est_pos);
      est_pos.header.frame_id = fixed_frame_;
      people_filter_pub_->publish(est_pos);

      // Visualize filter result
      filter_visualize[i].x = est_pos.pos.x;
      filter_visualize[i].y = est_pos.pos.y;
      filter_visualize[i].z = est_pos.pos.z;
      weights[i] = *(float*)&(rgb[min(998, 999 - max(1, (int)trunc(tracker->getQuality() * 999.0)))]);

      // Remove trackers that have zero quality
      if (tracker->getQuality() <= 0)
      {
        trackers_.remove(tracker);
        delete tracker;
      }
      i++;
    }

    // Visualize all trackers
    channel.name = "rgb";
    channel.values = weights;
    sensor_msgs::msg::PointCloud people_cloud;
    people_cloud.channels.push_back(channel);
    people_cloud.header.frame_id = fixed_frame_;

    std::vector<geometry_msgs::msg::Point32> point_cloud_points;
    point_cloud_points.reserve(filter_visualize.size());
    for (const auto& point : filter_visualize) {
      geometry_msgs::msg::Point32 p32;
      p32.x = point.x;
      p32.y = point.y;
      p32.z = point.z;
      point_cloud_points.push_back(p32);
    }

    people_cloud.points = point_cloud_points;
    people_filter_vis_pub_->publish(people_cloud);

    // Sleep
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 / freq_)));
  }
}
}; // namespace






// ----------
// -- MAIN --
// ----------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("people_tracker");

    estimation::PeopleTrackingNode my_tracking_node(node);

    my_tracking_node.spin();

    rclcpp::shutdown();

    return 0;
}