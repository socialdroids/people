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

#ifndef PEOPLE_TRACKING_FILTER_TRACKER_PARTICLE_H
#define PEOPLE_TRACKING_FILTER_TRACKER_PARTICLE_H

#include <people_tracking_filter/tracker.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <people_msgs/msg/position_measurement.hpp>
#include <bfl/filter/bootstrapfilter.h>
#include <people_tracking_filter/state_pos_vel.h>
#include <people_tracking_filter/mcpdf_pos_vel.h>
#include <people_tracking_filter/sysmodel_pos_vel.h>
#include <people_tracking_filter/measmodel_pos.h>
#include <fstream>
#include <string>

namespace estimation
{
class TrackerParticle : public Tracker
{
public:
    /// Constructor
    TrackerParticle(const std::string &name, unsigned int num_particles, const BFL::StatePosVel &sysnoise);

    /// Destructor
    virtual ~TrackerParticle();

    /// Initialize tracker
    virtual void initialize(const BFL::StatePosVel &mu, const BFL::StatePosVel &sigma, const double time);

    /// Check if tracker was initialized
    virtual bool isInitialized() const { return tracker_initialized_; }

    /// Get tracker quality: 0 = bad, 1 = good
    virtual double getQuality() const { return quality_; }

    /// Get the lifetime of the tracker
    virtual double getLifetime() const;

    /// Get the time of the tracker
    virtual double getTime() const;

    /// Update tracker
    virtual bool updatePrediction(const double time);
    virtual bool updateCorrection(const tf2::Vector3 &meas, const MatrixWrapper::SymmetricMatrix &cov);

    /// Get filter posterior
    virtual void getEstimate(BFL::StatePosVel &est) const;
    virtual void getEstimate(people_msgs::msg::PositionMeasurement &est) const;

    /// Get evenly spaced particle cloud
    void getParticleCloud(const tf2::Vector3 &step, double threshold, sensor_msgs::msg::PointCloud2 &cloud) const;

    /// Get histogram from a certain area
    MatrixWrapper::Matrix getHistogramPos(const tf2::Vector3 &min, const tf2::Vector3 &max, const tf2::Vector3 &step) const;
    MatrixWrapper::Matrix getHistogramVel(const tf2::Vector3 &min, const tf2::Vector3 &max, const tf2::Vector3 &step) const;

private:
    // ROS2 Node
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    // pdf / model / filter
    BFL::MCPdfPosVel prior_;
    BFL::BootstrapFilter<BFL::StatePosVel, tf2::Vector3> *filter_;
    BFL::SysModelPosVel sys_model_;
    BFL::MeasModelPos meas_model_;

    // Variables
    bool tracker_initialized_;
    double init_time_, filter_time_, quality_;
    unsigned int num_particles_;
};
} // namespace estimation

#endif // PEOPLE_TRACKING_FILTER_TRACKER_PARTICLE_H