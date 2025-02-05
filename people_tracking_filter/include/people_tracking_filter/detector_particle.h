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
/* Maluco que passou para ros2: Miguelito*/

#include "tracker.h"

// bayesian filtering
#include <bfl/filter/bootstrapfilter.h>
#include "mcpdf_vector.h"
#include "measmodel_vector.h"
#include "sysmodel_vector.h"

// TF2
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_listener.h>

// msgs
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <people_msgs/msg/position_measurement.hpp>

// log files
#include <fstream>

namespace estimation
{

class DetectorParticle
{
public:
  /// constructor
  DetectorParticle(unsigned int num_particles);

  /// destructor
  ~DetectorParticle();

  /// initialize detector
  void initialize(const tf2::Vector3& mu, const tf2::Vector3& size, const double time);

  /// return if detector was initialized
  bool isInitialized() const
  {
    return detector_initialized_;
  };

  /// return measure for detector quality: 0=bad 1=good
  double getQuality() const
  {
    return quality_;
  };

  /// update detector
  bool updatePrediction(const double dt);
  bool updateCorrection(const tf2::Vector3& meas,
                        const MatrixWrapper::SymmetricMatrix& cov,
                        const double time);

  /// get filter posterior
  void getEstimate(tf2::Vector3& est) const;
  void getEstimate(people_msgs::msg::PositionMeasurement& est) const;

  // get evenly spaced particle cloud
  void getParticleCloud(const tf2::Vector3& step, double threshold, sensor_msgs::msg::PointCloud2& cloud) const;

  /// Get histogram from certain area
  MatrixWrapper::Matrix getHistogram(const tf2::Vector3& min, const tf2::Vector3& max, const tf2::Vector3& step) const;

private:
  // pdf / model / filter
  BFL::MCPdfVector                                          prior_;
  BFL::BootstrapFilter<tf2::Vector3, tf2::Vector3>* filter_;
  BFL::SysModelVector                                       sys_model_;
  BFL::MeasModelVector                                      meas_model_;

  // vars
  bool detector_initialized_;
  double filter_time_, quality_;
  unsigned int num_particles_;
};

}; // namespace estimation
