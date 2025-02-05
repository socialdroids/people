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

#include "people_tracking_filter/mcpdf_pos_vel.h"
#include <assert.h>
#include <vector>
#include "std_msgs/msg/float64.hpp"
#include "people_tracking_filter/rgb.h"
#include "geometry_msgs/msg/point32.hpp"  
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/channel_float32.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include <cstring>

using namespace MatrixWrapper;
using namespace BFL;
using namespace tf2;

static const unsigned int NUM_CONDARG   = 1;


MCPdfPosVel::MCPdfPosVel(unsigned int num_samples)
  : MCPdf<StatePosVel> (num_samples, NUM_CONDARG)
{}

MCPdfPosVel::~MCPdfPosVel() {}


WeightedSample<StatePosVel>
MCPdfPosVel::SampleGet(unsigned int particle) const
{
  assert((int)particle >= 0 && particle < _listOfSamples.size());
  return _listOfSamples[particle];
}


StatePosVel MCPdfPosVel::ExpectedValueGet() const
{
  tf2::Vector3 pos(0, 0, 0);
  tf2::Vector3 vel(0, 0, 0);
  double current_weight;
  std::vector<WeightedSample<StatePosVel> >::const_iterator it_los;
  for (it_los = _listOfSamples.begin() ; it_los != _listOfSamples.end() ; it_los++)
  {
    current_weight = it_los->WeightGet();
    pos += (it_los->ValueGet().pos_ * current_weight);
    vel += (it_los->ValueGet().vel_ * current_weight);
  }
  return StatePosVel(pos, vel);
}


/// Get evenly distributed particle cloud
void MCPdfPosVel::getParticleCloud(const tf2::Vector3& step, double threshold, sensor_msgs::msg::PointCloud2& cloud) const
{
  unsigned int num_samples = _listOfSamples.size();
  assert(num_samples > 0);
  Vector3 m = _listOfSamples[0].ValueGet().pos_;
  Vector3 M = _listOfSamples[0].ValueGet().pos_;

  // Calculate min and max
  for (unsigned int s = 0; s < num_samples; s++)
  {
    Vector3 v = _listOfSamples[s].ValueGet().pos_;
    for (unsigned int i = 0; i < 3; i++)
    {
      if (v[i] < m[i]) m[i] = v[i];
      if (v[i] > M[i]) M[i] = v[i];
    }
  }

  // Get point cloud from histogram
  Matrix hist = getHistogramPos(m, M, step);
  unsigned int row = hist.rows();
  unsigned int col = hist.columns();
  unsigned int total = 0;
  unsigned int t = 0;
  for (unsigned int r = 1; r <= row; r++)
    for (unsigned int c = 1; c <= col; c++)
      if (hist(r, c) > threshold) total++;

  // Create points and weights
  std::vector<float> points; // To store the point data (x, y, z)
  std::vector<float> weights(total);

  // Create the PointField for the RGB channel
  sensor_msgs::msg::PointField rgb_field;
  rgb_field.name = "rgb";
  rgb_field.offset = 0;
  rgb_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  rgb_field.count = 1; // The size of the RGB data per point

  // Fill in the points and weights
  for (unsigned int r = 1; r <= row; r++)
    for (unsigned int c = 1; c <= col; c++)
      if (hist(r, c) > threshold)
      {
        // Add point (x, y, z)
        points.push_back(m[0] + (step[0] * r));
        points.push_back(m[1] + (step[1] * c));
        points.push_back(m[2]);

        // Add weight (RGB value)
        weights[t] = rgb[999 - (int)trunc(max(0.0, min(999.0, hist(r, c) * 2 * total * total)))];
        t++;
      }

  // Set up the PointCloud2 message
  cloud.header.frame_id = "odom_combined";
  cloud.height = 1;  // Typically for point clouds with a single row
  cloud.width = total; // Number of points
  cloud.fields.push_back(rgb_field); // Add the RGB channel field
  cloud.is_bigendian = false;
  cloud.point_step = sizeof(float) * 3 + sizeof(float); // 3 floats for XYZ and 1 float for RGB
  cloud.row_step = cloud.point_step * total;
  cloud.data.resize(cloud.row_step * cloud.height);

  // Fill the cloud data with the points and weights
  uint8_t* data_ptr = cloud.data.data();
  for (size_t i = 0; i < points.size(); i += 3)
  {
    memcpy(data_ptr, &points[i], sizeof(float) * 3); // XYZ
    data_ptr += sizeof(float) * 3;
    memcpy(data_ptr, &weights[i / 3], sizeof(float)); // RGB
    data_ptr += sizeof(float);
  }

  // Add the RGB channel data
  sensor_msgs::msg::ChannelFloat32 channel;
  channel.name = "rgb";
  channel.values = weights; // Set the weight (RGB) values to the channel
}



/// Get histogram from pos
MatrixWrapper::Matrix MCPdfPosVel::getHistogramPos(const Vector3& m, const Vector3& M, const Vector3& step) const
{
  return getHistogram(m, M, step, true);
}


/// Get histogram from vel
MatrixWrapper::Matrix MCPdfPosVel::getHistogramVel(const Vector3& m, const Vector3& M, const Vector3& step) const
{
  return getHistogram(m, M, step, false);
}


/// Get histogram from certain area
MatrixWrapper::Matrix MCPdfPosVel::getHistogram(const Vector3& m, const Vector3& M, const Vector3& step, bool pos_hist) const
{
  unsigned int num_samples = _listOfSamples.size();
  unsigned int rows = round((M[0] - m[0]) / step[0]);
  unsigned int cols = round((M[1] - m[1]) / step[1]);
  Matrix hist(rows, cols);
  hist = 0;

  // calculate histogram
  for (unsigned int i = 0; i < num_samples; i++)
  {
    Vector3 rel;
    if (pos_hist)
      rel = _listOfSamples[i].ValueGet().pos_ - m;
    else
      rel = _listOfSamples[i].ValueGet().vel_ - m;

    unsigned int r = round(rel[0] / step[0]);
    unsigned int c = round(rel[1] / step[1]);
    if (r >= 1 && c >= 1 && r <= rows && c <= cols)
      hist(r, c) += _listOfSamples[i].WeightGet();
  }

  return hist;
}



unsigned int
MCPdfPosVel::numParticlesGet() const
{
  return _listOfSamples.size();
}


