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
#ifndef __TRACKER__
#define __TRACKER__

#include "state_pos_vel.h"
#include <rclcpp/rclcpp.hpp> // ROS 2 base library
#include <tf2/LinearMath/Vector3.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <string>
#include "people_msgs/msg/position_measurement.hpp"
namespace estimation
{

class Tracker
{
public:
  /// Construtor
  Tracker(const std::string& name) : name_(name) {}

  /// Destruidor
  virtual ~Tracker() {}

  /// Retorna o nome do tracker
  const std::string& getName() const
  {
    return name_;
  }

  /// Inicializa o tracker
  virtual void initialize(const BFL::StatePosVel& mu, const BFL::StatePosVel& sigma, const double time) = 0;

  /// Retorna se o tracker foi inicializado
  virtual bool isInitialized() const = 0;

  /// Retorna uma medida de qualidade do tracker: 0=ruim 1=boa
  virtual double getQuality() const = 0;

  /// Retorna a vida útil do tracker
  virtual double getLifetime() const = 0;

  /// Retorna o tempo do tracker
  virtual double getTime() const = 0;

  /// Atualiza o tracker com a previsão
  virtual bool updatePrediction(const double time) = 0;

  /// Atualiza o tracker com a correção
  virtual bool updateCorrection(const tf2::Vector3& meas,
                                const MatrixWrapper::SymmetricMatrix& cov) = 0;

  /// Obtém a estimativa do filtro
  virtual void getEstimate(BFL::StatePosVel& est) const = 0;

  /// Obtém a estimativa em formato de medida de posição
  virtual void getEstimate(people_msgs::msg::PositionMeasurement& est) const = 0;

private:
  std::string name_;

}; // classe Tracker

}; // namespace estimation

#endif
