//
// You received this file as part of Finroc
// A framework for intelligent robot control
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
#pragma once

#include "plugins/structure/tModule.h"
#include "rrlib/distance_data/tDistanceData.h"
#include "rrlib/si_units/si_units.h"

namespace finroc::robprak_2024_2
{
/** 
 * Detects obstacles
 * uses:
 * pointcloud,
 * ultrasonic sensors
 * 
 * 
 */
class mObstacleRecognition : public structure::tModule
{
public:
  tInput<rrlib::si_units::tLength<double>> si_us_front;
  /**
   * 0 -> No Obstacle detected
   * n -> Distance to closest Obstacle in cm
   */
  tOutput<int> out_nearObstacle;
  /**
   * 0 -> No Obstacle detected
   * n -> Distance to closest Obstacle in cm
   */
  // tOutput<int> out_farObstacle;
  // tInput<rrlib::distance_data::tDistanceData> input_point_cloud;

public:
  mObstacleRecognition(core::tFrameworkElement *parent, const std::string &name = "ObstacleRecognition");


protected:
  /** Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  virtual ~mObstacleRecognition();


private:
  int counter = 4;
  int prev[4] = {0};
  int indx = 0;
  int sum = 0;

  virtual void OnParameterChange() override;

  void read_ultrasonic_front();
  virtual void Update() override;
};
}
