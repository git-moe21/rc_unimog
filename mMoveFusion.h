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

#include "plugins/structure/tSenseControlModule.h"
#include "rrlib/si_units/si_units.h"

namespace finroc::robprak_2024_2
{
/** 
 * combine movement functions of robot
 */
class mMoveFusion : public structure::tSenseControlModule
{
public:

public:
  mMoveFusion(core::tFrameworkElement *parent, const std::string &name = "MoveFusion");

  tControllerInput<rrlib::si_units::tCurvature<>> movement_curvature;
  // tControllerInput<rrlib::si_units::tVelocity<>> movement_velocity;
  tControllerInput<bool> point_found;

  tControllerInput<int> obstacle_distance;

  tControllerOutput<rrlib::si_units::tCurvature<>> out_curvature;
  tControllerOutput<rrlib::si_units::tVelocity<>> out_velocity;

  tParameter<int> emergency_stop_distance;

  tSensorInput<int> in_sign;

  tParameter<bool> calibration_active;
  tParameter<rrlib::si_units::tCurvature<>> calibration_curvature_param;

  tSensorInput<rrlib::si_units::tVelocity<double>> avg_wheel_velocity;
  tControllerInput<double> wheel_buffer_downhill;
  tControllerInput<double> break_velocity;
  tControllerInput<double> wheel_buffer_top;
  tParameter<float> p_upper_limit;
  tParameter<float> p_lower_limit;
  tParameter<bool> p_drive;
  tParameter<float> p_fast_limit;
  tParameter<float> p_after_bridge_curvature;
  tParameter<float> p_bridge_straight_timer;
  tParameter<float> p_bridge_curve_timer;
  tParameter<float> p_default_vel;
  tParameter<float> p_climb_vel;

  tParameter<float> p_stop_timer;
  tParameter<float> p_yieldnt_straight_timer;
  tParameter<float> p_yieldnt_curve_timer;

  tParameter<float> p_yieldnt_curve;
  tParameter<float> p_yieldnt_straight;

  tSensorInput<rrlib::si_units::tLength<double>> ci_right_ir;
  tSensorInput<rrlib::si_units::tLength<double>> ci_left_ir;
  tParameter<float> p_on_bridge_curvature_offset;
  tParameter<float> p_on_bridge_curvature;
  tParameter<float> p_lane_switch_distance;
  tParameter<bool> lane_right;
  tControllerOutput<bool> lane_right_output;

  tParameter<int> sign_frame_counter;


protected:
  /** Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  virtual ~mMoveFusion();


private:

enum MovementState {
  TrackMove,
  Bridge,
  ForceStraight,
  ForceLeft,
  Stop,
  AfterBridgeStraight,
  AfterBridgeCurve
};

enum Speed {
  Fast,
  Default,
  NoSpeed,
  Breaking,
};

virtual void OnParameterChange() override;

  virtual void Sense() override;

  virtual void Control() override;

  MovementState movement_state = TrackMove;
  
  time_t state_entered_time;

  int prev_sign = 0;
  int sign_counter = 0; 
  bool bridge_break = false;
  float stuck_counter = 0.0;

  Speed vehicle_speed = NoSpeed;
  bool isDriving = false;
  bool invert_lane = false;

};
}
