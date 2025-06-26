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

#include <cassert>
#include <cstddef>
#include <ctime>
#include "rrlib/si_units/si_units.h"
#include "projects/robprak_2024_2/mMoveFusion.h"

namespace finroc::robprak_2024_2
{
//----------------------------------------------------------------------
mMoveFusion::~mMoveFusion()
{}


#ifdef _LIB_FINROC_PLUGINS_RUNTIME_CONSTRUCTION_ACTIONS_PRESENT_
static const runtime_construction::tStandardCreateModuleAction<mMoveFusion> cCREATE_ACTION_FOR_M_MOVEFUSION("MoveFusion");
#endif

mMoveFusion::mMoveFusion(core::tFrameworkElement *parent, const std::string &name) : 
  tSenseControlModule(parent, name, false),
  movement_curvature("Movement Curvature", this),
  // movement_velocity("Movement Velocity", this), 
  point_found("Track Move Target Point Found",this, false),
  obstacle_distance("Obstacle Distance", this),
  out_curvature("Output Curvature", this),
  out_velocity("Output Velocity", this),
  emergency_stop_distance("Emergency Stop Distance", this, 5),
  in_sign("Input Sign", this),
  calibration_active("Calibration Active", this, false),
  calibration_curvature_param("Calibration Curvature", this, 0.0),
  avg_wheel_velocity("Avg Wheel Velocity", this),
  wheel_buffer_downhill("Wheel Buffer Downhill", this, 4.2),
  break_velocity("Break Velocity", this, -0.1),
  wheel_buffer_top("Wheel Buffer Top", this, 3.9),
  p_upper_limit("velocity upper bound",this,10.0),
  p_lower_limit("velocity lower bound",this,0.1),
  p_drive("enable driving", this, false),
  p_fast_limit("upper limit for vehicle fast", this, 4.8),
  p_after_bridge_curvature("curvature after bridge", this, 1.0),
  p_bridge_straight_timer("bridge straight bridge", this, 0.8),
  p_bridge_curve_timer("bridge curvature timer", this, 7.0),
  p_default_vel("default vel", this, 0.142),
  p_climb_vel("fast vel", this, 0.21),
  p_stop_timer("stop sign timer", this, 0.5),
  p_yieldnt_straight_timer("priority sign straight case timer",this,0.5),
  p_yieldnt_curve_timer("priority sign curve case timer",this,0.5),
  p_yieldnt_curve("priority sign curve amplitude",this,-0.5),
  p_yieldnt_straight("priority sign straight amplitude",this,0.0),
  ci_right_ir("infrared sensor right input", this),
  ci_left_ir("infrared sensor left input", this),
  p_on_bridge_curvature_offset("on bridge steering offset", this, 0.0),
  p_on_bridge_curvature("on bridge steering mult", this, 0.7),
  p_lane_switch_distance("Lane switch distance obstacle", this, 2.0),
  lane_right("Lane right", this, true),
  lane_right_output("Lane Right Output", this),
  sign_frame_counter("Sign Frame Counter", this,22)
{
  time(&state_entered_time);
}

void mMoveFusion::OnParameterChange()
{

}
void mMoveFusion::Sense()
{
}

void mMoveFusion::Control()
{
  float curvature = 0.0;
  
  // Sign rec continous detection counter
  int current_sign = in_sign.Get();
  if (current_sign == prev_sign) {
    sign_counter++;
  } else {
    sign_counter = 0;
    prev_sign = current_sign;
  }

  bool lane_right_value = lane_right.Get();

  auto obstacle_dist = obstacle_distance.Get();
  auto emergency_stop = emergency_stop_distance.Get();

  // Movement state machine
  switch(movement_state) {
    // Normal driving on track
    case TrackMove:
      curvature = movement_curvature.Get().Value();

      if (vehicle_speed == Fast) {
        time(&state_entered_time); 
        movement_state = Bridge;
      } 

      if (sign_counter >= sign_frame_counter.Get() && difftime(time(NULL), state_entered_time) > 3.0) {
        switch(prev_sign) {
          case 2: // Stop Sign ? 
            time(&state_entered_time);
            movement_state = Stop;
            break;
          //case 1: // Yield sign
          //  time(&state_entered_time);
          // movement_state = ForceLeft;
          //  break;
        }
      }


      if (obstacle_dist <= p_lane_switch_distance.Get() && obstacle_dist > emergency_stop && difftime(time(NULL), state_entered_time) > 4.0) {
        time(&state_entered_time);
        invert_lane = !invert_lane;
      }

      isDriving = true;
      break;
    case Bridge:
      curvature = 0.0;
      // ci_left_ir only output (in reality rear right), middle 230 - 300 + 600 , right 890, left 160
      // if (ci_left_ir.Get().Value() > 450) {
      //   curvature = p_on_bridge_curvature.Get();
      // } else if (ci_left_ir.Get().Value() < 220 && ci_left_ir.Get().Value() > 160) {
      //   curvature = -p_on_bridge_curvature.Get();
      // }

      if (ci_left_ir.Get().Value() > 500) {
        curvature = -(ci_left_ir.Get().Value() - 500)/500 * p_on_bridge_curvature.Get();
      } else if (ci_right_ir.Get().Value() > 400) {
        curvature = (ci_right_ir.Get().Value() - 400)/600 * p_on_bridge_curvature.Get();
      }

      // curvature = 0.0;
      if(vehicle_speed == Breaking){
        stuck_counter = 0;
        time(&state_entered_time); 
        movement_state = AfterBridgeStraight;
      }
      isDriving = true;
      break;
    // Drive down bridge
    case AfterBridgeStraight:
      curvature = 0.0;
      if (difftime(time(NULL), state_entered_time) > p_bridge_straight_timer.Get()) {
        time(&state_entered_time); 
        movement_state = AfterBridgeCurve;
      }
      isDriving = true;
      break;
    // drive down bridge second part with curve 
    case AfterBridgeCurve:
      curvature = p_after_bridge_curvature.Get();
      if (difftime(time(NULL), state_entered_time) > p_bridge_curve_timer.Get()) {
        time(&state_entered_time);
        movement_state = TrackMove;
      }
      isDriving = true;
      break;
    // Take straight path at sign
    case ForceStraight: // priority sign case 1
      curvature = p_yieldnt_straight.Get();
      if (difftime(time(NULL), state_entered_time) > p_yieldnt_straight_timer.Get()) {
        time(&state_entered_time);
        movement_state = TrackMove;
      }
      isDriving = true;
      break;
    // Take left path at sign
    case ForceLeft: // priority sign case 2
      curvature = p_yieldnt_curve.Get();
      if (difftime(time(NULL), state_entered_time) > p_yieldnt_curve_timer.Get()) {
        time(&state_entered_time);
        movement_state = TrackMove;
      }
      isDriving = true;
      break;
    // Wait at stop sign
    case Stop:
      curvature = movement_curvature.Get().Value();
      isDriving = false;
      if (difftime(time(NULL), state_entered_time) > p_stop_timer.Get()) {
        time(&state_entered_time);
        movement_state = TrackMove;
        sign_counter = 0;
      }
      break;
  }

  // handle vehicle velocity
  if (isDriving) {
    float avg_velocity = avg_wheel_velocity.Get().Value();
    if (vehicle_speed == Fast) {
      if ( avg_velocity > p_fast_limit.Get()) {
        vehicle_speed = Default;
      }
    } else {
      if (avg_velocity > p_upper_limit.Get()) {
        vehicle_speed = Breaking;
      } else if (avg_velocity <= p_lower_limit.Get() && stuck_counter >= 20) {
        vehicle_speed = Fast;
      } else {
        vehicle_speed = Default;
      }
    }
  } else {
    vehicle_speed = NoSpeed;
  }

  float velocity = 0;
  switch (vehicle_speed) {
    case Default:
      velocity = p_default_vel.Get();
      break;
    case Fast:
      velocity = p_climb_vel.Get();
      break;
    case Breaking:
      velocity = break_velocity.Get();
      break;
    case NoSpeed:
      break;
  }

  if (obstacle_dist < emergency_stop && obstacle_dist != 0) {
    velocity = 0.0;
  } 

  // Override Velocity with param
  if (!p_drive.Get()) {
    velocity = 0.0;
  }

  // Detect getting stuck at bridge ramp
  if (avg_wheel_velocity.Get().Value() == 0.0 && velocity > 0.0) {
    stuck_counter += 1;
  } else {
    stuck_counter = 0;
  }

  if (calibration_active.Get()) {
    out_curvature.Publish(calibration_curvature_param.Get());
  } else {
    out_curvature.Publish(curvature);
  }
  
  out_velocity.Publish(velocity);
  if (invert_lane) {
    lane_right_value = !lane_right_value;
  }
  lane_right_output.Publish(lane_right_value);
}


}
