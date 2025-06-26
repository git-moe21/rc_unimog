//
// You received this file as part of Finroc
// A framework for integrated robot control
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    gMainControl.h
 *
 * \author  Unknown
 *
 * \date    2011-11-07
 *
 * \brief Contains gMainControl
 *
 * \b gMainControl
 *
 */
//----------------------------------------------------------------------
#ifndef _rc_unimog__gMainControl_h_
#define _rc_unimog__gMainControl_h_


//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "plugins/structure/tSenseControlGroup.h"
#include "projects/rc_unimog/shared/gHardwareAbstractionBase.h"
//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace finroc
{
namespace robprak_2024_2
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Short description of gMainControl
/*! A more detailed description of gMainControl which
 *  Max has not done yet!
 *
 */
class gMainControl : public structure::tSenseControlGroup
{

//----------------------------------------------------------------------
// Ports (These are the only variables that may be declared public)
//----------------------------------------------------------------------
public:
  rc_unimog::shared::gHardwareAbstractionBase * hardware;
  
  tControllerOutput<rrlib::si_units::tVelocity<double>> co_velocity;
  tControllerOutput<rrlib::math::tAngleRad> co_slip_angle;
  tControllerOutput<rrlib::si_units::tCurvature<double>> co_curvature;
  
  tControllerOutput<bool> co_enable;
  tControllerOutput<bool> co_activate_camera;
  tControllerOutput<bool> co_warn_buzzer;
  tControllerOutput<bool> co_request_emergency_stop;
  tControllerOutput<data_ports::tEvent> co_request_emergency_stop_event;
  std::vector<tControllerInput<bool>> co_lights;

  tSensorInput<double> si_joystick_velocity_normalized;
  tSensorInput<rrlib::si_units::tVelocity<double>> si_joystick_velocity;
  tSensorInput<double> si_joystick_curvature_normalized;
  tSensorInput<rrlib::si_units::tCurvature<double>> si_joystick_curvature;
  tSensorInput<rrlib::math::tAngleRad> si_joystick_slip_angle;

  tSensorInput<data_ports::tEvent> si_init;
  tSensorInput<bool> si_joystick_initialized;

  tSensorInput<rrlib::si_units::tVelocity<double>> si_velocity;
  tSensorInput<rrlib::math::tAngleRad> si_slip_angle;
  tSensorInput<rrlib::si_units::tCurvature<double>> si_curvature;

  tSensorInput<rrlib::si_units::tVelocity<double>> si_wheel_velocity_rear_right;
  tSensorInput<rrlib::si_units::tVelocity<double>> si_wheel_velocity_rear_left;
  tSensorInput<rrlib::si_units::tAngularVelocity<double>> si_wheel_angular_velocity_rear_right;
  tSensorInput<rrlib::si_units::tAngularVelocity<double>> si_wheel_angular_velocity_rear_left;

  tSensorInput<rrlib::si_units::tLength<double>> si_us_front;
  tSensorInput<rrlib::si_units::tLength<double>> si_us_rear;
  tSensorInput<rrlib::si_units::tLength<double>> si_sharp_right_front;
  tSensorInput<rrlib::si_units::tLength<double>> si_sharp_right_rear;
  tSensorInput<rrlib::si_units::tLength<double>> si_sharp_left_front;
  tSensorInput<rrlib::si_units::tLength<double>> si_sharp_left_rear;

  tSensorInput<rrlib::coviroa::tImages> si_stereo_cam_image;
  tSensorInput<rrlib::distance_data::tDistanceData> si_stereo_point_cloud;

//----------------------------------------------------------------------
// Public methods and typedefs (no fields/variables)
//----------------------------------------------------------------------
public:

  gMainControl(finroc::core::tFrameworkElement *parent, const std::string &name = "MainControl",
               const std::string &structure_config_file = __FILE__".xml");


//----------------------------------------------------------------------
// Protected methods (no fields/variables)
//----------------------------------------------------------------------
protected:

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
