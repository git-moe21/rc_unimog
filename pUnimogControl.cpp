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
/*!\file    projects/rc_unimog_control_test/pUnimogControl.cpp
 *
 * \author  Tobias Groll
 *
 * \date    2017-03-31
 *
 * \brief Contains mUnimogControl
 *
 * \b pUnimogControl
 *
 * controller program for the rc unimog
 *
 */
//----------------------------------------------------------------------
#include "plugins/structure/default_main_wrapper.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#include <chrono>
#include <cassert>


//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "projects/rc_unimog/gRemoteInterface.h"
#include "projects/rc_unimog/hardware/gHardwareAbstraction.h"
#include "projects/robprak_2024_2/gMainControl.h"

#ifdef _LIB_FINROC_LIBRARIES_UNREAL_PRESENT_
#include "projects/rc_unimog/unreal/gHardwareAbstraction.h"
#endif
//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
enum class tUnimogControlMode
{
  eHARDWARE,
  eUNREAL,
  ePLAYBACK,
};
//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
const std::string cPROGRAM_DESCRIPTION = "This program executes the UnimogControl module/group.";
const std::string cCOMMAND_LINE_ARGUMENTS = "";
const std::string cADDITIONAL_HELP_TEXT = "";
bool make_all_port_links_unique = true;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

static tUnimogControlMode unimog_mode = tUnimogControlMode::eHARDWARE;

bool UnimogControlModeHandler(const rrlib::getopt::tNameToOptionMap &name_to_option_map)
{
  rrlib::getopt::tOption value(name_to_option_map.at("unimog_mode"));
  if (value->IsActive())
  {
    std::string unimog_mode_string = rrlib::getopt::EvaluateValue(value);
    if (unimog_mode_string == "hardware" || unimog_mode_string == "Hardware" || unimog_mode_string == "HARDWARE")
    {
      unimog_mode = tUnimogControlMode::eHARDWARE;
    }
    else if (unimog_mode_string == "unreal" || unimog_mode_string == "Unreal" || unimog_mode_string == "UNREAL")
    {
      unimog_mode = tUnimogControlMode::eUNREAL;
    }
    else if (unimog_mode_string == "playback" || unimog_mode_string == "Playback" || unimog_mode_string == "PLAYBACK")
    {
      unimog_mode = tUnimogControlMode::ePLAYBACK;
    }
    else
    {
      FINROC_LOG_PRINT(ERROR, "Unknown unimog control mode: ", unimog_mode_string);
    }
  }
  return true;
}

//----------------------------------------------------------------------
// StartUp
//----------------------------------------------------------------------
void StartUp()
{
      rrlib::getopt::AddValue("unimog_mode", 'm', "Selects the available unimog mode (hardware / unreal / playback).", &UnimogControlModeHandler);
}

//----------------------------------------------------------------------
// CreateMainGroup
//----------------------------------------------------------------------
void CreateMainGroup(const std::vector<std::string> &remaining_arguments)
{
    auto main_thread = new finroc::structure::tTopLevelThreadContainer<>("Main Thread", __FILE__".xml", true, make_all_port_links_unique);
    main_thread->SetCycleTime(std::chrono::milliseconds(20));

    auto control = new finroc::robprak_2024_2::gMainControl(main_thread);

    if (unimog_mode == tUnimogControlMode::eHARDWARE)
    {
        FINROC_LOG_PRINT(USER, "Initialize hardware connections.");
        control->hardware = new finroc::rc_unimog::gRemoteInterface(control, "Remote Interface");
        control->hardware->GetSensorOutputs().ConnectByName(control->GetSensorInputs(),true);
        control->hardware->GetControllerInputs().ConnectByName(control->GetControllerOutputs(),true);
    }
    #ifdef _LIB_FINROC_LIBRARIES_UNREAL_PRESENT_
    else if (unimog_mode == tUnimogControlMode::eUNREAL)
    {
        FINROC_LOG_PRINT(USER, "Initialize simulation connections.");
        control->hardware = new finroc::rc_unimog::unreal::gHardwareAbstraction(control, "Hardware Interface");
        control->hardware->GetSensorOutputs().ConnectByName(control->GetSensorInputs(),true);
        control->hardware->GetControllerInputs().ConnectByName(control->GetControllerOutputs(),true);   
     }
    #endif
    else
    {
        FINROC_LOG_PRINT(ERROR, "No argument given. The main thread will not work");
    }

}



