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
#include "rrlib/coviroa/tImage.h"

namespace finroc::robprak_2024_2
{
/**
 * Recognizes the race track and transforms the classified img into gridMap format
 */
class mTrackRecognition : public structure::tSenseControlModule
{
public:
  tSensorInput<rrlib::coviroa::tImages> zed_camera_images;     // unimog
  tSensorInput<rrlib::coviroa::tImage> zed_camera_solo_image;  // unreal
  tSensorOutput<rrlib::coviroa::tImage> out_image;
  tSensorOutput<rrlib::coviroa::tImage> out_red_mask;
  tSensorInput<bool> right_camera;
  tParameter<int> search_bot_y;
  tParameter<int> search_min_width;
  tParameter<int> search_left_cap;
  tParameter<int> search_border_offset;
  tParameter<int> search_top_y;
  tParameter<bool> dynamic_mask;
  tParameter<float> scaleFactor;

  tParameter<int> param_1;
  tParameter<int> param_2;

public:
  mTrackRecognition(core::tFrameworkElement *parent, const std::string &name = "TrackRecognition");

protected:
  /** Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  virtual ~mTrackRecognition();

private:
  virtual void OnParameterChange() override;

  virtual void Sense() override;

  virtual void Control() override;
};
}  // namespace finroc::robprak_2024_2
