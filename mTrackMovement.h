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
#include "rrlib/si_units/si_units.h"

namespace finroc::robprak_2024_2
{
/**
 * Drives the Racetrack using TrackRecognition, SignRecognition and ObstacleRecognition
 */
class mTrackMovement : public structure::tSenseControlModule
{
public:
public:
  mTrackMovement(core::tFrameworkElement *parent, const std::string &name = "TrackMovement");
  tSensorInput<rrlib::coviroa::tImage> input_image_track_rec;
  tControllerOutput<rrlib::si_units::tCurvature<>> curvature;
  tControllerOutput<rrlib::si_units::tVelocity<>> velocity;
  tParameter<int> track_width_param;
  tParameter<int> red_track_width_param;
  tParameter<int> bucket_width_param;
  tParameter<int> y_offset_param;
  tParameter<int> min_bucket_size_param;
  // tParameter<rrlib::si_units::tVelocity<>> velocity_param;
  // tParameter<bool> override_velocity_param;
  tSensorOutput<bool> select_image;
  tParameter<rrlib::si_units::tCurvature<>> curvature_param;
  tParameter<float> camera_offset_param_left;
  tParameter<float> camera_offset_param_right;
  tParameter<float> curvature_mult_param;
  tParameter<float> curvature_y_offset_param;
  tSensorInput<rrlib::coviroa::tImage> in_red_mask;
  tSensorOutput<rrlib::coviroa::tImage> bucket_viz_image;
  tControllerOutput<bool> co_point_found;
  tParameter<float> p_offset_to_mid;
  tControllerInput<bool> lane_right;

protected:
  /** Destructor
   *
   * The destructor of modules is declared protected to avoid accidental deletion. Deleting
   * modules is already handled by the framework.
   */
  virtual ~mTrackMovement();

private:


  struct RowPoint {
    int x;
    int y;
    bool is_red;
  };

  void ToggleLane();

  virtual void OnParameterChange() override;

  virtual void Sense() override;

  virtual void Control() override;

  std::vector<RowPoint> detect_lines(uchar* row,int width, int height, int track_width);

  std::vector<std::vector<std::array<int, 2>>> detect_buckets(int height, int width, uchar* grid, uchar* red_grid);

  int closest_white(std::vector<mTrackMovement::RowPoint> lines, int target_x);

  std::vector<mTrackMovement::RowPoint> slicing(std::vector<mTrackMovement::RowPoint>& arr,
    int X, int Y);

  bool wrong_lane = false;
  int prev_lanes_seen = 0;
  double prev_target_left = 0;
  double prev_target_right = 0;
  double track_width = 0;
  double prev_curvatures[5] = {0, 0, 0, 0, 0};
  int prev_curvatures_index = 0;
  float prev_target_value = 0;
  int three_lanes_continoues = 0;
};
}  // namespace finroc::robprak_2024_2
