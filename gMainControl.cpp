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
/*!\file    gMainControl.cpp
 *
 * \author  Unknown
 *
 * \date    2011-11-07
 *
 */
//----------------------------------------------------------------------
#include "projects/robprak_2024_2/gMainControl.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""

//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>
#include "mTrackMovement.h"
#include "mTrackRecognition.h"
#include "plugins/scheduling/tThreadContainerThread.h"
#include "projects/robprak_2024_2/mMoveFusion.h"
#include "projects/robprak_2024_2/mObstacleRecognition.h"
#include "projects/robprak_2024_2/mTrackRecognition.h"
#include "projects/robprak_2024_2/mSignRecognition.h"
//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
namespace finroc
{
namespace robprak_2024_2
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
runtime_construction::tStandardCreateModuleAction<gMainControl> cCREATE_ACTION_FOR_G_MAIN_CONTROL("MainControl");

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// gMainControl constructors
//----------------------------------------------------------------------
gMainControl::gMainControl(finroc::core::tFrameworkElement *parent, const std::string &name,
                           const std::string &structure_config_file) :
  tSenseControlGroup(parent, name, structure_config_file)

// to access hardware / unreal interface use e.g.
// this->si_velocity.ConnectTo(...)
// to set the velocity of the unimog: this->co_velocity.ConnectTo(out_your_velocity_port);
// etc.
{
  // TrackRecognition
  auto trackrec = new mTrackRecognition(this);
  this->si_stereo_cam_image.ConnectTo(trackrec->zed_camera_images);

  // sensors flipped in unimog
  auto obstacle_rec = new mObstacleRecognition(this);
  this->si_us_front.ConnectTo(obstacle_rec->si_us_front);
  //this->si_stereo_point_cloud.ConnectTo(obstacle_rec->input_point_cloud);

  // auto signrec = new finroc::robprak_2024_2::mSignRecognition(this);
  // this->si_stereo_cam_image.ConnectTo(signrec->zed_camera_image);
  // this->si_stereo_point_cloud.ConnectTo(signrec->point_cloud);
  // trackrec->grid_map.ConnectTo(signrec->grid_map);

  auto trackmove = new mTrackMovement(this);
  trackrec->out_image.ConnectTo(trackmove->input_image_track_rec);
  trackrec->out_red_mask.ConnectTo(trackmove->in_red_mask);
  trackmove->select_image.ConnectTo(trackrec->right_camera);

  auto fusion = new mMoveFusion(this);
  fusion->movement_curvature.ConnectTo(trackmove->curvature);
  // fusion->movement_velocity.ConnectTo(trackmove->velocity);
  this->co_curvature.ConnectTo(fusion->out_curvature);
  this->co_velocity.ConnectTo(fusion->out_velocity);
  fusion->obstacle_distance.ConnectTo(obstacle_rec->out_nearObstacle);
  fusion->avg_wheel_velocity.ConnectTo(this->si_velocity); 
  fusion->lane_right_output.ConnectTo(trackmove->lane_right);
  this->si_sharp_left_rear.ConnectTo(fusion->ci_right_ir);
  this->si_sharp_right_front.ConnectTo(fusion->ci_left_ir);

  // SignRecognition
  auto signrec = new mSignRecognition(this);
  this->si_stereo_cam_image.ConnectTo(signrec->zed_camera_images);
  signrec->out_sign.ConnectTo(fusion->in_sign);


}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}  // namespace robprak_2024_2
}  // namespace finroc
