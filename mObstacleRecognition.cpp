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

#include <algorithm>
#include <cassert>
#include <cmath>
#include <string>
#include "core/log_messages.h"
#include "projects/robprak_2024_2/mObstacleRecognition.h"

namespace finroc::robprak_2024_2
{
//----------------------------------------------------------------------
mObstacleRecognition::~mObstacleRecognition()
{}


#ifdef _LIB_FINROC_PLUGINS_RUNTIME_CONSTRUCTION_ACTIONS_PRESENT_
static const runtime_construction::tStandardCreateModuleAction<mObstacleRecognition> cCREATE_ACTION_FOR_M_OBSTACLERECOGNITION("ObstacleRecognition");
#endif

mObstacleRecognition::mObstacleRecognition(core::tFrameworkElement *parent, const std::string &name) : 
  tModule(parent, name, false),
  si_us_front("us_front", this),
  out_nearObstacle("nearObstacle", this)
  //out_farObstacle("farObstacle", this),
  //input_point_cloud("point_cloud", this)
{

}

void mObstacleRecognition::OnParameterChange()
{

}


void mObstacleRecognition::Update()
{
  read_ultrasonic_front();
}

void mObstacleRecognition::read_ultrasonic_front()
{
  const int MAX_DISTANCE = 25;

  // read ultrasonic front sensor
  if (this->si_us_front.HasChanged())
  {
    rrlib::si_units::tLength<double> us_front = si_us_front.Get();
    // Convert to cm and compensation
    int front_distance_cm = static_cast<int>(us_front * 100) / 2;

    if (prev[indx] <= MAX_DISTANCE && front_distance_cm > MAX_DISTANCE) {
      counter--;
    } else if (prev[indx] > MAX_DISTANCE && front_distance_cm <= MAX_DISTANCE) {
      counter++;
    }
    
    sum -= prev[indx];
    sum += front_distance_cm;

    counter = std::max(0, std::min(4, counter));

    int avg = 0;
    if (counter >= 3) { avg = sum / 4; } 
    out_nearObstacle.Publish(avg);

    prev[indx] = front_distance_cm;
    indx = (indx + 1) % 4;
  }

//  const int CHUNKSIZE = 5;
//  int chunks[18] = {0}; 


//  if (input_point_cloud.IsConnected() && input_point_cloud.HasChanged()) {
//    data_ports::tPortDataPointer<const rrlib::distance_data::tDistanceData> cloud = input_point_cloud.GetPointer();
//    const rrlib::math::tVec3f* data = reinterpret_cast<const rrlib::math::tVec3f*>(cloud->DataPtr());
//
//    for (unsigned int i = 0; i < cloud->Dimension(); i++) {
//      rrlib::math::tVec3f point = data[i];
//      double x = point.X();
//      double y = point.Y();
//      double z = point.Z();
//
//      double floor_height = -28 + x * tan(0.25 * 3.1415926535);
//
//      if (x < 0.9 && z > floor_height && 6.0 * std::abs(y) <= x) {
//        int indx = static_cast<int>(x * 100.0) / CHUNKSIZE;
//        chunks[indx]++;
//      }
//    } 
//
//    int min_count = *std::min_element(chunks, chunks + 17);
//    int empty_threadhold = min_count * 2 + 10;
//
//    int obstacle_index = 0;
//    int distance = 0;
//    for (int i = 0; i < 18; i++) {
//      if (chunks[i] > empty_threadhold) {
//        obstacle_index = i;
//        distance = obstacle_index * CHUNKSIZE;
//        break;
//      }
//    }
//    out_farObstacle.Publish(distance);
//
//    std::string chunks_str = "";
//    for (int i = 0; i < 18; i++) {
//      chunks_str += std::to_string(chunks[i]) + " ";
//    }
//
//    FINROC_LOG_PRINT(ERROR, "Chunks: %s", chunks_str.c_str());
//  }
}


}
