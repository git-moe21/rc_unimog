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
#include <algorithm>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "core/log_messages.h"
#include "rrlib/coviroa/tImage.h"
#include "rrlib/coviroa/opencv_utils.h"
#include "projects/robprak_2024_2/mTrackMovement.h"
#include <opencv2/core/hal/interface.h>
#include <stdio.h>

namespace finroc::robprak_2024_2
{

mTrackMovement::~mTrackMovement()
{
}

mTrackMovement::mTrackMovement(core::tFrameworkElement* parent, const std::string& name) :
  tSenseControlModule(parent, name, false),
  input_image_track_rec("Classified Track Image", this),
  track_width_param("Track Width", this, 40),
  red_track_width_param("Red Track Width", this, 40),
  bucket_width_param("Bucket Width", this, 50),
  y_offset_param("Y Offset", this, 120),  // 150 unmog 1
  min_bucket_size_param("Min Bucket Size", this, 40),
  // velocity_param("Velocity", this, 0.0),
  // override_velocity_param("Override Velocity", this, true),
  select_image("Select Image", this, false),
  curvature_param("Curvature", this, 0.0),
  camera_offset_param_left("Camera Offset Left", this, -20),   // 0 unimog 21
  camera_offset_param_right("Camera Offset Right", this, -14),  // -60 unimog 1
  curvature_mult_param("Curvature Mult", this, -1200.0),       // positive for unimog 1
  curvature_y_offset_param("Curvature Offset", this, 10.0),
  in_red_mask("Red Mask", this),
  bucket_viz_image("Bucket Viz", this),
  co_point_found("Track Move Point Found", this, false),
  p_offset_to_mid("offset width to center for target x selection",this, 0.5),
  lane_right("Lane right", this)
{
}

void mTrackMovement::OnParameterChange()
{
}

void mTrackMovement::Sense()
{
}

bool comp(std::array<unsigned long, 2> a, std::array<unsigned long, 2> b)
{
  return a[1] > b[1];
}

bool comp2(std::vector<std::array<int, 2>> a, std::vector<std::array<int, 2>> b)
{
  return a[a.size() - 1][0] > b[b.size() - 1][0];
}

std::vector<mTrackMovement::RowPoint> mTrackMovement::detect_lines(uchar* row, int width, int height, int track_width)
{
  std::vector<mTrackMovement::RowPoint> result = std::vector<mTrackMovement::RowPoint>();

  mTrackMovement::RowPoint ret_val = {0, 0, false};

  int MIN_TRACK_WIDTH = track_width;

  int start_x = 0;
  int end_x = 0;
  int skip = 0;

  for (int x = 0; x < width; x++)
  {
    int color = row[x];
    if (skip > 0)
    {
      if (color >= 155)
      {
        end_x = x;
      }
      skip--;
      continue;
    }

    if (color >= 155)
    {
      if (start_x != 0 && end_x != 0)
      {
        ret_val.x = (start_x + end_x) / 2;
        ret_val.y = height;
        result.push_back(ret_val);
        // result.push_back((start_x + end_x) / 2);
      }
      start_x = x;
      end_x = x;
      skip = MIN_TRACK_WIDTH;
    }
  }

  if (start_x != 0 && end_x != 0)
  {
    ret_val.x = (start_x + end_x) / 2;
    ret_val.y = height;
    result.push_back(ret_val);
    // result.push_back((start_x + end_x) / 2);
  }

  return result;
}

int mTrackMovement::closest_white(std::vector<mTrackMovement::RowPoint> lines, int target_x)
{
  int min_distance = INT_MAX;
  int ret_idx = -1;
  for (unsigned long i = 0; i < lines.size(); i++)
  {
    int distance = abs(lines[i].x - target_x);
    if (distance < min_distance)
    {
      min_distance = distance;
      ret_idx = i;
    }
  }
  return ret_idx;
}

std::vector<mTrackMovement::RowPoint> mTrackMovement::slicing(std::vector<mTrackMovement::RowPoint>& arr,
                                                              int X, int Y)
{
  // Starting and Ending iterators
  auto start = arr.begin() + X;
  auto end = arr.begin() + Y + 1;

  // To store the sliced vector
  std::vector<mTrackMovement::RowPoint> result(Y - X + 1);

  // Copy vector using copy function()
  copy(start, end, result.begin());

  // Return the final sliced vector
  return result;
}

std::vector<std::vector<std::array<int, 2>>> mTrackMovement::detect_buckets(int height, int width, uchar* grid, uchar* red_grid)
{
  std::vector<std::vector<std::array<int, 2>>> result = std::vector<std::vector<std::array<int, 2>>>();

  int MAX_DISTANCE = bucket_width_param.Get();

  for (int y = 0; y < height; y++)
  {
    uchar* row = &(y * width)[grid];          // Deranged Programming
    uchar* red_row = &(y * width)[red_grid];  // Deranged Programming

    std::vector<mTrackMovement::RowPoint> lines = detect_lines(row, width, y, track_width_param.Get());
    std::vector<mTrackMovement::RowPoint> red_lines = detect_lines(red_row, width, y, red_track_width_param.Get());

    int count_red_lines = red_lines.size();

    if (count_red_lines >= 2)
    {
      int x1 = red_lines[0].x;
      int x2 = red_lines[count_red_lines - 1].x;

      int x1_idx_white = closest_white(lines, x1);
      int x2_idx_white = closest_white(lines, x2);

      if (x1_idx_white >= 0 && x2_idx_white >= 0)
      {
        lines = slicing(lines, x1_idx_white, x2_idx_white);
      }
    }
    else if (count_red_lines == 1)
    {
      int x = red_lines[0].x;
      int x_idx_white = closest_white(lines, x);

      if (x_idx_white >= 0)
      {
        float dif = x - prev_target_value;
        bool right = dif > 0;
        if (right)
        {
          // lines = std::vector<mTrackMovement::RowPoint>(&lines[0], &lines[x_idx_white]);
          lines = slicing(lines, 0, x_idx_white);
        }
        else
        {
          lines = slicing(lines, x_idx_white, lines.size() - 1);
          //lines = std::vector<mTrackMovement::RowPoint>(&lines[x_idx_white], &lines[lines.size()-1]);
        }
      }
      //lines.insert(lines.end(), red_lines.begin(), red_lines.end());
    }
    else
    {
      //lines.clear();
    }

    for (unsigned long i = 0; i < lines.size(); i++)
    {
      int x = lines[i].x;

      int index = 0;
      unsigned long max_size = 0;
      bool found = false;
      // for all buckets
      for (unsigned long j = 0; j < result.size(); j++)
      {
        auto bucket = result[j];
        if (abs(x - bucket[bucket.size() - 1][0]) < MAX_DISTANCE)
        {
          if (bucket.size() > max_size)
          {
            max_size = bucket.size();
            index = j;
            found = true;
          }
        }
      }

      if (found)
      {
        std::array<int, 2> arr = {x, y};
        result[index].push_back(arr);
      }
      else
      {
        std::array<int, 2> arr = {x, y};
        std::vector<std::array<int, 2>> outer_arr = std::vector<std::array<int, 2>>();
        outer_arr.push_back(arr);
        result.push_back(outer_arr);
      }
    }
  }

  return result;
}

void mTrackMovement::Control()
{
  select_image.Publish(lane_right.Get());

  auto image_track_rec = input_image_track_rec.GetPointer();
  cv::Mat image_track_rec_mat = rrlib::coviroa::AccessImageAsMat(const_cast<rrlib::coviroa::tImage&>(*image_track_rec)); 

  auto red_mask = in_red_mask.GetPointer();
  cv::Mat red_mask_mat = rrlib::coviroa::AccessImageAsMat(const_cast<rrlib::coviroa::tImage&>(*red_mask));

  int width = image_track_rec_mat.cols;
  int height = image_track_rec_mat.rows;

  auto buckets = detect_buckets(height, width, image_track_rec_mat.data, red_mask_mat.data);

  std::vector<std::array<unsigned long, 2>> sizes;
  for (unsigned long i = 0; i < buckets.size(); i++)
  {
    auto bucket = buckets[i];
    unsigned long size = bucket.size();
    std::array<unsigned long, 2> arr = {i, size};
    sizes.push_back(arr);
  }

  std::sort(sizes.begin(), sizes.end(), comp);

  int count = 0;
  for (unsigned long i = 0; i < sizes.size() && i < 3; i++)
  {
    auto bucket_size = sizes[i][1];
    if (static_cast<int>(bucket_size) < min_bucket_size_param.Get())
    {
      break;
    }
    count += 1;
  }

  std::vector<std::array<int, 2>> left_lane;
  std::vector<std::array<int, 2>> right_lane;

  bool found = false;

  int prev_lanes_seen_tmp = 0;

  if (count == 3)
  {
    
    wrong_lane = false;
    prev_lanes_seen_tmp = 3;
    auto relevant_buckets = std::vector<std::vector<std::array<int, 2>>>();
    for (int i = 0; i < 3; i++)
    {
      relevant_buckets.push_back(buckets[sizes[i][0]]);
    }
    std::sort(relevant_buckets.begin(), relevant_buckets.end(), comp2);

    if (lane_right.Get())
    {
      right_lane = relevant_buckets[1];
      left_lane = relevant_buckets[0];
    }
    else
    {
      left_lane = relevant_buckets[1];
      right_lane = relevant_buckets[2];
    }
    found = true;
  }
  else if (count == 2)
  {
    
    prev_lanes_seen_tmp = 2;
    auto relevant_buckets = std::vector<std::vector<std::array<int, 2>>>();
    for (int i = 0; i < 2; i++)
    {
      relevant_buckets.push_back(buckets[sizes[i][0]]);
    }
    std::sort(relevant_buckets.begin(), relevant_buckets.end(), comp2);

    left_lane = relevant_buckets[0];
    right_lane = relevant_buckets[1];
    found = true;
  }

  auto target_x = 0;
  auto target_y = 0;
  bool point_found = false;

  if (found)
  {
    int i = left_lane.size() - 1;
    int j = right_lane.size() - 1;

    while (i > 0 && j > 0)
    {
      std::array<int, 2> left = left_lane[i];
      std::array<int, 2> right = right_lane[j];

      if (left[1] > height - y_offset_param.Get())
      {
        i -= 1;
        continue;
      }

      if (left[1] == right[1])
      {
        int x = (right[0] + left[0])/2;
        // int x;
        // if (lane_right.Get()) {
        //   x = left[0] + p_offset_to_mid.Get();
        // } else {
        //   x = right[0] - p_offset_to_mid.Get();
        // }
        target_x = x;
        if (prev_lanes_seen == 3)
        {
          if (prev_lanes_seen_tmp != prev_lanes_seen) //  && three_lanes_continoues > 3
          {
            bool is_closer_to_right = abs(x - prev_target_right) < abs(x - prev_target_left);
            wrong_lane = (is_closer_to_right && !lane_right.Get()) || (!is_closer_to_right && lane_right.Get());
          }
          track_width = abs(right[0] - left[0]);
          if (lane_right.Get())
          {
            prev_target_right = x;
            prev_target_left = x - track_width * 1.2;
          }
          else
          {
            prev_target_left = x;
            prev_target_right = x + track_width * 1.2;
          }
        }

        target_y = left[1];
        point_found = true;
        break;
      }
      else if (left[1] < right[1])
      {
        j -= 1;
      }
      else
      {
        i -= 1;
      }
    }
  }

  prev_lanes_seen = prev_lanes_seen_tmp;
  if (prev_lanes_seen == 3)
  {
    three_lanes_continoues += 1;
  }
  else
  {
    three_lanes_continoues = 0;
  }


  if (point_found)
  {
    cv::circle(image_track_rec_mat, cv::Point(target_x, target_y), 6, cv::Scalar(255), 2);
    float mid = width / 2;

    float offset = lane_right.Get() ? camera_offset_param_right.Get() : -camera_offset_param_left.Get();

    mid = mid + offset;

    target_x += (wrong_lane ? track_width * 1.2 : 0) * (lane_right.Get() ? 1 : -1);
    
    float center_offset = lane_right.Get() ? -track_width/2 : track_width/2;

    prev_target_value = target_x + center_offset;
    if (wrong_lane)
    {
      cv::circle(image_track_rec_mat, cv::Point(target_x, target_y), 8, cv::Scalar(255), 3);
    }

    float x_diff = target_x - mid;

    float curvature_value = x_diff / width;

    curvature_value = curvature_value / (target_y + curvature_y_offset_param.Get());

    curvature.Publish(curvature_value * curvature_mult_param.Get());

  }
  co_point_found.Publish(point_found);
}

void mTrackMovement::ToggleLane()
{
  lane_right.Set(!lane_right.Get());
}

}  // namespace finroc::robprak_2024_2