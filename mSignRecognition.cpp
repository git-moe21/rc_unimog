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
#include "projects/robprak_2024_2/mSignRecognition.h"
#include "rrlib/coviroa/opencv_utils.h"
#include <opencv2/opencv.hpp>
#include <cassert>


namespace finroc::robprak_2024_2
{
mSignRecognition::~mSignRecognition()
{
}

#ifdef _LIB_FINROC_PLUGINS_RUNTIME_CONSTRUCTION_ACTIONS_PRESENT_
static const runtime_construction::tStandardCreateModuleAction<mSignRecognition> cCREATE_ACTION_FOR_M_SIGNRECOGNITION("SignRecognition");
#endif


mSignRecognition::mSignRecognition(core::tFrameworkElement *parent, const std::string &name) :
  tSenseControlModule(parent, name, true),
  zed_camera_images("ZED Cam Image", this),
  zed_camera_solo_image("ZED Cam Single Image", this),
  out_sign("Output Sign Number", this, 0),
  out_image("Output Image", this),

  left_boundary("Left boundary", this, 0.42),  
  right_boundary("Right boundary", this, 0.3),
  bot_crop("Crop factor bot", this, 0.05),
  top_crop("Crop factor top", this, 0.3),
  right_image("Right Image", this, false),

  eps_small("Epsilon small", this, 0.02),
  eps_large("Epsilon large", this, 0.14),
  ignore_contour("Ignore Contours <", this, 1000),

  enable_sign_rec("Enable Sign Recognition", this, true),

  param_1("Param 1", this, 130),
  param_2("Param 2", this, 90),
  param_3("Param 3", this, 135),
  param_4("Param 4", this, 56),

  brightness("Brightness", this, 0.8),
  kernel_size("Kernel Size", this, 19),
  dilationIterations("Dilate Iterations", this, 3),
  erosionIterations("Erosion Iterations", this, 3)

{
}



void mSignRecognition::OnParameterChange()
{
}


void mSignRecognition::Sense()
{
}


void mSignRecognition::Control()
{
  if (enable_sign_rec.Get()) {
     // fuer Unimog
    cv::Mat solo_image_mat;
    data_ports::tPortDataPointer<const std::vector<rrlib::coviroa::tImage>> image = this->zed_camera_images.GetPointer();

    if(image->size() > 0){ 
      if (right_image.Get()) {
        solo_image_mat = rrlib::coviroa::AccessImageAsMat((*image)[1]); // right image
      } else {
        solo_image_mat = rrlib::coviroa::AccessImageAsMat((*image)[0]); // left image
      }
    }  
      

    //auto solo_image = zed_camera_solo_image.GetPointer();

    if (solo_image_mat.empty())  // fuer unimog: solo_image_mat.empty(), unreal: !solo_image
    {
      return; // Falls zed instanz noch nicht geladen ist
    }

    //cv::Mat solo_image_mat = rrlib::coviroa::AccessImageAsMat(const_cast<rrlib::coviroa::tImage &>(*solo_image));

    cv::Mat bright_image, yellow_bright;
    solo_image_mat.convertTo(bright_image, -1, brightness.Get(), 0);
    solo_image_mat.convertTo(yellow_bright, -1, brightness.Get() * 1.5, 0);

    // Zuschneiden
    int top_crop_value = static_cast<int>(bright_image.rows * top_crop.Get());
    int bottom_crop_value = static_cast<int>(bright_image.rows * bot_crop.Get());
    int cropHeight = bright_image.rows - top_crop_value - bottom_crop_value;

    cv::Rect roi(0, top_crop_value, bright_image.cols, cropHeight);
    cv::Mat cropped = bright_image(roi);

    // Zuschneiden
    int top_crop_value_y = static_cast<int>(yellow_bright.rows * top_crop.Get());
    int bottom_crop_value_y = static_cast<int>(yellow_bright.rows * bot_crop.Get());
    int cropHeight_y = yellow_bright.rows - top_crop_value_y - bottom_crop_value_y;

    cv::Rect roi_y(0, top_crop_value_y, yellow_bright.cols, cropHeight_y);
    cv::Mat cropped_y = bright_image(roi_y);

    

    int leftBoundary = static_cast<int>(left_boundary.Get() * cropped.cols); 
    int rightBoundary = static_cast<int>((1 - right_boundary.Get()) * cropped.cols);

    cv::rectangle(cropped, cv::Rect(0, 0, leftBoundary, cropped.rows), cv::Scalar(0, 255, 255), 2);
    cv::rectangle(cropped, cv::Rect(rightBoundary, 0, cropped.cols - rightBoundary, cropped.rows), cv::Scalar(0, 255, 255), 2);




    // Brown Bridge Mask
    cv::Mat hsv_image, brown_mask, hsv_image_y;
    cv::Scalar lower_brown(10, 40, 40);
    cv::Scalar upper_brown(30, 255, 255); 
    cv::cvtColor(cropped, hsv_image, cv::COLOR_BGR2HSV);
    cv::cvtColor(cropped_y, hsv_image_y, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_image, lower_brown, upper_brown, brown_mask);


    cv::Mat kernel_brown = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::morphologyEx(brown_mask, brown_mask, cv::MORPH_OPEN, kernel_brown);


    // Rot Maske
    cv::Mat red_mask, red_mask1, red_mask2;
    cv::Scalar lower_red1(0, param_1.Get(), param_2.Get());
    cv::Scalar upper_red1(10, 255, 255);
    cv::Scalar lower_red2(170, param_3.Get(), param_4.Get());
    cv::Scalar upper_red2(180, 255, 255);

    cv::inRange(hsv_image, lower_red1, upper_red1, red_mask1);
    cv::inRange(hsv_image, lower_red2, upper_red2, red_mask2);
    red_mask = red_mask1 | red_mask2;


    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size.Get(), kernel_size.Get()));

    // Mehrere Dilatations-Durchgänge: Erweitert die weißen Bereiche
    cv::dilate(red_mask, red_mask, kernel, cv::Point(-1, -1), dilationIterations.Get());

    // Mehrere Erosions-Durchgänge: Schrumpft die weißen Bereiche
    cv::erode(red_mask, red_mask, kernel, cv::Point(-1, -1), erosionIterations.Get());

    //cv::imshow("Red Mask", red_mask);
    //cv::waitKey(1);


    cv::Mat not_brown, red_mask_filtered;
    cv::bitwise_not(brown_mask, not_brown);        // Invertiere die braune Maske
    cv::bitwise_and(red_mask, not_brown, red_mask_filtered);

    //cv::imshow("Red Mask filtered", red_mask_filtered);
    //cv::waitKey(1);


    // Gelb Maske


    cv::Mat yellowMask;
    cv::Scalar lower_yellow(20, 100, 100);
    cv::Scalar upper_yellow(30, 255, 255);
    cv::inRange(hsv_image_y, lower_yellow, upper_yellow, yellowMask);

    //cv::Mat kernel_yellow = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size.Get(), kernel_size.Get()));
    //cv::morphologyEx(yellowMask, yellowMask, cv::MORPH_CLOSE, kernel_yellow);
    //cv::morphologyEx(yellowMask, yellowMask, cv::MORPH_OPEN, kernel_yellow);


    // Extends White pixels
    //cv::dilate(yellowMask, yellowMask, kernel, cv::Point(-1, -1), dilationIterations.Get());

    // Shrinks white pixels
    //cv::erode(yellowMask, yellowMask, kernel, cv::Point(-1, -1), erosionIterations.Get());


    //cv::imshow("yellow Mask", yellowMask);

    int detectedSign = 0; // 0 = kein Schild, 1 = Vorfahrts-/Vorfahrtachten-Schild, 2 = Stoppschild, 3 = Vorfahrtsstraßenschild

    // --- Find contours ---
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(red_mask_filtered, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>> YellowContours;
    cv::findContours(yellowMask, YellowContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    bool success_in_red = false;

    for (auto &contour : contours)
    {
      double area = cv::contourArea(contour);

      // min size
      if (area < ignore_contour.Get())
        continue;
      

      
      double perimeter = cv::arcLength(contour, true);

      std::vector<cv::Point> approx_small;
      double eps_small_temp = eps_small.Get() * perimeter;
      cv::approxPolyDP(contour, approx_small, eps_small_temp, true);


      std::vector<cv::Point> approx_large;
      double eps_large_temp = eps_large.Get() * perimeter;
      cv::approxPolyDP(contour, approx_large, eps_large_temp, true);


      cv::Rect bbox_small = cv::boundingRect(approx_small);
      bool isCompletelyLeft_small = (bbox_small.x >= 0 && (bbox_small.x + bbox_small.width) <= leftBoundary);
      bool isCompletelyRight_small = (bbox_small.x >= rightBoundary && (bbox_small.x + bbox_small.width) <= cropped.cols);
      bool boundary_ok_small = isCompletelyLeft_small || isCompletelyRight_small;

      cv::Rect bbox_large = cv::boundingRect(approx_large);
      bool isCompletelyLeft_large = (bbox_large.x >= 0 && (bbox_large.x + bbox_large.width) <= leftBoundary);
      bool isCompletelyRight_large = (bbox_large.x >= rightBoundary && (bbox_large.x + bbox_large.width) <= cropped.cols);
      bool boundary_ok_large = isCompletelyLeft_large || isCompletelyRight_large;


      if (!boundary_ok_small || !boundary_ok_large)
      {
        continue;
      }

      
      // filter out driving road edge
      cv::Rect bbox = cv::boundingRect(contour);
      float aspect_ratio = static_cast<float>(bbox.width) / static_cast<float>(bbox.height);

      if (aspect_ratio < 0.7f || aspect_ratio > 2.0f)  // driving road edge is elongated
        continue;
      
    
      //cv::drawContours(cropped, std::vector<std::vector<cv::Point>>{approx_small}, -1, cv::Scalar(255, 0, 0), 2);
      //std::cout << "Drawing Contour with size " << cv::contourArea(contour) << approx_small.size() << std::endl;


      bool is_stop = (approx_small.size() >= 7 && approx_small.size() <= 9);
      bool is_yield = (approx_large.size() == 3);


      if (is_stop)
      { 
        success_in_red = true;
        //std::cout << "Area: " << area << std::endl;
        //std::cout << "Aspect: " << aspect_ratio << std::endl;
        detectedSign = 2; // Stop
        cv::drawContours(cropped, std::vector<std::vector<cv::Point>>{approx_small}, 
                         -1, cv::Scalar(255, 0, 0), 2);
                        
      }
      else if (is_yield)
      {
        success_in_red = true;
        //std::cout << "Area: " << area << std::endl;
        //std::cout << "Aspect: " << aspect_ratio << std::endl;
        detectedSign = 1; // Vorfahrt achten
        cv::drawContours(cropped, std::vector<std::vector<cv::Point>>{approx_large}, 
                         -1, cv::Scalar(0, 255, 0), 2);
      }
      //std::cout << success_in_red << std::endl;
    }   

    if (!success_in_red) {
      //std::cout << "Searching yellow";
      for (auto& contour : YellowContours){
        double area = cv::contourArea(contour);

        // min size
        if (area < ignore_contour.Get() * 0.5)
          continue;
        
        double perimeter = cv::arcLength(contour, true);

        std::vector<cv::Point> approx_large;
        double eps_large_temp = eps_large.Get() * perimeter;
        cv::approxPolyDP(contour, approx_large, eps_large_temp, true);

        cv::Rect bbox_large = cv::boundingRect(approx_large);
        bool isCompletelyLeft_large = (bbox_large.x >= 0 && (bbox_large.x + bbox_large.width) <= leftBoundary);
        bool isCompletelyRight_large = (bbox_large.x >= rightBoundary && (bbox_large.x + bbox_large.width) <= cropped.cols);
        bool boundary_ok_large = isCompletelyLeft_large || isCompletelyRight_large;

        //std::cout << "Area: " << area << ", " << approx_large.size() << std::endl;
        if (!boundary_ok_large)
        {
          continue;
        }
        cv::Rect bbox = cv::boundingRect(contour);
        float aspect_ratio = static_cast<float>(bbox.width) / static_cast<float>(bbox.height);

        if (approx_large.size() <= 5 && approx_large.size() >= 4){

          //std::cout << "Aspect: " << aspect_ratio << std::endl;
          detectedSign = 3; // Vorfahrtsstraße
          cv::drawContours(cropped, std::vector<std::vector<cv::Point>>{approx_large}, 
                           -1, cv::Scalar(0, 0, 255), 2);
        }
      } 
    }


    //cv::imshow("Classification", cropped);
    cv::waitKey(1);
    std::cout << "Detected Sign: " << detectedSign << std::endl;
    //std::cout << "---------" << std::endl;

    out_sign.Publish(detectedSign);

    /*
    // Output Port
    rrlib::coviroa::tImageFormat format = rrlib::coviroa::tImageFormat::eIMAGE_FORMAT_MONO8;

    rrlib::coviroa::tImage out_tImage(cropped.cols, cropped.rows, format);
    cropped.copyTo(AccessImageAsMat(out_tImage));

    data_ports::tPortDataPointer<rrlib::coviroa::tImage> img_out = this->out_image.GetUnusedBuffer();
    rrlib::rtti::GenericOperations<rrlib::coviroa::tImage>::DeepCopy(out_tImage, *img_out);
    this->out_image.Publish(img_out);
    */
  }
}
}

