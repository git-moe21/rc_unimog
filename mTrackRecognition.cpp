#include <cassert>
#include <algorithm>
#include "projects/robprak_2024_2/mTrackRecognition.h"
#include "rrlib/coviroa/tImage.h"
#include "rrlib/coviroa/opencv_utils.h"
#include <opencv2/opencv.hpp>

namespace finroc::robprak_2024_2
{
//----------------------------------------------------------------------

mTrackRecognition::~mTrackRecognition()
{
}

#ifdef _LIB_FINROC_PLUGINS_RUNTIME_CONSTRUCTION_ACTIONS_PRESENT_
static const runtime_construction::tStandardCreateModuleAction<mTrackRecognition> cCREATE_ACTION_FOR_M_TRACKRECOGNITION("TrackRecognition");
#endif

mTrackRecognition::mTrackRecognition(core::tFrameworkElement *parent, const std::string &name) :
  tSenseControlModule(parent, name, false),
  zed_camera_images("ZED Cam Image", this),
  zed_camera_solo_image("ZED Cam Single Image", this),
  out_image("Classified Output Image", this),
  out_red_mask("Output Red Mask Image", this),
  right_camera("Use right camera image (Unimog)", this, true),
  search_bot_y("Search start bot index", this, 140),
  search_min_width("Search min. width", this, 320),
  search_left_cap("Search left min. distance", this, 210),  // bis zu dieser distanz wird es noch max. als linke linie erkannt
  search_border_offset("Search border offset", this, 10),
  search_top_y("Search start top index", this, 100),
  dynamic_mask("Dynamic Mask", this, false),
  scaleFactor("Scaling factor for output", this, 0.5),
  param_1("Red Mask Width Param 1", this, 4),
  param_2("Red Mask Width Param 2", this, 10)
{
}

void mTrackRecognition::OnParameterChange()
{
}

void mTrackRecognition::Sense()
{
}

void mTrackRecognition::Control()
{
  // fuer Unimog
  cv::Mat solo_image_mat;
  data_ports::tPortDataPointer<const std::vector<rrlib::coviroa::tImage>> image = this->zed_camera_images.GetPointer();

  if (image->size() > 0)
  {  // currently right image
    if (right_camera.Get())
    {
      solo_image_mat = rrlib::coviroa::AccessImageAsMat((*image)[1]);
    }
    else
    {
      solo_image_mat = rrlib::coviroa::AccessImageAsMat((*image)[0]);
    }
  }

  // auto solo_image = zed_camera_solo_image.GetPointer();

  if (solo_image_mat.empty())  // fuer unimog: solo_image_mat.empty(), unreal: !solo_image
  {
    return;  // Daten sind noch nicht verfügbar
  }

  // cv::Mat solo_image_mat = rrlib::coviroa::AccessImageAsMat(const_cast<rrlib::coviroa::tImage &>(*solo_image));

  // --------------------- View Transformation ---------------------
  cv::Mat bird_view_image;
  std::vector<cv::Point2f> src_pts = {
    cv::Point2f(100, solo_image_mat.rows),
    cv::Point2f(solo_image_mat.cols - 100, solo_image_mat.rows),
    cv::Point2f(0, solo_image_mat.rows / 2),
    cv::Point2f(solo_image_mat.cols, solo_image_mat.rows / 2)};
  std::vector<cv::Point2f> dst_pts = {
    cv::Point2f(0, solo_image_mat.rows),
    cv::Point2f(solo_image_mat.cols, solo_image_mat.rows),
    cv::Point2f(0, 0),
    cv::Point2f(solo_image_mat.cols, 0)};

  cv::Mat perspective_matrix = cv::getPerspectiveTransform(src_pts, dst_pts);
  cv::warpPerspective(solo_image_mat, bird_view_image, perspective_matrix, solo_image_mat.size());

  // --------------------- Red Mask ---------------------
  cv::Mat red_mask, red_hsv, red_mask1, red_mask2;
  cv::cvtColor(bird_view_image, red_hsv, cv::COLOR_BGR2HSV);

  cv::Scalar lower_red1(0, 74, 50);
  cv::Scalar upper_red1(10, 255, 255);
  cv::Scalar lower_red2(170, 70, 50);
  cv::Scalar upper_red2(180, 255, 255);

  cv::inRange(red_hsv, lower_red1, upper_red1, red_mask1);
  cv::inRange(red_hsv, lower_red2, upper_red2, red_mask2);
  red_mask = red_mask1 | red_mask2;
  

  cv::Mat kernel_red_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel_red_close);

  cv::Mat kernel_red_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel_red_open);

  cv::Mat kernel_red_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(param_1.Get(), param_1.Get()));
  cv::dilate(red_mask, red_mask, kernel_red_dilate, cv::Point(-1, -1), param_2.Get());

  cv::Mat kernel_red_close2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
  cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel_red_close2);


  // cv::imshow("Red Mask", red_mask);
  // cv::waitKey(1);


  cv::Mat region_mask = cv::Mat::zeros(red_mask.size(), CV_8UC1);
  std::vector<cv::Point> mask_points;

  if (dynamic_mask.Get())
  {
    // --------------------- Suche nach unteren äußersten roten Punkten ---------------------
    int y_start = red_mask.rows - search_bot_y.Get();
    int leftmost_x_bottom = -1, rightmost_x_bottom = -1;

    for (int x = 0; x < red_mask.cols; x++)
    {
      for (int y = y_start; y < red_mask.rows; y++)
      {
        if (red_mask.at<uchar>(y, x) > 0)
        {
          if (leftmost_x_bottom == -1 && x <= search_left_cap.Get())
            leftmost_x_bottom = std::max(0, x - search_border_offset.Get());

          if (x - leftmost_x_bottom >= search_min_width.Get())
          {
            rightmost_x_bottom = std::min(red_mask.cols - 1, x + search_border_offset.Get());
            break;
          }
        }
      }
    }

    if (leftmost_x_bottom == -1)
    {
      leftmost_x_bottom = 0;
    }

    if (rightmost_x_bottom == -1)
    {
      rightmost_x_bottom = red_mask.cols - 1;
    }

    // std::cout << "Bottom: " << leftmost_x_bottom << ", " << rightmost_x_bottom << std::endl;

    // --------------------- Suche nach oberen äußersten roten Punkten ---------------------
    int leftmost_x_top = -1, rightmost_x_top = -1;

    for (int x = 0; x < red_mask.cols; x++)
    {
      for (int y = search_top_y.Get(); y < y_start; y++)
      {
        if (red_mask.at<uchar>(y, x) > 0)
        {
          if (leftmost_x_top == -1 && x <= search_left_cap.Get())
            leftmost_x_top = std::max(0, x - search_border_offset.Get());

          if (x - leftmost_x_top >= search_min_width.Get())
          {
            rightmost_x_top = std::min(red_mask.cols - 1, x + search_border_offset.Get());
            break;
          }
        }
      }
    }

    if (leftmost_x_top == -1)
    {
      leftmost_x_top = 0;
    }

    if (rightmost_x_top == -1)
    {
      rightmost_x_top = red_mask.cols - 1;
    }

    // std::cout << "Top: " << leftmost_x_top << ", " << rightmost_x_top << std::endl;

    mask_points = {
      cv::Point(leftmost_x_bottom, red_mask.rows),
      cv::Point(leftmost_x_top, 0),
      cv::Point(rightmost_x_top, 0),
      cv::Point(rightmost_x_bottom, red_mask.rows)};
  }
  else
  {
    mask_points = {
      cv::Point(0, red_mask.rows),
      cv::Point(0, 0),
      cv::Point(red_mask.cols, 0),
      cv::Point(red_mask.cols, red_mask.rows)};
  }

  cv::fillPoly(region_mask, std::vector<std::vector<cv::Point>>{mask_points}, cv::Scalar(255));

  // cv::imshow("Mask", region_mask);
  // cv::waitKey(1);

  cv::Mat masked_bird_view;
  bird_view_image.copyTo(masked_bird_view, region_mask);

  // --------------------- Canny ---------------------
  cv::Mat gray_image, blurred_image, edge_image;
  cv::cvtColor(masked_bird_view, gray_image, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray_image, blurred_image, cv::Size(3, 3), 0);
  cv::Canny(blurred_image, edge_image, 80, 120);

  cv::bitwise_and(edge_image, region_mask, edge_image);

  // cv::imshow("Canny", edge_image);

  cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(21, 1));
  cv::morphologyEx(edge_image, edge_image, cv::MORPH_CLOSE, kernel_close);

  // Dünne einzelne Linien herausfiltern
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::dilate(edge_image, edge_image, kernel, cv::Point(-1, -1), 1);  // (Dilatation), damit nahe Linien sich verbinden
  cv::erode(edge_image, edge_image, kernel, cv::Point(-1, -1), 2);   // (Erosion), um einzelne, schmale Linien zu entfernen

  // cv::imshow("Canny after transformation", edge_image);

  // --------------------- Hough ---------------------
  std::vector<cv::Vec4i> lines;
  cv::HoughLinesP(edge_image, lines, 0.5, CV_PI / 360, 8, 2, 26);

  const int min_line_length = 16;
  cv::Mat binary_output = cv::Mat::zeros(edge_image.size(), CV_8UC1);

  for (const auto &line : lines)
  {
    cv::Vec4i l = line;
    int line_length = cv::norm(cv::Point(l[0], l[1]) - cv::Point(l[2], l[3]));

    if (line_length >= min_line_length)
    {
      cv::line(binary_output,
               cv::Point(l[0], l[1]),
               cv::Point(l[2], l[3]),
               cv::Scalar(255), 3, cv::LINE_AA);
    }
  }

  // --------------------- Output Ports ---------------------
  rrlib::coviroa::tImageFormat format = rrlib::coviroa::tImageFormat::eIMAGE_FORMAT_MONO8;

  // Runterskalieren des binären Output-Bildes
  cv::Mat scaled_binary_output;
  cv::resize(binary_output, scaled_binary_output, cv::Size(), scaleFactor.Get(), scaleFactor.Get(), cv::INTER_AREA);

  // Runterskalieren der Red Mask
  cv::Mat scaled_red_mask;
  cv::resize(red_mask, scaled_red_mask, cv::Size(), scaleFactor.Get(), scaleFactor.Get(), cv::INTER_AREA);

  // Output classified abstract Image
  rrlib::coviroa::tImage out_tImage(scaled_binary_output.cols, scaled_binary_output.rows, format);
  scaled_binary_output.copyTo(AccessImageAsMat(out_tImage));
  data_ports::tPortDataPointer<rrlib::coviroa::tImage> img_out = this->out_image.GetUnusedBuffer();
  rrlib::rtti::GenericOperations<rrlib::coviroa::tImage>::DeepCopy(out_tImage, *img_out);
  out_image.Publish(img_out);

  // Output Red Mask
  rrlib::coviroa::tImage out_tImage_red(scaled_red_mask.cols, scaled_red_mask.rows, format);
  scaled_red_mask.copyTo(AccessImageAsMat(out_tImage_red));
  data_ports::tPortDataPointer<rrlib::coviroa::tImage> img_out_red = this->out_red_mask.GetUnusedBuffer();
  rrlib::rtti::GenericOperations<rrlib::coviroa::tImage>::DeepCopy(out_tImage_red, *img_out_red);
  out_red_mask.Publish(img_out_red);
}
}  // namespace finroc::robprak_2024_2
