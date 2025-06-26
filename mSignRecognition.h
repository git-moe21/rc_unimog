#ifndef __projects__robprak_2024_2__mSignRecognition_h__
#define __projects__robprak_2024_2__mSignRecognition_h__

#include "plugins/structure/tSenseControlModule.h"
#include "rrlib/coviroa/tImage.h"

namespace finroc
{
namespace robprak_2024_2
{

/*!
 * " This module is used to detect the different traffic signs, e.g. stop signs"
 */
class mSignRecognition : public structure::tSenseControlModule
{
public:
  tSensorInput<rrlib::coviroa::tImages> zed_camera_images; // unimog
  tSensorInput<rrlib::coviroa::tImage> zed_camera_solo_image; // unreal

  tSensorOutput<int> out_sign;
  tSensorOutput<rrlib::coviroa::tImage> out_image;

  tParameter<float> left_boundary;
  tParameter<float> right_boundary;
  tParameter<float> bot_crop;
  tParameter<float> top_crop;
  tParameter<bool> right_image;
  tParameter<double> eps_small;
  tParameter<double> eps_large;

  tParameter<float> ignore_contour;
  tParameter<bool> enable_sign_rec;

  tParameter<float> param_1;
  tParameter<float> param_2;
  tParameter<float> param_3;
  tParameter<float> param_4;

  tParameter<float> brightness;
  tParameter<int> kernel_size;
  tParameter<int> dilationIterations;
  tParameter<int> erosionIterations;
  


public:
  mSignRecognition(core::tFrameworkElement *parent, const std::string &name = "SignRecognition");


protected:
  virtual ~mSignRecognition();

private:
  virtual void OnParameterChange() override;

  virtual void Sense() override;

  virtual void Control() override; 

};

}  // namespace robprak_2024_2
}  // namespace finroc

#endif
