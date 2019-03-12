#include "target_tracker.h"
#include <iostream>

namespace calibu_tracker
{

TargetTracker::TargetTracker(std::shared_ptr<calibu::TargetGridDot> target,
    std::shared_ptr<calibu::CameraInterfaced> projection) :
  target_(target),
  projection_(projection)
{
  Initialize();
}

std::shared_ptr<calibu::TargetGridDot> TargetTracker::target() const
{
  return target_;
}

std::shared_ptr<calibu::CameraInterfaced> TargetTracker::projection() const
{
  return projection_;
}

void TargetTracker::Track(const cv::Mat& image, TrackStatus& status)
{
  PreprocessImage(image);
  DetectTarget(status);

  if (status.target_found)
  {
    GetConicCenters(status);
    EstimatePose(status);
    ComputeError(status);
  }
}

void TargetTracker::PreprocessImage(const cv::Mat& image)
{
  image_processing_->Process(image.data, image.cols, image.rows, image.cols);
}

void TargetTracker::DetectTarget(TrackStatus& status)
{
  conic_finder_->Find(*image_processing_);

  status.target_found = target_->FindTarget(*image_processing_,
      conic_finder_->Conics(), status.ellipse_map);
}

void TargetTracker::GetConicCenters(TrackStatus& status)
{
  const auto& conics = conic_finder_->Conics();
  const size_t conic_count = conics.size();
  status.ellipses.resize(conic_count);

  for (size_t i = 0; i < conic_count; ++i)
  {
    status.ellipses[i] = conics[i].center;
  }
}

void TargetTracker::EstimatePose(TrackStatus& status)
{
  Sophus::SE3d T_camera_target;

  calibu::PosePnPRansac(projection_, status.ellipses, target_->Circles3D(),
      status.ellipse_map, 0, 0, &T_camera_target);

  status.T_target_camera = T_camera_target.inverse();
}

void TargetTracker::ComputeError(TrackStatus& status)
{
  status.tracking_error = calibu::ReprojectionErrorRMS(projection_,
      status.T_target_camera.inverse(), target_->Circles3D(), status.ellipses,
      status.ellipse_map);
}

void TargetTracker::Initialize()
{
  CreateImageProcessing();
  CreateConicFinder();
}

void TargetTracker::CreateImageProcessing()
{
  const int w = projection_->Width();
  const int h = projection_->Height();
  image_processing_ = std::make_unique<calibu::ImageProcessing>(w, h);
  image_processing_->Params().black_on_white = true;
  image_processing_->Params().at_window_ratio = 30.0;
  image_processing_->Params().at_threshold = 0.9;
}

void TargetTracker::CreateConicFinder()
{
  conic_finder_ = std::make_unique<calibu::ConicFinder>();
  conic_finder_->Params().conic_min_density = 0.6;
  conic_finder_->Params().conic_min_aspect = 0.2;
  conic_finder_->Params().conic_min_area = 4.0;
}

} // namespace calibu_tracker