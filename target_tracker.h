#pragma once

#include <memory>
#include <calibu/cam/camera_xml.h>
#include <calibu/conics/ConicFinder.h>
#include <calibu/image/ImageProcessing.h>
#include <calibu/pose/Pnp.h>
#include <calibu/target/TargetGridDot.h>
#include <opencv2/opencv.hpp>

namespace calibu_tracker
{

struct TrackStatus
{
  typedef Eigen::aligned_allocator<Eigen::Vector2d> Vector2dAllocator;

  bool target_found;

  double tracking_error;

  Sophus::SE3d T_target_camera;

  std::vector<int> ellipse_map;

  std::vector<Eigen::Vector2d, Vector2dAllocator> ellipses;
};

class TargetTracker
{
  public:

    TargetTracker(std::shared_ptr<calibu::TargetGridDot> target,
        std::shared_ptr<calibu::CameraInterfaced> projection);

    std::shared_ptr<calibu::TargetGridDot> target() const;

    std::shared_ptr<calibu::CameraInterfaced> projection() const;

    void Track(const cv::Mat& image, TrackStatus& status);

  protected:

    void PreprocessImage(const cv::Mat& image);

    void DetectTarget(TrackStatus& status);

    void GetConicCenters(TrackStatus& status);

    void EstimatePose(TrackStatus& status);

    void ComputeError(TrackStatus& status);

  private:

    void Initialize();

    void CreateImageProcessing();

    void CreateConicFinder();

  protected:

    std::shared_ptr<calibu::TargetGridDot> target_;

    std::shared_ptr<calibu::CameraInterfaced> projection_;

    std::unique_ptr<calibu::ImageProcessing> image_processing_;

    std::unique_ptr<calibu::ConicFinder> conic_finder_;
};

} // namespace calibu_tracker