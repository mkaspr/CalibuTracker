#include <iostream>
#include <calibu/target/TargetGridDot.h>
#include <calibu/target/GridDefinitions.h>
#include <calibu/cam/camera_xml.h>
#include "target_tracker.h"

using namespace calibu_tracker;

int main(int argc, char** argv)
{
  // read camera projection
  std::cout << "Reading camera projectdion file...." << std::endl;

  const std::string config_file = "cameras.xml";
  std::shared_ptr<calibu::CameraInterfaced> projection;;
  projection = calibu::ReadXmlRig(config_file)->cameras_[0];

  // create calibu grid
  std::cout << "Creating calibu grid..." << std::endl;

  Eigen::MatrixXi grid;
  double spacing = 0;
  double large_radius = 0;
  double small_radius = 0;
  const std::string preset = "medium";
  calibu::LoadGridFromPreset(preset, grid, spacing, large_radius, small_radius);

  // create calibu target
  std::cout << "Creating calibu target..." << std::endl;

  std::shared_ptr<calibu::TargetGridDot> target;
  target = std::make_shared<calibu::TargetGridDot>(spacing, grid);

  // create target tracker
  std::cout << "Creating target tracker..." << std::endl;

  std::shared_ptr<TargetTracker> tracker;
  tracker = std::make_shared<TargetTracker>(target, projection);

  // read images from file
  std::cout << "Reading images from file..." << std::endl;

  // create list of image filenames
  std::vector<std::string> filenames;
  filenames.emplace_back("image0.png");
  filenames.emplace_back("image1.png");
  filenames.emplace_back("image2.png");

  // allocate image list
  std::vector<cv::Mat> images;
  images.reserve(filenames.size());

  // read each image in filename list
  for (const std::string& filename : filenames)
  {
    images.emplace_back(cv::imread(filename, cv::IMREAD_GRAYSCALE));
  }

  // tracker target in images
  std::cout << "Tracking target in images..." << std::endl;
  std::cout << std::endl;

  // track each image in list
  for (size_t i = 0; i < images.size(); ++i)
  {
    // get reference to current image
    const cv::Mat& image = images[i];

    // track current image
    TrackStatus status;
    tracker->Track(image, status);

    // get tracked pose relative to target
    const Sophus::SE3d& pose = status.T_target_camera;
    const Eigen::Quaterniond& orientation = pose.unit_quaternion();
    const Eigen::Vector3d& position = pose.translation();

    // print tracking results
    std::cout << "Target found:       " << status.target_found << std::endl;
    std::cout << "Tracking error:     " << status.tracking_error << std::endl;
    std::cout << "Camera orientation: " << orientation.coeffs() << std::endl;
    std::cout << "Camera position:    " << position << std::endl;
    std::cout << std::endl;

    // create result image
    cv::Mat result;
    cv::cvtColor(image, result, CV_GRAY2RGB);

    // initialize box
    cv::Rect box;
    box.width = 10;
    box.height = 10;

    // draw box for each detected conics
    for (Eigen::Vector2d& center : status.ellipses)
    {
      box.x = center[0] - 5;
      box.y = center[1] - 5;
      cv::rectangle(result, box, cv::Scalar(0, 255, 0), 1);
    }

    // save resulting image to file
    const std::string filename = "result_" + filenames[i];
    cv::imwrite(filename, result);
  }

  std::cout << "Success." << std::endl;
  return 0;
}