#pragma once

#include <opencv2/opencv.hpp>

#include <VoxelVolume.h>


class SphereVolume : public VoxelVolume {
public:
  SphereVolume(size_t IDimension, size_t JDimension, size_t KDimension,
               const Coordinate &center, const PixelSpacing& spacing, float radius, float ball_value);
  ~SphereVolume() = default;

  float p(float x, float y, float z) override;

private:
  cv::Matx44f IDXtoXYZ_;
  cv::Matx44f XYZtoIDX_;
  float radius_{};
  // density
  float ball_value_{};
};
