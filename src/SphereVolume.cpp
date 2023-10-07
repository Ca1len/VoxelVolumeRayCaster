#include "VoxelVolume.h"
#include <SphereVolume.h>


SphereVolume::SphereVolume(size_t IDimension, size_t JDimension,
                           size_t KDimension, const Coordinate &ppCenter,
                           const PixelSpacing &spacing, float radius,
                           float ball_value)
    : VoxelVolume(IDimension, JDimension, KDimension), radius_(radius),
      ball_value_(ball_value) {
  pixelSpacing_ = spacing;
  parallelepipedCenter_ = ppCenter;
  cv::Matx44f scale = {spacing.sx, 0, 0,          0, 0, spacing.sy, 0, 0,
                       0,          0, spacing.sz, 0, 0, 0,          0, 1};
  cv::Matx44f translation = {1, 0, 0, ppCenter.x, 0, 1, 0, ppCenter.y,
                             0, 0, 1, ppCenter.z, 0, 0, 0, 1};
  IDXtoXYZ_ = translation * scale;
  XYZtoIDX_ = IDXtoXYZ_.inv();

  auto ijk_c = cv::Vec4f(IDimension, JDimension, KDimension, 2) / 2;
  auto xyz_c = IDXtoXYZ_ * ijk_c;

  for (size_t i = 0; i < parallelepiped_->shape_.nx; ++i) {
    for (size_t j = 0; j < parallelepiped_->shape_.ny; ++j) {
      for (size_t k = 0; k < parallelepiped_->shape_.nz; ++k) {
        auto temp = IDXtoXYZ_ * cv::Vec4f(i, j, k, 1);
        cv::Vec4f xyz{temp[0], temp[1], temp[2]};
        if (cv::norm(xyz - xyz_c) < radius_) {
          (*parallelepiped_)[{i, j, k}] = ball_value;
        }
      }
    }
  }
}

float SphereVolume::p(float x, float y, float z) {
  cv::Vec4f point{x, y, z, 1};
  cv::Vec4i idx = XYZtoIDX_ * point;
  if (!parallelepiped_->checkInShape(idx[0], idx[1], idx[2]))
    return 0;
  return (*parallelepiped_)[{static_cast<size_t>(idx[0]),
                             static_cast<size_t>(idx[1]),
                             static_cast<size_t>(idx[2])}];
}
