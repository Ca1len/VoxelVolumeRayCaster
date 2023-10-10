#include "VoxelVolume.h"
#include <Projection.h>
#include <cstdint>
#include <deque>
#include <mutex>
#include <new>
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/matx.hpp>

static uint8_t n_cores = std::thread::hardware_concurrency();

Projection::Projection(const Resolution &res, const Shape &shape)
    : resolution_(res), shape_(shape) {

  cv::Matx33f scale{
      (float)shape.rx / res.px, 0, 0, 0, (float)shape.ry / res.py, 0, 0, 0, 1};
  cv::Matx33f translation{1, 0, shape_.rx / 2.0f, 0, 1, shape_.ry / 2.0f, 0,
                          0, 1};
  IJtoXY = translation.inv() * scale;
  XYtoIJ = IJtoXY.inv();
}

static inline std::deque<Parallelepiped::Index>
getInterestVoxels(const VoxelVolume &volume, const Ray &ray) {
  std::deque<Parallelepiped::Index> res{};
  auto &pp = volume.parallelepiped_;
  auto &volume_shape = pp->shape_;
  auto &IDXtoXYZ = volume.getIJKtoXYZ();
  auto XYZtoIDX = IDXtoXYZ.inv();

  cv::Vec4f ijk_upper{(float)volume_shape.nx, (float)volume_shape.ny,
                      (float)volume_shape.nz, 1};
  cv::Vec4f xyz_upper = IDXtoXYZ * ijk_upper;
  cv::Vec4f ijk_lower{0, 0, 0, 1};
  cv::Vec4f xyz_lower = IDXtoXYZ * ijk_lower;

  Box zone_of_interest{{xyz_lower[0], xyz_lower[1], xyz_lower[2]},
                       {xyz_upper[0], xyz_upper[1], xyz_upper[2]}};

  auto segment = zone_of_interest.intersect(ray, 0, 1);
  cv::Vec3f xyz_s_t = ray.direction * segment.source + ray.origin;
  cv::Vec3f xyz_d_t = ray.direction * segment.dest + ray.origin;

  cv::Vec4f xyz_source(xyz_s_t[0], xyz_s_t[1], xyz_s_t[2], 1);

  cv::Vec4f xyz_dest(xyz_d_t[0], xyz_d_t[1], xyz_d_t[2], 1);

  auto ijk_s_t = XYZtoIDX * xyz_source;
  auto ijk_d_t = XYZtoIDX * xyz_dest;
  cv::Vec3f ijk_source{
      std::ceil((ijk_s_t[0] < ijk_d_t[0]) ? ijk_s_t[0] : ijk_d_t[0]),
      std::ceil((ijk_s_t[1] < ijk_d_t[1]) ? ijk_s_t[1] : ijk_d_t[1]),
      std::ceil((ijk_s_t[2] < ijk_d_t[2]) ? ijk_s_t[2] : ijk_d_t[2]),
  };
  cv::Vec3f ijk_dest{
      std::floor((ijk_s_t[0] > ijk_d_t[0]) ? ijk_s_t[0] : ijk_d_t[0]),
      std::floor((ijk_s_t[1] > ijk_d_t[1]) ? ijk_s_t[1] : ijk_d_t[1]),
      std::floor((ijk_s_t[2] > ijk_d_t[2]) ? ijk_s_t[2] : ijk_d_t[2]),
  };


  for (size_t i = ijk_source[0]; i < ijk_dest[0]; ++i) {
    for (size_t j = ijk_source[1]; j < ijk_dest[1]; ++j) {
      for (size_t k = ijk_source[2]; k < ijk_dest[2]; ++k) {
        Parallelepiped::Index idx{i, j, k};
        if ((*pp)[idx] != 0) {
          res.push_back(std::move(idx));
        }
      }
    }
  }
  return res;
}

static inline std::deque<Parallelepiped::Index>
getInterestVoxels(const VoxelVolume &volume) {
  std::deque<Parallelepiped::Index> res{};
  auto &pp = volume.parallelepiped_;
  auto &volume_shape = pp->shape_;

  for (size_t i = 0; i < volume_shape.nx; ++i) {
    for (size_t j = 0; j < volume_shape.ny; ++j) {
      for (size_t k = 0; k < volume_shape.nz; ++k) {
        Parallelepiped::Index idx{i, j, k};
        if ((*pp)[idx] != 0) {
          res.push_back(std::move(idx));
        }
      }
    }
  }
  return res;
}

static std::deque<Parallelepiped::Index> interestIndexes;

static inline float transparency_integral(const VoxelVolume &volume,
                                          const Ray &ray) {
  float integral{0};
  auto &pp = volume.parallelepiped_;
  auto &volume_shape = pp->shape_;
  auto &spacing = volume.pixelSpacing_;
  auto &IDXtoXYZ = volume.getIJKtoXYZ();
#if 0
  auto interestIndexes = getInterestVoxels(volume, ray);
#endif
  for (const auto &idx : interestIndexes) {
    cv::Vec4f ijk_point{(float)idx.i, (float)idx.j, (float)idx.k, 1};
    cv::Vec4f xyz_point = IDXtoXYZ * ijk_point;
    Box voxel{{xyz_point[0] - spacing.sx / 2, xyz_point[1] - spacing.sy / 2,
               xyz_point[2] - spacing.sz / 2},
              {xyz_point[0] + spacing.sx / 2, xyz_point[1] + spacing.sy / 2,
               xyz_point[2] + spacing.sz / 2}};
    auto segment = voxel.intersect(ray, 0, 1);
    float length = 0;
    if (segment) {
      length = cv::norm(ray.direction * segment.dest -
                        ray.direction * segment.source);
    }
    integral += volume.p(xyz_point[0], xyz_point[1], xyz_point[2]) * length;
  }
#if 0
  for (int i = 0; i < volume_shape.nx; ++i) {
    for (int j = 0; j < volume_shape.ny; ++j) {
      for (int k = 0; k < volume_shape.nz; ++k) {
        cv::Vec4f ijk_point{(float)i, (float)j, (float)k, 1};
        cv::Vec4f xyz_point = IDXtoXYZ * ijk_point;
        Box voxel{{xyz_point[0] - spacing.sx / 2, xyz_point[1] - spacing.sy / 2,
                   xyz_point[2] - spacing.sz / 2},
                  {xyz_point[0] + spacing.sx / 2, xyz_point[1] + spacing.sy / 2,
                   xyz_point[2] + spacing.sz / 2}};
        auto segment = voxel.intersect(ray, 0, 1);
        float length = 0;
        if (segment) {
          length = cv::norm(ray.direction * segment.dest -
                            ray.direction * segment.source);
        }
        integral += volume.p(xyz_point[0], xyz_point[1], xyz_point[2]) * length;
      }
    }
  }
#endif
  return integral;
}

cv::Mat Projection::compute(const VoxelVolume &volume, const Source &source) {
  cv::Mat img(cv::Mat::zeros(resolution_.py, resolution_.px, CV_32F));
  cv::Vec3f source_point{source.x_src, source.y_src, source.z_src};
  static std::mutex img_mutex;
  interestIndexes = getInterestVoxels(volume);
  auto foo = [&](float i, float j) {
    cv::Vec3f ij_point{i, j, 1};
    cv::Vec3f xyz_point = IJtoXY * ij_point;
    xyz_point[2] = 0;
    cv::Vec3f direction = xyz_point - source_point;
    Ray ray{source_point, direction};
    float value = transparency_integral(volume, ray);
    std::lock_guard<std::mutex> lock(img_mutex);
    img.at<float>(j, i) = value;
  };
  for (int i = 0; i < resolution_.px; ++i) {
    for (int j = 0; j < resolution_.py; ++j) {
      futures_.push_back(std::async(std::launch::async, foo, i, j));
      if (futures_.size() % n_cores == 0) {
        for (auto &future : futures_) {
          future.wait();
        }
        futures_.clear();
      }
    }
  }
  double min, max;
  cv::minMaxLoc(img, &min, &max);
  img = (img - min) / (max - min) * 255;
  cv::Mat out_img(cv::Mat::zeros(resolution_.py, resolution_.px, CV_8U));
  img.convertTo(out_img, CV_8U);
  return img;
}

Box::Segment Box::intersect(const Ray &r, float t0, float t1) const {
  float tmin, tmax, tymin, tymax, tzmin, tzmax;
  tmin = (bounds[r.sign[0]][0] - r.origin[0]) * r.inv_direction[0];
  tmax = (bounds[1 - r.sign[0]][0] - r.origin[0]) * r.inv_direction[0];
  tymin = (bounds[r.sign[1]][1] - r.origin[1]) * r.inv_direction[1];
  tymax = (bounds[1 - r.sign[1]][1] - r.origin[1]) * r.inv_direction[1];
  if ((tmin > tymax) || (tymin > tmax))
    return {};
  if (tymin > tmin)
    tmin = tymin;
  if (tymax < tmax)
    tmax = tymax;
  tzmin = (bounds[r.sign[2]][2] - r.origin[2]) * r.inv_direction[2];
  tzmax = (bounds[1 - r.sign[2]][2] - r.origin[2]) * r.inv_direction[2];
  if ((tmin > tzmax) || (tzmin > tmax))
    return {};
  if (tzmin > tmin)
    tmin = tzmin;
  if (tzmax < tmax)
    tmax = tzmax;
  if ((tmin < t1) && (tmax > t0))
    return {tmin, tmax};
  return {};
}
