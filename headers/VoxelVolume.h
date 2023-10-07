#pragma once

#include <cstddef>
#include <cstdint>
#include <opencv2/core/matx.hpp>
#include <stdexcept>
#include <vector>

struct Parallelepiped {
  struct Index {
    size_t i{};
    size_t j{};
    size_t k{};
  };
  Parallelepiped(size_t IDimension, size_t JDimension, size_t KDimension)
      : shape_({IDimension, JDimension, KDimension}),
        array_(IDimension * JDimension * KDimension, 0.) {}
  ~Parallelepiped() = default;

  struct Shape {
    size_t nx{};
    size_t ny{};
    size_t nz{};
  } const shape_;

  bool checkInShape(size_t i, size_t j, size_t k) const {
    if (i >= 0 && i < shape_.nx && j >= 0 && j < shape_.ny && k >= 0 &&
        k < shape_.nz)
      return true;
    return false;
  }

  float operator[](const Index &idx) const {
    if (idx.i >= shape_.nx || idx.j >= shape_.ny || idx.k >= shape_.nz)
      throw std::out_of_range("Parallelepiped operator[]");
    auto calcIdx = shape_.nx * (shape_.ny * idx.k + idx.j) + idx.i;
    return array_[calcIdx];
  }
  float &operator[](const Index &idx) {
    if (idx.i >= shape_.nx || idx.j >= shape_.ny || idx.k >= shape_.nz)
      throw std::out_of_range("Parallelepiped operator[]");
    auto calcIdx = shape_.nx * (shape_.ny * idx.k + idx.j) + idx.i;
    return array_[calcIdx];
  }

private:
  std::vector<float> array_;
};

struct VoxelVolume {
  VoxelVolume(size_t IDimension, size_t JDimension, size_t KDimension)
      : parallelepiped_(
            new Parallelepiped(IDimension, JDimension, KDimension)){};
  virtual ~VoxelVolume() {
    if (parallelepiped_)
      delete parallelepiped_;
  };
  virtual float p(float x, float y, float z) { return 0; };

  struct PixelSpacing {
    float sx{};
    float sy{};
    float sz{};
  } pixelSpacing_{};

  struct Coordinate {
    float x{};
    float y{};
    float z{};
    Coordinate operator-(const Coordinate &rhs) const {
      return {x - rhs.x, y - rhs.y, z - rhs.z};
    }
  } parallelepipedCenter_{};

  Parallelepiped *parallelepiped_;
};
