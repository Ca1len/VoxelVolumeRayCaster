#pragma once

#include <cstddef>
#include <opencv2/core/matx.hpp>
#include <vector>


template <typename VoxelValueType, size_t FirstDimSize, size_t SecondDimSize>
struct Array2D {
  Array2D();
  Array2D(const Array2D &) = delete;
  Array2D(Array2D &&) = delete;
  Array2D &operator=(const Array2D &) = delete;
  Array2D &operator=(Array2D &&) = delete;

  std::vector<size_t> shape{FirstDimSize, SecondDimSize};
private:
  std::vector<std::vector<VoxelValueType>> array;
};

template <typename VoxelValueType> struct Array3D {
  Array3D();
  Array3D(const Array3D &) = delete;
  Array3D(Array3D &&) = delete;
  Array3D &operator=(const Array3D &) = delete;
  Array3D &operator=(Array3D &&) = delete;

private:
  std::vector<std::vector<VoxelValueType>> array;
};

template <typename VoxelValueType> struct Parellelepiped {};

class VoxelVolume {
public:
  VoxelVolume();
  virtual ~VoxelVolume();
  virtual float p(float x, float y, float z) { return 0; };
};
