#pragma once

#include <cstddef>
#include <cstdint>
#include <opencv2/core/matx.hpp>
#include <vector>


template <typename VoxelValueType=int16_t, size_t XDimension=512,
          size_t YDimension=512, size_t ZDimension=512>
struct Parallelepiped {
    struct Index{
        size_t i{};
        size_t j{};
        size_t k{};
    };
    Parallelepiped();
    const std::vector<size_t> shape{XDimension, YDimension, ZDimension};
    private:
    std::vector<VoxelValueType> array{XDimension*YDimension*ZDimension, 0};

    // (xd * yd) * z + (xd) * y + x
    // xd * (yd * z + y) + x
    VoxelValueType operator[](const Index& idx){
        auto calcIdx = shape[0] * (shape[1] * idx.k + idx.j) + idx.i;
        if (calcIdx >= array.size())
            throw "Out of range";
        return array[calcIdx];
    }
};

class VoxelVolume {
public:
  VoxelVolume();
  virtual ~VoxelVolume();
  virtual float p(float x, float y, float z) { return 0; };
};
