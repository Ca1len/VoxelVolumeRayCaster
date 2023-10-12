#pragma once

#include <cstddef>
#include <cstdint>
#include <opencv2/core/matx.hpp>
#include <stdexcept>
#include <vector>

struct Parallelepiped {
    struct Index {
        uint16_t i{};
        uint16_t j{};
        uint16_t k{};
    };
    Parallelepiped(uint16_t IDimension, uint16_t JDimension,
                   uint16_t KDimension)
        : shape_({IDimension, JDimension, KDimension})
        , array_(IDimension * JDimension * KDimension, 0.) {}
    ~Parallelepiped() = default;

    struct Shape {
        uint16_t nx{};
        uint16_t ny{};
        uint16_t nz{};
    } const shape_;

    bool checkInShape(uint16_t i, uint16_t j, uint16_t k) const {
        if (i >= 0 && i < shape_.nx && j >= 0 && j < shape_.ny && k >= 0 &&
            k < shape_.nz)
            return true;
        return false;
    }
    bool checkInShape(const Index& idx) const {
        return checkInShape(idx.i, idx.j, idx.k);
    }

    float operator[](const Index& idx) const {
        if (idx.i >= shape_.nx || idx.j >= shape_.ny || idx.k >= shape_.nz)
            throw std::out_of_range("Parallelepiped operator[]");
        auto calcIdx = shape_.nx * (shape_.ny * idx.k + idx.j) + idx.i;
        return array_[calcIdx];
    }
    float& operator[](const Index& idx) {
        if (idx.i >= shape_.nx || idx.j >= shape_.ny || idx.k >= shape_.nz)
            throw std::out_of_range("Parallelepiped operator[]");
        auto calcIdx = shape_.nx * (shape_.ny * idx.k + idx.j) + idx.i;
        return array_[calcIdx];
    }

private:
    std::vector<float> array_;
};

struct VoxelVolume {
    VoxelVolume(uint16_t IDimension, uint16_t JDimension, uint16_t KDimension)
        : parallelepiped_(
              new Parallelepiped(IDimension, JDimension, KDimension)){};
    virtual ~VoxelVolume() {
        if (parallelepiped_) delete parallelepiped_;
    };
    virtual float p(float x, float y, float z) { return 0; };
    virtual float p(float x, float y, float z) const { return 0; };
    virtual const cv::Matx44f& getIJKtoXYZ() const = 0;

    struct PixelSpacing {
        float sx{};
        float sy{};
        float sz{};
    } pixelSpacing_{};

    struct Coordinate {
        float x{};
        float y{};
        float z{};
        Coordinate operator-(const Coordinate& rhs) const {
            return {x - rhs.x, y - rhs.y, z - rhs.z};
        }
    } parallelepipedCenter_{};

    Parallelepiped* parallelepiped_;
};
