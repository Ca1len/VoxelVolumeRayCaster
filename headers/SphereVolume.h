#pragma once

#include <opencv2/opencv.hpp>

#include <VoxelVolume.h>

class SphereVolume : public VoxelVolume {
public:
    SphereVolume(uint16_t IDimension, uint16_t JDimension, uint16_t KDimension,
                 const Coordinate& ppCenter, const PixelSpacing& spacing,
                 const Parallelepiped::Index& ballCenter, float radius,
                 float ball_value);
    ~SphereVolume() = default;

    float p(float x, float y, float z) override;
    float p(float x, float y, float z) const override;
    const cv::Matx44f& getIJKtoXYZ() const override;

private:
    cv::Matx44f IDXtoXYZ_;
    cv::Matx44f XYZtoIDX_;
    Parallelepiped::Index ball_center_;
    float radius_{};
    // density
    float ball_value_{};
};
