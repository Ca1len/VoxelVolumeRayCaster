#include <SphereVolume.h>
#include <VoxelVolume.h>

SphereVolume::SphereVolume(uint16_t IDimension, uint16_t JDimension,
                           uint16_t KDimension, const Coordinate& ppCenter,
                           const PixelSpacing& spacing,
                           const Parallelepiped::Index& ballCenter,
                           float radius, float ball_value)
    : VoxelVolume(IDimension, JDimension, KDimension)
    , ball_center_(ballCenter)
    , radius_(radius)
    , ball_value_(ball_value) {
    pixelSpacing_ = spacing;
    parallelepipedCenter_ = ppCenter;

    cv::Matx44f scale = {spacing.sx, 0, 0,          0, 0, spacing.sy, 0, 0,
                         0,          0, spacing.sz, 0, 0, 0,          0, 1};

    cv::Matx44f translation = {1, 0, 0, ppCenter.x, 0, 1, 0, ppCenter.y,
                               0, 0, 1, ppCenter.z, 0, 0, 0, 1};
    cv::Matx44f ijk_translation = {
        1, 0, 0, (float)IDimension / 2, 0, 1, 0, (float)JDimension / 2,
        0, 0, 1, (float)KDimension / 2, 0, 0, 0, 1};
    IDXtoXYZ_ = translation * scale * ijk_translation.inv();
    XYZtoIDX_ = IDXtoXYZ_.inv();

    auto ijk_c = cv::Vec4f(ballCenter.i, ballCenter.j, ballCenter.k, 1);
    auto xyz_c = IDXtoXYZ_ * ijk_c;
    xyz_c[3] = 0;

    for (uint16_t i = 0; i < parallelepiped_->shape_.nx; ++i) {
        for (uint16_t j = 0; j < parallelepiped_->shape_.ny; ++j) {
            for (uint16_t k = 0; k < parallelepiped_->shape_.nz; ++k) {
                auto xyz = IDXtoXYZ_ * cv::Vec4f(i, j, k, 1);
                xyz[3] = 0;
                if (cv::norm(xyz - xyz_c) < radius_) {
                    (*parallelepiped_)[{i, j, k}] = ball_value;
                }
            }
        }
    }
}

float SphereVolume::p(float x, float y, float z) const {
    cv::Vec4f point{x, y, z, 1};
    cv::Vec4i idx = XYZtoIDX_ * point;
    if (!parallelepiped_->checkInShape(idx[0], idx[1], idx[2])) return 0;
    return (*parallelepiped_)[{static_cast<uint16_t>(idx[0]),
                               static_cast<uint16_t>(idx[1]),
                               static_cast<uint16_t>(idx[2])}];
}

float SphereVolume::p(float x, float y, float z) {
    cv::Vec4f point{x, y, z, 1};
    cv::Vec4i idx = XYZtoIDX_ * point;
    if (!parallelepiped_->checkInShape(idx[0], idx[1], idx[2])) return 0;
    return (*parallelepiped_)[{static_cast<uint16_t>(idx[0]),
                               static_cast<uint16_t>(idx[1]),
                               static_cast<uint16_t>(idx[2])}];
}

const cv::Matx44f& SphereVolume::getIJKtoXYZ() const { return IDXtoXYZ_; }
