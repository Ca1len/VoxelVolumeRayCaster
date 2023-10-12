#include "VoxelVolume.h"
#include <Projection.h>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <mutex>
#include <new>
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/matx.hpp>
#include <queue>

static uint8_t n_cores = std::thread::hardware_concurrency();

Projection::Projection(const Resolution& res, const Shape& shape)
    : resolution_(res), shape_(shape) {

    cv::Matx33f scale{(float)shape.rx / res.px,
                      0,
                      0,
                      0,
                      (float)shape.ry / res.py,
                      0,
                      0,
                      0,
                      1};
    cv::Matx33f translation{1, 0, shape_.rx / 2.0f, 0, 1, shape_.ry / 2.0f, 0,
                            0, 1};
    IJtoXY = translation.inv() * scale;
    XYtoIJ = IJtoXY.inv();
}

cv::Vec3f transformVec3(const cv::Matx44f& mat, const cv::Vec3f& vec) {
    cv::Vec4f temp = {vec[0], vec[1], vec[2], 0};
    temp = mat * temp;
    return {temp[0], temp[1], temp[2]};
}

cv::Vec3f transformPoint3(const cv::Matx44f& mat, const cv::Vec3f& point) {
    cv::Vec4f temp = {point[0], point[1], point[2], 1};
    temp = mat * temp;
    return {temp[0], temp[1], temp[2]};
}

static inline std::deque<Parallelepiped::Index> getIntersectedVoxels(
    const VoxelVolume& volume, const Ray& ray) {
    std::deque<Parallelepiped::Index> res{};
    auto pp = *volume.parallelepiped_;
    auto volume_shape = pp.shape_;
    auto IDXtoXYZ = volume.getIJKtoXYZ();
    auto XYZtoIDX = IDXtoXYZ.inv();

    cv::Vec3f ijk_lower{0, 0, 0};
    cv::Vec3f ijk_upper{(float)volume_shape.nx, (float)volume_shape.ny,
                        (float)volume_shape.nz};

    Ray ijk_ray = {transformPoint3(XYZtoIDX, ray.origin),
                   transformVec3(XYZtoIDX, ray.direction)};
    Box::Segment segment{};
    Box ijk_box = {ijk_lower, ijk_upper};

    segment = ijk_box.intersect(ijk_ray, 0, 1);
    int i = 0;
    do {
        auto ijk_source = ijk_ray.origin + segment.source * ijk_ray.direction;
        auto ijk_dest = ijk_ray.origin + segment.dest * ijk_ray.direction;

        auto source_voxel_idx = Parallelepiped::Index{
            MIN(std::floor(ijk_source[0]), ijk_upper[0] - 1),
            MIN(std::floor(ijk_source[1]), ijk_upper[1] - 1),
            MIN(std::floor(ijk_source[2]), ijk_upper[2] - 1),
        };

        auto dest_voxel_idx = Parallelepiped::Index{
            MIN(std::floor(ijk_dest[0]), ijk_upper[0] - 1),
            MIN(std::floor(ijk_dest[1]), ijk_upper[1] - 1),
            MIN(std::floor(ijk_dest[2]), ijk_upper[2] - 1),
        };

        Box source_voxel_bb{{static_cast<float>(source_voxel_idx.i),
                             static_cast<float>(source_voxel_idx.j),
                             static_cast<float>(source_voxel_idx.k)},
                            {static_cast<float>(source_voxel_idx.i + 1),
                             static_cast<float>(source_voxel_idx.j + 1),
                             static_cast<float>(source_voxel_idx.k + 1)}};

        Box dest_voxel_bb{{static_cast<float>(dest_voxel_idx.i),
                           static_cast<float>(dest_voxel_idx.j),
                           static_cast<float>(dest_voxel_idx.k)},
                          {static_cast<float>(dest_voxel_idx.i + 1),
                           static_cast<float>(dest_voxel_idx.j + 1),
                           static_cast<float>(dest_voxel_idx.k + 1)}};

#if 0
        if (std::isinf(ijk_ray.inv_direction[0])) {

            if (std::isinf(ijk_ray.inv_direction[1])) {
                res.push_back({static_cast<uint16_t>(source_voxel_idx.i - 1),
                               static_cast<uint16_t>(source_voxel_idx.j - 1),
                               static_cast<uint16_t>(source_voxel_idx.k)});
                res.push_back({static_cast<uint16_t>(dest_voxel_idx.i - 1),
                               static_cast<uint16_t>(dest_voxel_idx.j - 1),
                               static_cast<uint16_t>(dest_voxel_idx.k)});
            } else if (std::isinf(ijk_ray.inv_direction[2])) {
                res.push_back({static_cast<uint16_t>(source_voxel_idx.i - 1),
                               static_cast<uint16_t>(source_voxel_idx.j),
                               static_cast<uint16_t>(source_voxel_idx.k - 1)});
                res.push_back({static_cast<uint16_t>(dest_voxel_idx.i - 1),
                               static_cast<uint16_t>(dest_voxel_idx.j),
                               static_cast<uint16_t>(dest_voxel_idx.k - 1)});
            } else {
                res.push_back({static_cast<uint16_t>(source_voxel_idx.i - 1),
                               static_cast<uint16_t>(source_voxel_idx.j),
                               static_cast<uint16_t>(source_voxel_idx.k)});
                res.push_back({static_cast<uint16_t>(dest_voxel_idx.i - 1),
                               static_cast<uint16_t>(dest_voxel_idx.j),
                               static_cast<uint16_t>(dest_voxel_idx.k)});
            }
        }
        if (std::isinf(ijk_ray.inv_direction[1])) {
            if (std::isinf(ijk_ray.inv_direction[2])) {
                res.push_back({static_cast<uint16_t>(source_voxel_idx.i),
                               static_cast<uint16_t>(source_voxel_idx.j - 1),
                               static_cast<uint16_t>(source_voxel_idx.k - 1)});
                res.push_back({static_cast<uint16_t>(dest_voxel_idx.i),
                               static_cast<uint16_t>(dest_voxel_idx.j - 1),
                               static_cast<uint16_t>(dest_voxel_idx.k - 1)});
            } else {
                res.push_back({static_cast<uint16_t>(source_voxel_idx.i),
                               static_cast<uint16_t>(source_voxel_idx.j - 1),
                               static_cast<uint16_t>(source_voxel_idx.k)});
                res.push_back({static_cast<uint16_t>(dest_voxel_idx.i),
                               static_cast<uint16_t>(dest_voxel_idx.j - 1),
                               static_cast<uint16_t>(dest_voxel_idx.k)});
            }
        }
        if (std::isinf(ijk_ray.inv_direction[2])) {
            res.push_back({static_cast<uint16_t>(source_voxel_idx.i),
                           static_cast<uint16_t>(source_voxel_idx.j),
                           static_cast<uint16_t>(source_voxel_idx.k - 1)});
            res.push_back({static_cast<uint16_t>(dest_voxel_idx.i),
                           static_cast<uint16_t>(dest_voxel_idx.j),
                           static_cast<uint16_t>(dest_voxel_idx.k - 1)});
        }
#endif

        auto source_segment = source_voxel_bb.intersect(ray, 0, 1);
        auto dest_segment = dest_voxel_bb.intersect(ray, 0, 1);

        res.push_back(source_voxel_idx);
        res.push_back(dest_voxel_idx);

        segment.source = source_segment.dest;
        segment.dest = dest_segment.source;

    } while (segment.dest - segment.source > 0 /* && ++i < 1 */);

    return res;
}

static inline std::deque<Parallelepiped::Index> getInterestVoxels(
    const VoxelVolume& volume) {
    std::deque<Parallelepiped::Index> res{};
    const auto& pp = *volume.parallelepiped_;
    const auto& volume_shape = pp.shape_;

    for (uint16_t i = 0; i < volume_shape.nx; ++i) {
        for (uint16_t j = 0; j < volume_shape.ny; ++j) {
            for (uint16_t k = 0; k < volume_shape.nz; ++k) {
                Parallelepiped::Index idx{i, j, k};
                if (pp[idx] != 0) {
                    res.push_back(std::move(idx));
                }
            }
        }
    }
    return res;
}

#if 1
static std::deque<Parallelepiped::Index> interestIndexes;
#endif

static inline float transparency_integral(const VoxelVolume& volume,
                                          const Ray& ray) {
    float integral{0};
    auto& pp = volume.parallelepiped_;
    auto& volume_shape = pp->shape_;
    auto& spacing = volume.pixelSpacing_;
    auto& IDXtoXYZ = volume.getIJKtoXYZ();
    // auto interestIndexes = getIntersectedVoxels(volume, ray);

#if 1
    for (const auto& idx : interestIndexes) {
        cv::Vec4f ijk_point_s{(float)idx.i, (float)idx.j, (float)idx.k, 1};
        cv::Vec4f ijk_point_d{(float)(idx.i + 1), (float)(idx.j + 1),
                              (float)(idx.k + 1), 1};
        cv::Vec4f xyz_point_s = IDXtoXYZ * ijk_point_s;
        cv::Vec4f xyz_point_d = IDXtoXYZ * ijk_point_d;
        Box voxel{{(xyz_point_s[0] < xyz_point_d[0]) ? xyz_point_s[0]
                                                     : xyz_point_d[0],
                   (xyz_point_s[1] < xyz_point_d[1]) ? xyz_point_s[1]
                                                     : xyz_point_d[1],
                   (xyz_point_s[2] < xyz_point_d[2]) ? xyz_point_s[2]
                                                     : xyz_point_d[2]},
                  {(xyz_point_s[0] > xyz_point_d[0]) ? xyz_point_s[0]
                                                     : xyz_point_d[0],
                   (xyz_point_s[1] > xyz_point_d[1]) ? xyz_point_s[1]
                                                     : xyz_point_d[1],
                   (xyz_point_s[2] > xyz_point_d[2]) ? xyz_point_s[2]
                                                     : xyz_point_d[2]}};
        auto segment = voxel.intersect(ray, 0, 1);
        float length = 0;
        if (!segment) {
            continue;
        }
        length = cv::norm(ray.direction * (segment.dest - segment.source));
        integral += volume.p((xyz_point_s[0] + xyz_point_d[0]) / 2.,
                             (xyz_point_s[1] + xyz_point_d[1]) / 2.,
                             (xyz_point_s[2] + xyz_point_d[2]) / 2.) *
                    length;
    }
#endif

#if 0
    for (int i = 0; i < volume_shape.nx; ++i) {
        for (int j = 0; j < volume_shape.ny; ++j) {
            for (int k = 0; k < volume_shape.nz; ++k) {

                cv::Vec4f ijk_point_s{(float)i, (float)j, (float)k, 1};
                cv::Vec4f ijk_point_d{(float)(i + 1), (float)(j + 1),
                                      (float)(k + 1), 1};
                cv::Vec4f xyz_point_s = IDXtoXYZ * ijk_point_s;
                cv::Vec4f xyz_point_d = IDXtoXYZ * ijk_point_d;
                Box voxel{{(xyz_point_s[0] < xyz_point_d[0]) ? xyz_point_s[0]
                                                             : xyz_point_d[0],
                           (xyz_point_s[1] < xyz_point_d[1]) ? xyz_point_s[1]
                                                             : xyz_point_d[1],
                           (xyz_point_s[2] < xyz_point_d[2]) ? xyz_point_s[2]
                                                             : xyz_point_d[2]},
                          {(xyz_point_s[0] > xyz_point_d[0]) ? xyz_point_s[0]
                                                             : xyz_point_d[0],
                           (xyz_point_s[1] > xyz_point_d[1]) ? xyz_point_s[1]
                                                             : xyz_point_d[1],
                           (xyz_point_s[2] > xyz_point_d[2]) ? xyz_point_s[2]
                                                             : xyz_point_d[2]}};
                auto segment = voxel.intersect(ray, 0, 1);
                float length = 0;
                if (!segment) {
                    continue;
                }
                length =
                    cv::norm(ray.direction * (segment.dest - segment.source));
                if (std::isinf(ray.inv_direction[0]) || std::isinf(ray.inv_direction[1]) || std::isinf(ray.inv_direction[2]))
                    length /= 2;
                integral += volume.p((xyz_point_s[0] + xyz_point_d[0]) / 2.,
                                     (xyz_point_s[1] + xyz_point_d[1]) / 2.,
                                     (xyz_point_s[2] + xyz_point_d[2]) / 2.) *
                            length;
            }
        }
    }
#endif

#if 0
    for (int i = 0; i < volume_shape.nx; ++i) {
        for (int j = 0; j < volume_shape.ny; ++j) {
            for (int k = 0; k < volume_shape.nz; ++k) {
                cv::Vec4f ijk_point{(float)i, (float)j, (float)k, 1};
                cv::Vec4f xyz_point = IDXtoXYZ * ijk_point;
                Box voxel{{xyz_point[0] - spacing.sx / 2,
                           xyz_point[1] - spacing.sy / 2,
                           xyz_point[2] - spacing.sz / 2},
                          {xyz_point[0] + spacing.sx / 2,
                           xyz_point[1] + spacing.sy / 2,
                           xyz_point[2] + spacing.sz / 2}};
                auto segment = voxel.intersect(ray, 0, 1);
                float length = 0;
                if (segment) {
                    length = cv::norm(ray.direction * segment.dest -
                                      ray.direction * segment.source);
                }
                integral +=
                    volume.p(xyz_point[0], xyz_point[1], xyz_point[2]) * length;
            }
        }
    }
#endif
    return integral;
}

cv::Mat Projection::compute(const VoxelVolume& volume, const Source& source) {
    cv::Mat img(cv::Mat::zeros(resolution_.py, resolution_.px, CV_32F));
    cv::Vec3f source_point{source.x_src, source.y_src, source.z_src};
    static std::mutex img_mutex;
#if 1
    interestIndexes = getInterestVoxels(volume);
#endif 
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
    auto steps = (resolution_.px * resolution_.py) / n_cores;
    int step{0};
    for (int i = 0; i < resolution_.px; ++i) {
        for (int j = 0; j < resolution_.py; ++j) {
            futures_.push_back(std::async(std::launch::async, foo, i, j));
            if (futures_.size() % n_cores == 0) {
                for (auto& future : futures_) {
                    future.wait();
                }
                futures_.clear();
                std::cout << "Current/All: " << ++step << "/" << steps << "\n";
            }
        }
    }
#if 0
    double min, max;
    cv::minMaxLoc(img, &min, &max);
    img = (img - min) / (max - min) * 255;
    // cv::Mat out_img(cv::Mat::zeros(resolution_.py, resolution_.px, CV_8U));
    // img.convertTo(out_img, CV_8U);
#endif
    return img;
}

Box::Segment Box::intersect(const Ray& r, float t0, float t1) const {
    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    tmin = (bounds[r.sign[0]][0] - r.origin[0]) * r.inv_direction[0];
    tmax = (bounds[1 - r.sign[0]][0] - r.origin[0]) * r.inv_direction[0];
    tymin = (bounds[r.sign[1]][1] - r.origin[1]) * r.inv_direction[1];
    tymax = (bounds[1 - r.sign[1]][1] - r.origin[1]) * r.inv_direction[1];
    tzmin = (bounds[r.sign[2]][2] - r.origin[2]) * r.inv_direction[2];
    tzmax = (bounds[1 - r.sign[2]][2] - r.origin[2]) * r.inv_direction[2];
    // if (std::isinf(tmin)) {
    //     if (std::isinf(tymin)) {
    //         tmin = tzmin;
    //         tmax = tzmax;
    //         if ((tmin < t1) && (tmax > t0)) return {tmin, tmax};
    //         // tymin = tzmin;
    //         // tymax = tzmax;
    //     } else {
    //         tmin = tymin;
    //         tmax = tymax;
    //     }
    //     if (std::isinf(tzmin)) {
    //         tmin = tymin;
    //         tmax = tymax;
    //         if ((tmin < t1) && (tmax > t0)) return {tmin, tmax};
    //         // tzmin = tymin;
    //         // tzmax = tymax;
    //     } else {
    //         tmin = tzmin;
    //         tmax = tzmax;
    //     }
    // } else {
    //     if (std::isinf(tymin)) {
    //         tymin = tmin;
    //         tymax = tmax;
    //         if (std::isinf(tzmin)) {
    //             tzmin = tmin;
    //             tzmax = tmax;
    //             if ((tmin < t1) && (tmax > t0)) return {tmin, tmax};
    //         }
    //     }
    //     if (std::isinf(tzmin)) {
    //         tzmin = tymin;
    //         tzmax = tymax;
    //     }
    // }
    if ((tmin > tymax) || (tymin > tmax)) return {};
    if (tymin > tmin) tmin = tymin;
    if (tymax < tmax) tmax = tymax;
    if ((tmin > tzmax) || (tzmin > tmax)) return {};
    if (tzmin > tmin) tmin = tzmin;
    if (tzmax < tmax) tmax = tzmax;
    if ((tmin < t1) && (tmax > t0)) return {tmin, tmax};
    return {};
}
