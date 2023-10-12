#pragma once

#include <Source.h>
#include <VoxelVolume.h>
#include <future>
#include <opencv2/opencv.hpp>

#include <cstddef>

class Ray {
public:
    Ray(cv::Vec3f o, cv::Vec3f d) {
        origin = o;
        direction = d;
        inv_direction = cv::Vec3f(1 / d[0], 1 / d[1], 1 / d[2]);
        sign[0] = (inv_direction[0] < 0);
        sign[1] = (inv_direction[1] < 0);
        sign[2] = (inv_direction[2] < 0);
    }
    cv::Vec3f origin;
    cv::Vec3f direction;
    cv::Vec3f inv_direction;
    int sign[3];
};

class Box {
public:
    Box(cv::Vec3f lower, cv::Vec3f upper) {
        bounds[0] = lower;
        bounds[1] = upper;
    }

    struct Segment {
        float source{0};
        float dest{0};
        operator bool() const {
            if (source || dest) return true;
            return false;
        };
    };

    Segment intersect(const Ray& r, float t0, float t1) const;
    cv::Vec3f bounds[2];
};

class Projection {
public:
    struct Resolution {
        uint16_t px{};
        uint16_t py{};
    } resolution_;

    struct Shape {
        float rx{};
        float ry{};
    } shape_;

    Projection(const Resolution& res, const Shape& shape);

    cv::Mat compute(const VoxelVolume& volume, const Source& source);

private:
    cv::Matx33f IJtoXY;
    cv::Matx33f XYtoIJ;

    std::vector<std::future<void>> futures_;
};
