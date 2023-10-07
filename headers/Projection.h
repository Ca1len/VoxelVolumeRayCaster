#pragma once

#include "Source.h"
#include "VoxelVolume.h"
#include <opencv2/opencv.hpp>

#include <cstddef>


class Projection{
public:

    struct Resolution {
        size_t px{};
        size_t py{};
    } resolution_;

    struct Shape {
        size_t rx{};
        size_t ry{};
    } shape_;

    cv::Mat compute(const VoxelVolume& volume, const Source& source);

};
