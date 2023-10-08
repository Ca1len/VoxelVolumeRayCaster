#pragma once

#include <Source.h>
#include <VoxelVolume.h>
#include <opencv2/opencv.hpp>

#include <cstddef>


class Projection{
public:
    struct Resolution {
        size_t px{};
        size_t py{};
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
};
