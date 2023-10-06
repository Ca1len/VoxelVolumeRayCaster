#pragma once

#include <VoxelVolume.h>

class SphereVolume : public VoxelVolume{
public:
    SphereVolume(float radius, float ball_value);
    ~SphereVolume();

    float p(float x, float y, float z);

private:
    float radius_{};
    // density
    float ball_value{};
};
