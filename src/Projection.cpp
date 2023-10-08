#include <Projection.h>

Projection::Projection(const Resolution &res, const Shape &shape)
    : resolution_(res), shape_(shape) {

  cv::Matx33f scale{
      (float)shape.rx / res.px, 0, 0, 
      0, (float)shape.ry / res.py, 0, 
      0, 0, 1};
  cv::Matx33f translation{1, 0, resolution_.px / 2.0f,
                          0, 1, resolution_.py / 2.0f,
                          0, 0, 1};
  IJtoXY = scale * translation.inv();
  XYtoIJ = IJtoXY.inv();
}

cv::Mat Projection::compute(const VoxelVolume &volume, const Source &source) {
    cv::Vec3f source_point {source.x_src, source.y_src, source.z_src};
    for (int i = 0; i < resolution_.px; ++i) {
        for (int j = 0; j < resolution_.py; ++j) {
            cv::Vec3f ij_point{(float)i, (float)j, 1};
            cv::Vec3f xyz_point = IJtoXY * ij_point;
            xyz_point[2] = 0;
            cv::Vec3f vec = xyz_point - source_point;
        }
    }
  return {};
}
