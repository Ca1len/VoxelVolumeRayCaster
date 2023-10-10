#include "Projection.h"
#include "Source.h"
#include "SphereVolume.h"
#include "VoxelVolume.h"
#include <iostream>
#include <opencv2/opencv.hpp>

// using namespace cv;

int main(int argc, char **argv) {
  const size_t dims[3]{102, 102, 40};
  const Parallelepiped::Index ball_position{dims[0] / 2 + 3, dims[1] / 2 - 1,
                                            dims[2] / 2 + 5};
  const VoxelVolume::Coordinate volume_position{0, 0, 15};
  const VoxelVolume::PixelSpacing pixel_spacing{0.5, 0.4, 0.3};
  const float ball_radius{4};
  const float ball_density{20};
  VoxelVolume *volume =
      new SphereVolume(dims[0], dims[1], dims[2], volume_position,
                       pixel_spacing, ball_position, ball_radius, ball_density);
  Source source{0, 0, 40};
  const Projection::Resolution projection_resolution{102, 102};
  const Projection::Shape projection_shape{51, 51};
  Projection projection{projection_resolution, projection_shape};
  cv::Mat image = projection.compute(*volume, source);
  if (!image.data) {
    printf("No image data \n");
    return -1;
  }
  namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
  imshow("Display Image", image);
  cv::imwrite("out_image.tif", image);
  cv::waitKey(0);
  return 0;
}
