#include <iostream>
#include <opencv2/core/matx.hpp>
#include <opencv2/opencv.hpp>

// using namespace cv;

#if 1
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
  // Optimized method
  bool intersect(const Ray &r, float t0, float t1) const {
    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    tmin = (bounds[r.sign[0]][0] - r.origin[0]) * r.inv_direction[0];
    tmax = (bounds[1 - r.sign[0]][0] - r.origin[0]) * r.inv_direction[0];
    tymin = (bounds[r.sign[1]][1] - r.origin[1]) * r.inv_direction[1];
    tymax = (bounds[1 - r.sign[1]][1] - r.origin[1]) * r.inv_direction[1];
    if ((tmin > tymax) || (tymin > tmax))
      return false;
    if (tymin > tmin)
      tmin = tymin;
    if (tymax < tmax)
      tmax = tymax;
    tzmin = (bounds[r.sign[2]][2] - r.origin[2]) * r.inv_direction[2];
    tzmax = (bounds[1 - r.sign[2]][2] - r.origin[2]) * r.inv_direction[2];
    if ((tmin > tzmax) || (tzmin > tmax))
      return false;
    if (tzmin > tmin)
      tmin = tzmin;
    if (tzmax < tmax)
      tmax = tzmax;
    return ((tmin < t1) && (tmax > t0));
  }
  cv::Vec3f bounds[2];
};
#endif

int main(int argc, char **argv) {
  Box box({0,0,0}, {5,5,5});

  Ray ray({0, 0, 0}, cv::Vec3f(1, 1, 1));
  bool res = box.intersect(ray, 4.9, 10);

  std::cout << res << "\n";
      // if ( argc != 2 )
      // {
      //     printf("usage: DisplayImage.out <Image_Path>\n");
      //     return -1;
      // }
      // Mat image;
      // image = imread( argv[1], IMREAD_COLOR );
      // if ( !image.data )
      // {
      //     printf("No image data \n");
      //     return -1;
      // }
      // namedWindow("Display Image", WINDOW_AUTOSIZE );
      // imshow("Display Image", image);
      // waitKey(0);
      return 0;
}
