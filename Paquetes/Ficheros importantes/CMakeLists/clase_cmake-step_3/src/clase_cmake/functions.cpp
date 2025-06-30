#include "clase_cmake/functions.hpp"

namespace clase_cmake {

double duplica(double value_in)
{
  return value_in * 2.0;
}

void show_image(cv::Mat & image)
{
  cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
  cv::imshow("Display Image", image);
  cv::waitKey(0);
}

}  // namespace clase_cmake
