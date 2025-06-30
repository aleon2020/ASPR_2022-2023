#include "opencv2/opencv.hpp"
#include "std_msgs/msg/string.hpp"

namespace clase_cmake {

double duplica(double value_in);
void show_image(cv::Mat & image);
void fill_msg(std_msgs::msg::String & msg_in);

}  // namespace clase_cmake
