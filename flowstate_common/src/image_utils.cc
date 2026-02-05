#include "flowstate_common/image_utils.h"

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace flowstate_common {

bool ConvertBgrToRgb(sensor_msgs::msg::Image& image) {
  if (image.encoding != sensor_msgs::image_encodings::BGR8) {
    return false;
  }

  if (image.width == 0 || image.height == 0) {
    return false;
  }

  cv::Mat bgr_image(image.height, image.width, CV_8UC3, image.data.data(),
                    image.step);
  cv::Mat rgb_image(image.height, image.width, CV_8UC3, image.data.data(),
                    image.step);
  cv::cvtColor(bgr_image, rgb_image, cv::COLOR_BGR2RGB);

  image.encoding = sensor_msgs::image_encodings::RGB8;
  return true;
}

std::unique_ptr<sensor_msgs::msg::Image> ConvertBgrImageToRgb(
    const sensor_msgs::msg::Image& bgr_image) {
  if (bgr_image.encoding != sensor_msgs::image_encodings::BGR8) {
    return nullptr;
  }

  if (bgr_image.width == 0 || bgr_image.height == 0) {
    return nullptr;
  }

  auto rgb_image = std::make_unique<sensor_msgs::msg::Image>();
  rgb_image->header = bgr_image.header;
  rgb_image->height = bgr_image.height;
  rgb_image->width = bgr_image.width;
  rgb_image->encoding = sensor_msgs::image_encodings::RGB8;
  rgb_image->is_bigendian = bgr_image.is_bigendian;
  rgb_image->step = bgr_image.step;
  rgb_image->data.resize(bgr_image.width * bgr_image.step);

  const cv::Mat bgr_mat(bgr_image.height, bgr_image.width, CV_8UC3,
                        const_cast<uint8_t*>(bgr_image.data.data()),
                        bgr_image.step);
  cv::Mat rgb_mat(rgb_image->height, rgb_image->width, CV_8UC3,
                  rgb_image->data.data(), rgb_image->step);
  cv::cvtColor(bgr_mat, rgb_mat, cv::COLOR_BGR2RGB);

  return rgb_image;
}

}  // namespace flowstate_common
