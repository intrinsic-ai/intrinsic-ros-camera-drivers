#ifndef FLOWSTATE_COMMON_IMAGE_UTILS_H_
#define FLOWSTATE_COMMON_IMAGE_UTILS_H_

#include <memory>

#include "sensor_msgs/msg/image.hpp"

namespace flowstate_common {

/**
 * @brief Convert a BGR image to RGB format in-place.
 * 
 * Converts a BGR8 encoded sensor_msgs::msg::Image to RGB8 format.
 * The conversion is done in-place to the provided image.
 * 
 * @param image BGR8 encoded image to convert (modified in-place)
 * @return true on success, false if image format is not BGR8 or has invalid dimensions
 */
bool ConvertBgrToRgb(sensor_msgs::msg::Image& image);

/**
 * @brief Convert BGR image data to RGB in a new image.
 *
 * Creates a new RGB8 encoded image from BGR8 source data.
 * 
 * @param bgr_image Source BGR8 image
 * @return New RGB8 image with converted data, or nullptr on error
 */
std::unique_ptr<sensor_msgs::msg::Image> ConvertBgrImageToRgb(
    const sensor_msgs::msg::Image& bgr_image);

}  // namespace flowstate_common

#endif  // FLOWSTATE_COMMON_IMAGE_UTILS_H_
