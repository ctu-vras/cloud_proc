#pragma once

#include <cloud_proc/common.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>

namespace cloud_proc
{

template<typename T>
void box_filter(const sensor_msgs::PointCloud2& input,
                const std::string& field,
                const T min,
                const T max,
                const bool negative,
                sensor_msgs::PointCloud2& output)
{
  assert(input.row_step == input.width * input.point_step);
  size_t n = num_points(input);
  std::vector<size_t> keep;
  keep.reserve(n);
  sensor_msgs::PointCloud2ConstIterator<T> x_in(input, field);
  for (size_t i = 0; i < n; ++i, ++x_in)
  {
    if ((!negative && x_in[0] >= min && x_in[0] <= max)
        || (negative && (x_in[0] < min || x_in[0] > max)))
      keep.push_back(i);
  }
  // Copy selected indices.
  copy_points(input, keep, output);
}

template<typename T>
class BoxFilter
{
public:
  std::string field_ = "x";
  // Use min/max applicable for integral types too.
  T min_ = std::numeric_limits<T>::min();
  T max_ = std::numeric_limits<T>::max();
  bool negative_ = false;
  virtual ~BoxFilter() = default;
  void process(const sensor_msgs::PointCloud2& input, sensor_msgs::PointCloud2& output)
  {
    box_filter(input, field_, min_, max_, negative_, output);
  }
};

}
