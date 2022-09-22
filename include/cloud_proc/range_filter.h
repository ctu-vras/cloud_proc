#pragma once

#include <cmath>
#include <cloud_proc/common.h>
#include <cloud_proc/timer.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>

namespace cloud_proc
{

template<typename T>
void range_filter(const sensor_msgs::PointCloud2& input,
                  const std::string& field,
                  const T min,
                  const T max,
                  sensor_msgs::PointCloud2& output)
{
  assert(input.row_step == input.width * input.point_step);
  size_t n = num_points(input);
  std::vector<size_t> keep;
  keep.reserve(n);
  sensor_msgs::PointCloud2ConstIterator<T> x_in(input, field);
  for (size_t i = 0; i < n; ++i, ++x_in)
  {
    auto& x = x_in[0];
    auto& y = x_in[1];
    auto& z = x_in[2];
    if (!is_point_valid(x, y, z, false))
    {
      continue;
    }
    auto r = std::hypot(std::hypot(x, y), z);
    if (r >= min && r <= max)
    {
      keep.push_back(i);
    }
  }
  // Copy selected indices.
  copy_points(input, keep, output);
}

template<typename T>
class RangeFilter
{
public:
  std::string field_ = "x";
  T min_ = 0.0;
  T max_ = std::numeric_limits<T>::infinity();
  virtual ~RangeFilter() = default;
  void process(const sensor_msgs::PointCloud2& input, sensor_msgs::PointCloud2& output)
  {
    range_filter(input, field_, min_, max_, output);
  }
};

}
