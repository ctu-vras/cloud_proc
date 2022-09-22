#pragma once

#include <sensor_msgs/PointCloud2.h>

namespace cloud_proc
{

/**
 * Copy selected points.
 * @tparam C A container type, with begin and size methods.
 * @param input Input cloud.
 * @param indices Container of indices.
 * @param output Output cloud.
 */
template<typename C>
void copy_points(const sensor_msgs::PointCloud2& input,
                 const C& indices,
                 sensor_msgs::PointCloud2& output)
{
  output.header = input.header;
  output.height = 1;
  output.width = decltype(output.width)(indices.size());
  output.fields = input.fields;
  output.is_bigendian = input.is_bigendian;
  output.point_step = input.point_step;
  output.row_step = output.width * output.point_step;
  output.data.resize(indices.size() * output.point_step);
  output.is_dense = input.is_dense;
  const auto in_ptr = input.data.data();
  uint8_t* out_ptr = output.data.data();
  auto it = indices.begin();
  for (size_t i = 0; i != indices.size(); ++i, ++it)
  {
    std::copy(in_ptr + (*it) * input.point_step,
              in_ptr + (*it + 1) * input.point_step,
              out_ptr + i * output.point_step);
  }
}

inline
size_t num_points(const sensor_msgs::PointCloud2& cloud)
{
  return size_t(cloud.height) * cloud.width;
}

template<class T>
bool is_point_valid(T x, T y, T z, bool zero_valid = true)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z)
      && (zero_valid || x != 0 || y != 0 || z != 0);
}

//template<typename C>
//void index_range(size_t n, C& indices)
//{
//  indices.reserve(n);
//  for (size_t i = 0; i < n; ++i)
//  {
//    indices.push_back(i);
//  }
//  indices.resize(n);
//}

//template<typename It>
//void shuffle(It begin, It end)
//{
//  std::random_device rd;
//  std::mt19937 g(rd());
//  std::shuffle(begin, end, g);
//}

//template<typename C>
//void random_permutation(size_t n, C& indices)
//{
//  Timer t;
//  index_range(n, indices);
//  shuffle(indices.begin(), indices.end());
//}

}
