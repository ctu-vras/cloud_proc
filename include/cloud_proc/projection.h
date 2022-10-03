#pragma once

#include <cmath>
#include <cloud_proc/common.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>

namespace cloud_proc
{

template<class T>
T azimuth(T x, T y, T z)
{
  return std::atan2(y, x);
}

template<class T>
T elevation(T x, T y, T z)
{
  return std::atan2(z, std::hypot(x, y));
}

template<class T>
T hypot(const T x, const T y, const T z)
{
  // Use 3-argument hypot in C++17.
#if ((defined(_MSVC_LANG) && _MSVC_LANG >= 201703L) || __cplusplus >= 201703L)
  //C++17 specific stuff here
  return std::hypot(x, y, z);
#else
  return std::hypot(std::hypot(x, y), z);
#endif
}

template<class T>
class Projection
{
public:
  enum Keep
  {
      KEEP_FIRST = 0,
      KEEP_LAST = 1,
      KEEP_CLOSEST = 2,
      KEEP_FARTHEST = 3,
  };

  int height_ = 0;
  int width_ = 0;
  T fov_azimuth_ = std::numeric_limits<T>::quiet_NaN();
  T fov_elevation_ = std::numeric_limits<T>::quiet_NaN();
  T f_azimuth_ = std::numeric_limits<T>::quiet_NaN();
  T f_elevation_ = std::numeric_limits<T>::quiet_NaN();
  T c_azimuth_ = std::numeric_limits<T>::quiet_NaN();
  T c_elevation_ = std::numeric_limits<T>::quiet_NaN();
  int keep_ = KEEP_LAST;
  bool azimuth_only_ = false;

  void process(const sensor_msgs::PointCloud2& input,
               const geometry_msgs::Transform& transform,
               sensor_msgs::PointCloud2& output)
  {
    typedef Eigen::Vector3f Vec3f;
    typedef Eigen::Map<const Vec3f> ConstVec3fMap;
    typedef Eigen::Transform<float, 3, Eigen::Isometry> Transform3f;
    assert(input.height >= 1);
    assert(input.width >= 1);
    assert(input.row_step == input.width * input.point_step);
    // Construct eigen transform to target frame.
    Transform3f tf = tf2::transformToEigen(transform).cast<float>();
    output.header = input.header;
    output.height = (azimuth_only_ || height_ <= 0) ? input.height : height_;
    assert(output.height >= 1);
    output.width = (width_ <= 0) ? input.width : width_;
    assert(output.width >= 1);
    output.fields = input.fields;
    output.is_bigendian = input.is_bigendian;
    output.point_step = input.point_step;
    output.row_step = output.width * output.point_step;
    output.data.clear();
    // Initialize output points to invalid (zero float are zero bytes).
    output.data.resize(size_t(output.height) * output.row_step, 0);
    output.is_dense = input.is_dense;

    T fov_azimuth = std::isfinite(fov_azimuth_) ? fov_azimuth_ : (2 * M_PI);
    T fov_elevation = std::isfinite(fov_elevation_) ? fov_elevation_ : (M_PI_2);
    T f_azimuth = std::isfinite(f_azimuth_) ? f_azimuth_ : -T(output.width) / fov_azimuth;
    T f_elevation = std::isfinite(f_elevation_) ? f_elevation_ : -T(output.height) / fov_elevation;
    T c_azimuth = std::isfinite(c_azimuth_) ? c_azimuth_ : T(output.width) / 2 - 0.5;
    T c_elevation = std::isfinite(c_elevation_) ? c_elevation_ : T(output.height) / 2 - 0.5;

    sensor_msgs::PointCloud2ConstIterator<T> x_in(input, "x");
    sensor_msgs::PointCloud2Iterator<T> x_out_begin(output, "x");
    for (size_t i_in = 0; i_in < input.height; ++i_in)
    {
      for (size_t j_in = 0; j_in < input.width; ++j_in, ++x_in)
      {
        ConstVec3fMap p(&x_in[0]);
        if (!is_point_valid(p(0), p(1), p(2), false))
          continue;
        Vec3f q = (transform.rotation.w == 1) ? p : tf * p;
        T u, v;
        u = f_azimuth * azimuth(q(0), q(1), q(2)) + c_azimuth;
        // Move [-0.5, 0.5) to [0, 1).
        u += 0.5;
        // Skip points outside the image.
        if (u < 0 || u >= output.width || std::isnan(u))
          continue;
        size_t j_out = u;
        size_t i_out;
        if (azimuth_only_)
          i_out = i_in;
        else
        {
          v = f_elevation * elevation(q(0), q(1), q(2)) + c_elevation;
          v += 0.5;
          if (v < 0 || v >= output.height || std::isnan(v))
            continue;
          i_out = v;
        }
        auto x_out = x_out_begin + i_out * output.width + j_out;
        // Skip if target pixel already contains a valid point.
        if (keep_ == KEEP_FIRST)
        {
          if (is_point_valid(x_out[0], x_out[1], x_out[2], false))
            continue;
        }
        // Skip if target pixel contains a valid closer point.
        else if (keep_ == KEEP_CLOSEST)
        {
          if (is_point_valid(x_out[0], x_out[1], x_out[2], false)
              && hypot(x_out[0], x_out[1], x_out[2]) <= hypot(x_in[0], x_in[1], x_in[2]))
            continue;
        }
        // Skip if target pixel contains a valid farther point.
        else if (keep_ == KEEP_FARTHEST)
        {
          if (is_point_valid(x_out[0], x_out[1], x_out[2], false)
              && hypot(x_out[0], x_out[1], x_out[2]) >= hypot(x_in[0], x_in[1], x_in[2]))
            continue;
        }

        // Copy the whole point to target pixel.
        std::copy(input.data.begin() + i_in * input.row_step + j_in * input.point_step,
                  input.data.begin() + i_in * input.row_step + (j_in + 1) * input.point_step,
                  output.data.begin() + i_out * output.row_step + j_out * output.point_step);
      }
    }
  }
};

}
