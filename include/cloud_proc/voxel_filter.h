#pragma once

#include <cmath>
#include <cloud_proc/timer.h>
#include <random>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <unordered_set>
#include <unordered_map>

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

template<class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
  // Boost-like hash combine
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

template<typename I>
class Voxel
{
public:
    typedef Voxel<I> Same;

    class Hash
    {
    public:
        size_t operator()(const Same& voxel) const
        noexcept
        {
          return voxel.hash();
        }
    };

    Voxel()
    {}
    Voxel(I x, I y, I z)
        :
        x_(x), y_(y), z_(z)
    {}
    Voxel(const Same& other)
        :
        x_(other.x_), y_(other.y_), z_(other.z_)
    {}
    size_t hash() const
    noexcept
    {
      size_t h = 0;
      hash_combine(h, x_);
      hash_combine(h, y_);
      hash_combine(h, z_);
      return h;
    }
    friend bool operator==(const Same& a, const Same& b)
    {
      return (a.x_ == b.x_) && (a.y_ == b.y_) && (a.z_ == b.z_);
    }

    static I lb()
    { return std::numeric_limits<I>::min(); }
    static I ub()
    { return std::numeric_limits<I>::max(); }

    template<typename T>
    bool from(const T* values, T grid)
    {
      if (!std::isfinite(values[0]) || !std::isfinite(values[1]) || !std::isfinite(values[2]))
      {
        return false;
      }
      // TODO: Saturate cast.
      T x = std::floor(values[0] / grid);
      T y = std::floor(values[1] / grid);
      T z = std::floor(values[2] / grid);
      if (x < lb() || x > ub() || y < lb() || y > ub() || z < lb() || z > ub())
      {
        return false;
      }
      x_ = static_cast<I>(x);
      y_ = static_cast<I>(y);
      z_ = static_cast<I>(z);
      return true;
    }
    template<typename T>
    void center(T* values, T grid)
    {
      values[0] = (T(x_) + 0.5) * grid;
      values[1] = (T(y_) + 0.5) * grid;
      values[2] = (T(z_) + 0.5) * grid;
    }

    I x_{0};
    I y_{0};
    I z_{0};
};

template<typename I>
using VoxelVec = std::vector<Voxel<I>>;

template<typename I>
using VoxelSet = std::unordered_set<Voxel<I>, typename Voxel<I>::Hash>;

template<typename I, typename T>
using VoxelMap = std::unordered_map<Voxel<I>, T, typename Voxel<I>::Hash>;

template<typename T, typename I>
void voxel_filter(const sensor_msgs::PointCloud2& input,
                  const std::string& field,
                  const T grid,
                  sensor_msgs::PointCloud2& output,
                  VoxelSet<I>& voxels)
{
  assert(input.row_step == input.width * input.point_step);
  size_t n_pts = input.height * input.width;
//  std::vector<size_t> indices;
//  random_permutation(n_pts, indices);
  std::vector<size_t> keep;
  keep.reserve(n_pts);
  sensor_msgs::PointCloud2ConstIterator<T> pt_it(input, field);
  voxels.reserve(keep.size());
  for (size_t i = 0; i < n_pts; ++i, ++pt_it)
  {
    Voxel<I> voxel;
    if (!voxel.from(&pt_it[0], grid))
      continue;

    if (voxels.find(voxel) == voxels.end())
    {
      voxels.insert(voxel);
      keep.push_back(i);
    }
  }
  // Copy selected indices.
  copy_points(input, keep, output);
}

template<typename T, typename I>
class VoxelFilter
{
public:

//  enum Keep
//  {
//      KEEP_FIRST = 0,
//      KEEP_LAST = 1,
//      KEEP_RANDOM = 2,
//  };

  std::string field_ = "x";
  T grid_ = 1.0;
//  int keep_ = KEEP_FIRST;

  virtual ~VoxelFilter() = default;

  void process(const sensor_msgs::PointCloud2& input, sensor_msgs::PointCloud2& output)
  {
//    assert(keep_ == KEEP_FIRST);
    VoxelSet<I> voxels;
    voxel_filter<T, I>(input, field_, grid_, output, voxels);
  }
};

}
