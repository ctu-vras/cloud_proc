#pragma once

#include <cmath>
#include <cloud_proc/common.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>

namespace cloud_proc
{

template<class T>
class RunningStats
{
/// Naive implementation of running statistics.
public:
    T x_ = 0;
    T x2_ = 0;
    T min_ = std::numeric_limits<T>::infinity();
    T max_ = -std::numeric_limits<T>::infinity();
    unsigned n_ = 0;

    void reset()
    {
        x_ = 0;
        x2_ = 0;
        min_ = std::numeric_limits<T>::infinity();
        max_ = -std::numeric_limits<T>::infinity();
        n_ = 0;
    }

    void update(const T x)
    {
        x_ += x;
        x2_ += x * x;
        min_ = std::min(min_, x);
        max_ = std::max(max_, x);
        ++n_;
    }

    T mean() const
    {
        return n_ ? x_ / n_ : std::numeric_limits<T>::quiet_NaN();
    }

    T var() const
    {
        return n_ ? (x2_ / n_ - mean() * mean()) : std::numeric_limits<T>::quiet_NaN();
    }

    T std() const
    {
        return std::sqrt(var());
    }

    T min() const
    {
        return n_ ? min_ : std::numeric_limits<T>::quiet_NaN();
    }

    T max() const
    {
        return n_ ? max_ : std::numeric_limits<T>::quiet_NaN();
    }
};

template<class T>
class RunningStats3d
{
/// Naive implementation of running statistics.
public:
    typedef Eigen::Matrix<T, 3, 3> Mat3;
    typedef Eigen::Matrix<T, 3, 1> Vec3;
    typedef Eigen::Map<const Vec3> ConstVec3Map;
    Vec3 x_ = Vec3::Zero();
    Mat3 x2_ = Mat3::Zero();
    Vec3 min_ = Vec3::Constant(std::numeric_limits<T>::infinity());
    Vec3 max_ = Vec3::Constant(-std::numeric_limits<T>::infinity());
    unsigned n_ = 0;

    void reset()
    {
        x_.setZero();
        x2_.setZero();
        min_.setConstant(std::numeric_limits<T>::infinity());
        max_.setConstant(-std::numeric_limits<T>::infinity());
        n_ = 0;
    }

    void update(const T* x_ptr)
    {
        ConstVec3Map x(x_ptr);
        x_ += x;
        x2_ += x * x.transpose();
        min_ = min_.cwiseMin(x);
        max_ = max_.cwiseMax(x);
        ++n_;
    }

    Vec3 mean() const
    {
        return n_ ? Vec3(x_ / n_) : Vec3::Constant(std::numeric_limits<T>::quiet_NaN());
    }

    Mat3 cov() const
    {
        return n_ ? Mat3(x2_ / n_ - mean() * mean().transpose()) : Mat3::Constant(std::numeric_limits<T>::quiet_NaN());
    }

    Vec3 min() const
    {
        return n_ ? min_ : Vec3::Constant(std::numeric_limits<T>::quiet_NaN());
    }

    Vec3 max() const
    {
        return n_ ? max_ : Vec3::Constant(std::numeric_limits<T>::quiet_NaN());
    }
};

template<class T>
class OrthoStats
{
public:
    typedef Eigen::Matrix<T, 3, 3> Mat3;
    typedef Eigen::Vector3f Vec3f;
    typedef Eigen::Map<const Vec3f> ConstVec3fMap;
    typedef Eigen::Transform<float, 3, Eigen::Isometry> Transform3f;

    enum Output
    {
        OUTPUT_Z_MIN = 0,
        OUTPUT_Z_MAX = 1,
        OUTPUT_Z_MEAN = 2,
    };
    enum Mode
    {
        MODE_1D = 0,
        MODE_3D = 1,
    };

    float min_z_ = -std::numeric_limits<float>::infinity();
    float max_z_ = std::numeric_limits<float>::infinity();
    bool zero_valid_ = false;

    float fx_ = 25.6;
    float fy_ = 25.6;
    float cx_ = 0.0;
    float cy_ = 0.0;
    int height_ = 256;
    int width_ = 256;

    int output_z_ = OUTPUT_Z_MEAN;
    int mode_ = MODE_1D;

    std::vector<RunningStats<T>> stats_x_;
    std::vector<RunningStats<T>> stats_y_;
    std::vector<RunningStats<T>> stats_z_;
    std::vector<RunningStats3d<T>> stats_xyz_;

    void reset_stats()
    {
        if (mode_ == MODE_1D)
        {
            stats_x_.resize(width_ * height_);
            stats_y_.resize(width_ * height_);
            stats_z_.resize(width_ * height_);
            for (auto& stats : stats_x_)
                stats.reset();
            for (auto& stats : stats_y_)
                stats.reset();
            for (auto& stats : stats_z_)
                stats.reset();
        }
        else
        {
            stats_xyz_.resize(width_ * height_);
            for (auto& stats : stats_xyz_)
                stats.reset();
        }
    }

    void process(const sensor_msgs::PointCloud2& input,
                 const geometry_msgs::Transform& transform,
                 sensor_msgs::PointCloud2& output)
    {
        assert(input.height >= 1);
        assert(input.width >= 1);
        assert(input.row_step == input.width * input.point_step);

        // Construct eigen transform to target frame.
        Transform3f tf = tf2::transformToEigen(transform).cast<float>();
        bool is_identity = (transform.rotation.w == 1.0
                            && transform.rotation.x == 0.0
                            && transform.rotation.y == 0.0
                            && transform.rotation.z == 0.0);

        // Compute statistics.
        reset_stats();
        size_t n = num_points(input);
        sensor_msgs::PointCloud2ConstIterator<T> x_in(input, "x");
        for (size_t i = 0; i < n; ++i, ++x_in)
        {
            ConstVec3fMap p(&x_in[0]);
            if (!is_point_valid(p(0), p(1), p(2), zero_valid_))
                continue;
            Vec3f q = is_identity ? p : tf * p;

            if (q(2) < min_z_ || q(2) > max_z_)
                continue;

            T u = fx_ * q(0) + cx_;
            // Move [-0.5, 0.5) to [0, 1).
            u += 0.5;
            // Skip points outside the image.
            if (u < 0 || u >= width_ || std::isnan(u))
                continue;
            size_t c = u;

            T v = fy_ * q(1) + cy_;
            v += 0.5;
            if (v < 0 || v >= height_ || std::isnan(v))
                continue;
            size_t r = v;

            auto j = r * width_ + c;

            if (mode_ == MODE_1D)
            {
                stats_x_[j].update(q(0));
                stats_y_[j].update(q(1));
                stats_z_[j].update(q(2));
            }
            else
            {
                stats_xyz_[j].update(&q(0));
            }
        }

        // Construct output point cloud.
        output.header = input.header;
        output.height = height_;
        output.width = width_;
        output.is_bigendian = input.is_bigendian;
        output.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(output);
        if (mode_ == MODE_1D)
        {
            modifier.setPointCloud2Fields(8,
                                          "x", 1, sensor_msgs::PointField::FLOAT32,
                                          "y", 1, sensor_msgs::PointField::FLOAT32,
                                          "z", 1, sensor_msgs::PointField::FLOAT32,
                                          "z_min", 1, sensor_msgs::PointField::FLOAT32,
                                          "z_max", 1, sensor_msgs::PointField::FLOAT32,
                                          "z_mean", 1, sensor_msgs::PointField::FLOAT32,
                                          "z_std", 1, sensor_msgs::PointField::FLOAT32,
                                          "support", 1, sensor_msgs::PointField::UINT32);
        }
        else if (mode_ == MODE_3D)
        {
            modifier.setPointCloud2Fields(14,
                                          "x", 1, sensor_msgs::PointField::FLOAT32,
                                          "y", 1, sensor_msgs::PointField::FLOAT32,
                                          "z", 1, sensor_msgs::PointField::FLOAT32,
                                          "z_min", 1, sensor_msgs::PointField::FLOAT32,
                                          "z_max", 1, sensor_msgs::PointField::FLOAT32,
                                          "z_mean", 1, sensor_msgs::PointField::FLOAT32,
                                          "z_std", 1, sensor_msgs::PointField::FLOAT32,
                                          "xx", 1, sensor_msgs::PointField::FLOAT32,
                                          "xy", 1, sensor_msgs::PointField::FLOAT32,
                                          "xz", 1, sensor_msgs::PointField::FLOAT32,
                                          "yy", 1, sensor_msgs::PointField::FLOAT32,
                                          "yz", 1, sensor_msgs::PointField::FLOAT32,
                                          "zz", 1, sensor_msgs::PointField::FLOAT32,
                                          "support", 1, sensor_msgs::PointField::UINT32);
        }

        n = num_points(output);
        sensor_msgs::PointCloud2Iterator<T> x_out(output, "x");
        sensor_msgs::PointCloud2Iterator<uint32_t> n_out(output, "support");
        for (size_t i = 0; i < n; ++i, ++x_out, ++n_out)
        {
            if (mode_ == MODE_1D)
            {
                x_out[0] = stats_x_[i].mean();
                x_out[1] = stats_y_[i].mean();
                if (output_z_ == OUTPUT_Z_MIN)
                    x_out[2] = stats_z_[i].min();
                else if (output_z_ == OUTPUT_Z_MAX)
                    x_out[2] = stats_z_[i].max();
                else if (output_z_ == OUTPUT_Z_MEAN)
                    x_out[2] = stats_z_[i].mean();
                x_out[3] = stats_z_[i].min();
                x_out[4] = stats_z_[i].max();
                x_out[5] = stats_z_[i].mean();
                x_out[6] = stats_z_[i].std();
                *n_out = stats_z_[i].n_;
            }
            else if (mode_ == MODE_3D)
            {
                x_out[0] = stats_xyz_[i].mean()(0);
                x_out[1] = stats_xyz_[i].mean()(1);
                if (output_z_ == OUTPUT_Z_MIN)
                    x_out[2] = stats_xyz_[i].min()(2);
                else if (output_z_ == OUTPUT_Z_MAX)
                    x_out[2] = stats_xyz_[i].max()(2);
                else if (output_z_ == OUTPUT_Z_MEAN)
                    x_out[2] = stats_xyz_[i].mean()(2);
                x_out[3] = stats_xyz_[i].min()(2);
                x_out[4] = stats_xyz_[i].max()(2);
                x_out[5] = stats_xyz_[i].mean()(2);
                Mat3 cov = stats_xyz_[i].cov();
                x_out[6] = std::sqrt(cov(2, 2));
                x_out[7] = stats_xyz_[i].cov()(0, 0);
                x_out[8] = stats_xyz_[i].cov()(0, 1);
                x_out[9] = stats_xyz_[i].cov()(0, 2);
                x_out[10] = stats_xyz_[i].cov()(1, 1);
                x_out[11] = stats_xyz_[i].cov()(1, 2);
                x_out[12] = stats_xyz_[i].cov()(2, 2);
                *n_out = stats_xyz_[i].n_;
            }
        }
    }
};

}
