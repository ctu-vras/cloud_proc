#include <cloud_proc/ImprovedPclFilter.h>
#include <cloud_proc/PclCropBoxFixed.h>
#include <pcl/filters/crop_box.h>
#include <pcl_ros/CropBoxConfig.h>

#include <pluginlib/class_list_macros.h>

/**
 * This file is basically a copy of pcl_ros CropBox with one change - it is based on
 * ImprovedPclFilter. Unfortunately, it is impossible to reuse the pcl_ros code or binaries
 * as they hide all symbols in the built filters library.
 */

namespace cloud_proc
{

class CropBox : public ImprovedFilter
{
protected:

  /** \brief Pointer to a dynamic reconfigure service. */
  std::unique_ptr<dynamic_reconfigure::Server<pcl_ros::CropBoxConfig>> srv_;

  /** \brief Call the actual filter.
    * \param input the input point cloud dataset
    * \param indices the input set of indices to use from \a input
    * \param output the resultant filtered dataset
    */
  inline void filter(const PointCloud2::ConstPtr& input, const IndicesPtr& indices,
          PointCloud2& output) override
  {
      boost::mutex::scoped_lock lock (this->mutex_);
      pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
      pcl_conversions::toPCL(*input, *pcl_input);
      this->impl_.setInputCloud(pcl_input);
      this->impl_.setIndices(indices);
      pcl::PCLPointCloud2 pcl_output;
      this->impl_.filter(pcl_output);
      pcl_conversions::moveFromPCL(pcl_output, output);
  }

  /** \brief Child initialization routine.
    * \param nh ROS node handle
    * \param has_service set to true if the child has a Dynamic Reconfigure service
    */
  bool child_init(ros::NodeHandle &nh, bool &has_service) override;

  /** \brief Dynamic reconfigure service callback.
    * \param config the dynamic reconfigure CropBoxConfig object
    * \param level the dynamic reconfigure level
    */
  void config_callback(pcl_ros::CropBoxConfig &config, uint32_t level);

private:
  /** \brief The PCL filter implementation used. */
  CropBoxPointCloud2 impl_;  // pcl::CropBox<> is broken in PCL 1.9

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

bool CropBox::child_init(ros::NodeHandle &nh, bool &has_service)
{
    // Enable the dynamic reconfigure service
    has_service = true;
    this->srv_ = std::make_unique<dynamic_reconfigure::Server<pcl_ros::CropBoxConfig>>(nh);
    dynamic_reconfigure::Server<pcl_ros::CropBoxConfig>::CallbackType f =
        boost::bind(&CropBox::config_callback, this, _1, _2);
    this->srv_->setCallback(f);

    return true;
}

void CropBox::config_callback(pcl_ros::CropBoxConfig &config, uint32_t /*level*/)
{
    boost::mutex::scoped_lock lock(this->mutex_);

    Eigen::Vector4f new_min_point(config.min_x, config.min_y, config.min_z, 0.0);
    Eigen::Vector4f new_max_point(config.max_x, config.max_y, config.max_z, 0.0);

    if (this->impl_.getMin() != new_min_point)
    {
        NODELET_DEBUG("[%s::config_callback] Setting the minimum point to: %f %f %f.",
            this->getName().c_str(), new_min_point(0), new_min_point(1), new_min_point(2));
        this->impl_.setMin(new_min_point);
    }

    if (this->impl_.getMax() != new_max_point)
    {
        NODELET_DEBUG("[%s::config_callback] Setting the maximum point to: %f %f %f.",
            this->getName().c_str(), new_max_point(0), new_max_point(1), new_max_point(2));
        this->impl_.setMax(new_max_point);
    }

    if (this->impl_.getKeepOrganized () != config.keep_organized)
    {
        NODELET_DEBUG("[%s::config_callback] Setting the filter keep_organized value to: %s.",
            this->getName().c_str(), config.keep_organized ? "true" : "false");
        this->impl_.setKeepOrganized (config.keep_organized);
    }

    if (this->impl_.getNegative() != config.negative)
    {
        NODELET_DEBUG("[%s::config_callback] Setting the filter negative flag to: %s.",
            this->getName().c_str(), config.negative ? "true" : "false");
        this->impl_.setNegative(config.negative);
    }

    if (this->tf_input_frame_ != config.input_frame)
    {
        this->tf_input_frame_ = config.input_frame;
        NODELET_DEBUG("[%s::config_callback] Setting the input TF frame to: %s.",
            this->getName().c_str(), this->tf_input_frame_.c_str());
    }

    if (this->tf_output_frame_ != config.output_frame)
    {
        this->tf_output_frame_ = config.output_frame;
        NODELET_DEBUG("[%s::config_callback] Setting the output TF frame to: %s.",
            this->getName().c_str(), this->tf_output_frame_.c_str());
    }
}

}

PLUGINLIB_EXPORT_CLASS(cloud_proc::CropBox, nodelet::Nodelet)

