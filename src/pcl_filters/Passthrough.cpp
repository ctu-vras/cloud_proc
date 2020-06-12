#include <cloud_proc/ImprovedPclFilter.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/FilterConfig.h>

#include <pluginlib/class_list_macros.h>

/**
 * This file is basically a copy of pcl_ros PassThrough with one change - it is based on
 * ImprovedPclFilter. Unfortunately, it is impossible to reuse the pcl_ros code or binaries
 * as they hide all symbols in the built filters library.
 */

namespace cloud_proc
{

class PassThrough : public ImprovedFilter
{
protected:

  /** \brief Pointer to a dynamic reconfigure service. */
  std::unique_ptr<dynamic_reconfigure::Server<pcl_ros::FilterConfig>> srv_;

  /** \brief Call the actual filter.
    * \param input the input point cloud dataset
    * \param indices the input set of indices to use from \a input
    * \param output the resultant filtered dataset
    */
  inline void filter(const PointCloud2::ConstPtr& input, const IndicesPtr& indices,
                     PointCloud2& output) override
  {
      boost::mutex::scoped_lock lock(this->mutex_);
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
  void config_callback(pcl_ros::FilterConfig &config, uint32_t level) override;

private:
  /** \brief The PCL filter implementation used. */
  pcl::PassThrough<pcl::PCLPointCloud2> impl_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

bool PassThrough::child_init(ros::NodeHandle &nh, bool &has_service)
{
    // Enable the dynamic reconfigure service
    has_service = true;
    this->srv_ = std::make_unique<dynamic_reconfigure::Server<pcl_ros::FilterConfig>>(nh);
    dynamic_reconfigure::Server<pcl_ros::FilterConfig>::CallbackType f =
        boost::bind(&PassThrough::config_callback, this, _1, _2);
    this->srv_->setCallback(f);

    return true;
}

void PassThrough::config_callback(pcl_ros::FilterConfig &config, uint32_t /*level*/)
{
    boost::mutex::scoped_lock lock(this->mutex_);

    double filter_min, filter_max;
    this->impl_.getFilterLimits(filter_min, filter_max);

    if (filter_min != config.filter_limit_min)
    {
        filter_min = config.filter_limit_min;
        NODELET_DEBUG("[%s::config_callback] Setting the minimum filtering value a point will be considered from to: %f.",
            this->getName().c_str(), filter_min);
        this->impl_.setFilterLimits(filter_min, filter_max);
    }

    if (filter_max != config.filter_limit_max)
    {
        filter_max = config.filter_limit_max;
        NODELET_DEBUG("[%s::config_callback] Setting the maximum filtering value a point will be considered from to: %f.",
            this->getName().c_str(), filter_max);
        this->impl_.setFilterLimits(filter_min, filter_max);
    }

    if (this->impl_.getFilterFieldName () != config.filter_field_name)
    {
        this->impl_.setFilterFieldName(config.filter_field_name);
        NODELET_DEBUG("[%s::config_callback] Setting the filter field name to: %s.",
            this->getName().c_str(), config.filter_field_name.c_str());
    }

    if (this->impl_.getKeepOrganized () != config.keep_organized)
    {
        NODELET_DEBUG("[%s::config_callback] Setting the filter keep_organized value to: %s.",
            this->getName().c_str(), config.keep_organized ? "true" : "false");
        this->impl_.setKeepOrganized(config.keep_organized);
    }

    if (this->impl_.getFilterLimitsNegative () != config.filter_limit_negative)
    {
        NODELET_DEBUG("[%s::config_callback] Setting the filter negative flag to: %s.",
            this->getName().c_str(), config.filter_limit_negative ? "true" : "false");
        this->impl_.setFilterLimitsNegative(config.filter_limit_negative);
    }

    if (this->tf_input_frame_ != config.input_frame)
    {
        this->tf_input_frame_ = config.input_frame;
        NODELET_DEBUG("[%s::config_callback] Setting the input TF frame to: %s.",
            getName().c_str(), tf_input_frame_.c_str());
    }

    if (this->tf_output_frame_ != config.output_frame)
    {
        this->tf_output_frame_ = config.output_frame;
        NODELET_DEBUG("[%s::config_callback] Setting the output TF frame to: %s.",
            this->getName().c_str(), tf_output_frame_.c_str());
    }
}

}

PLUGINLIB_EXPORT_CLASS(cloud_proc::PassThrough, nodelet::Nodelet)

