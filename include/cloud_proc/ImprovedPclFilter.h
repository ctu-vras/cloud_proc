#pragma once

#include <pcl_ros/filters/filter.h>
#include <pcl_ros/transforms.h>

#include <diagnostic_updater/publisher.h>

#include <cras_cpp_common/diag_utils.hpp>
#include <cras_cpp_common/nodelet_utils.hpp>
#include <cras_cpp_common/tf2_sensor_msgs.h>

#include <memory>
#include <mutex>

namespace cloud_proc
{

class ImprovedFilter : public cras::NodeletBase<pcl_ros::Filter>
{
protected:

  typedef sensor_msgs::PointCloud2 PointCloud2;
  typedef boost::shared_ptr<std::vector<int>> IndicesPtr;
  typedef boost::shared_ptr<const std::vector<int>> IndicesConstPtr;
  typedef pcl_msgs::PointIndices PointIndices;
  typedef PointIndices::ConstPtr PointIndicesConstPtr;
  typedef pcl_ros::sync_policies::ApproximateTime<PointCloud2, pcl_msgs::PointIndices> ApproxPolicy;
  typedef pcl_ros::sync_policies::ExactTime<PointCloud2, pcl_msgs::PointIndices> ExactPolicy;

  std::unique_ptr<dynamic_reconfigure::Server<pcl_ros::FilterConfig>> srv;
  std::unique_ptr<message_filters::Synchronizer<ExactPolicy>> sync_input_indices_e;
  std::unique_ptr<message_filters::Synchronizer<ApproxPolicy>> sync_input_indices_a;

  std::mutex lastInputMutex;
  std::pair<PointCloud2::ConstPtr, IndicesPtr> lastInput;
  ros::Timer periodicPublishTimer;

  ros::Duration tfWaitTimeout;

  bool publishPeriodically;
  std::string fixedFrame;
  std::unordered_map<std::string, cras::CloudChannelType> transformChannels;

  std::unique_ptr<cras::DiagnosedPublisher<sensor_msgs::PointCloud2>> pubOutputDiag;
  std::unique_ptr<cras::DiagnosedPubSub<sensor_msgs::PointCloud2>> inputDiag;
  mutable bool diagHasTfProblems = false;
  double receiveRateTolerance;
  double publishRateTolerance;
  size_t receiveRateWindowSize;
  size_t publishRateWindowSize;
  double minAcceptableReceiveDelay;
  double maxAcceptableReceiveDelay;
  double minAcceptablePublishDelay;
  double maxAcceptablePublishDelay;

  void produceTfDiag(diagnostic_updater::DiagnosticStatusWrapper &stat);

  using Filter::getName;

public:
  ImprovedFilter();
  ~ImprovedFilter() override {}  // correctly delete the unique_ptrs

protected:

  inline sensor_msgs::PointCloud2Ptr transformPointCloud(const std::string& target_frame, const ros::Time& target_time,
      const sensor_msgs::PointCloud2& in, const std::string& logName) const
  {
      if (in.header.frame_id == target_frame && in.header.stamp == target_time)
          return boost::make_shared<sensor_msgs::PointCloud2>(in);

      // Get the TF transform
      geometry_msgs::TransformStamped transform;
      try
      {
          if (target_time == in.header.stamp)
          {
              NODELET_DEBUG("[%s] Transforming %s from %s to %s.", this->getName().c_str(), logName.c_str(),
                  in.header.frame_id.c_str(), target_frame.c_str());

              transform = this->getBuffer().lookupTransform(target_frame, in.header.frame_id, in.header.stamp,
                  this->tfWaitTimeout);
          }
          else
          {
              NODELET_DEBUG("[%s] Transforming %s from %s to %s via fixed frame %s.", this->getName().c_str(),
                  logName.c_str(), in.header.frame_id.c_str(), target_frame.c_str(), this->fixedFrame.c_str());

              transform = this->getBuffer().lookupTransform(target_frame, target_time, in.header.frame_id,
                  in.header.stamp, this->fixedFrame, this->tfWaitTimeout);
          }
      }
      catch (const tf::TransformException &e)
      {
        this->diagHasTfProblems = true;
        NODELET_ERROR("[%s] Error converting %s from %s to %s. Error: %s", this->getName().c_str(), logName.c_str(),
            in.header.frame_id.c_str(), target_frame.c_str(), e.what());
        return nullptr;
      }

      sensor_msgs::PointCloud2Ptr out(new sensor_msgs::PointCloud2);
      cras::transformWithChannels(in, *out, transform, this->transformChannels);

      return out;
  }

  inline PointCloud2::Ptr transformAsOutput(const PointCloud2& output) const
  {
      // Check whether the user has given a different output TF frame
      if (!this->tf_output_frame_.empty() && output.header.frame_id != this->tf_output_frame_)
          return this->transformPointCloud(this->tf_output_frame_, output.header.stamp, output, "output dataset");
      // no tf_output_frame given, transform the dataset to its original frame
      else if (this->tf_output_frame_.empty() && output.header.frame_id != this->tf_input_orig_frame_)
          return this->transformPointCloud(this->tf_input_orig_frame_, output.header.stamp, output, "output dataset");
      else
          return boost::make_shared<PointCloud2>(output);
  }

  inline PointCloud2::ConstPtr transformAsInput(const PointCloud2::ConstPtr& cloud) const
  {
      if (!this->tf_input_frame_.empty () && cloud->header.frame_id != this->tf_input_frame_)
          return this->transformPointCloud(this->tf_input_frame_, cloud->header.stamp, *cloud, "input dataset");
      else
          return boost::make_shared<PointCloud2>(*cloud);
  }

  inline void computePublish(const PointCloud2::ConstPtr& input, const IndicesPtr& indices)
  {
      PointCloud2 output;
      // Call the virtual method in the child
      this->filter(input, indices, output);

      // Copy timestamp from the PointCloud2 message because PCL decreases its precision
      output.header.stamp = input->header.stamp;

      auto cloud_tf = this->transformAsOutput(output);
      if (cloud_tf == nullptr)
          return;

      // Publish a boost shared ptr
      this->pubOutputDiag->publish(cloud_tf);

      this->diagHasTfProblems = false;
  }

  void periodicComputePublish(const ros::TimerEvent& event);

  void input_indices_callback (const PointCloud2::ConstPtr &cloud, const PointIndicesConstPtr &indices);

  void subscribe() override;

  void unsubscribe() override;

  void onInit() override;
};

}
