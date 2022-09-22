#include <cloud_proc/box_filter.h>
#include <cloud_proc/timer.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace cloud_proc
{

class BoxFilterNodelet: public nodelet::Nodelet
{
protected:
  BoxFilter<float> filter_;
  ros::Publisher cloud_pub_;
  ros::Subscriber cloud_sub_;
  ~BoxFilterNodelet() override = default;
public:
  void updateParams()
  {
    getPrivateNodeHandle().param("field", filter_.field_, filter_.field_);
    getPrivateNodeHandle().param("min", filter_.min_, filter_.min_);
    getPrivateNodeHandle().param("max", filter_.max_, filter_.max_);
    getPrivateNodeHandle().param("negative", filter_.negative_, filter_.negative_);
  }
  void advertise()
  {
    cloud_pub_ = getNodeHandle().advertise<sensor_msgs::PointCloud2>("output", 2);
  }
  void subscribe()
  {
    cloud_sub_ = getNodeHandle().subscribe("input", 2, &BoxFilterNodelet::onCloud, this);
  }
  void onInit() override
  {
    updateParams();
    advertise();
    subscribe();
  }
  void onCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    Timer t;
    auto output = boost::make_shared<sensor_msgs::PointCloud2>();
    // TODO: Allow data types other than float.
    filter_.process(*msg, *output);
    cloud_pub_.publish(output);
    NODELET_INFO("Box filter (%s) kept %lu / %lu points: %f s.",
                 filter_.field_.c_str(), num_points(*output), num_points(*msg), t.secondsElapsed());
  }
};

}

PLUGINLIB_EXPORT_CLASS(cloud_proc::BoxFilterNodelet, nodelet::Nodelet);
