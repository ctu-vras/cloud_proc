#include <cloud_proc/range_filter.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace cloud_proc
{

class RangeFilterNodelet : public nodelet::Nodelet
{
protected:
  RangeFilter<float> filter_;
  ros::Publisher cloud_pub_;
  ros::Subscriber cloud_sub_;
  ~RangeFilterNodelet() override = default;
public:
  void updateParams()
  {
    getPrivateNodeHandle().param("field", filter_.field_, filter_.field_);
    getPrivateNodeHandle().param("min", filter_.min_, filter_.min_);
    getPrivateNodeHandle().param("max", filter_.max_, filter_.max_);
  }
  void advertise()
  {
    cloud_pub_ = getNodeHandle().advertise<sensor_msgs::PointCloud2>("output", 2);
  }
  void subscribe()
  {
    cloud_sub_ = getNodeHandle().subscribe("input", 2, &RangeFilterNodelet::onCloud, this);
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
    filter_.process(*msg, *output);
    cloud_pub_.publish(output);
    NODELET_INFO("Range filter kept %lu / %lu points: %f s.",
                 num_points(*output), num_points(*msg), t.secondsElapsed());
  }
};

}

PLUGINLIB_EXPORT_CLASS(cloud_proc::RangeFilterNodelet, nodelet::Nodelet);
