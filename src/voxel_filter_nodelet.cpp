#include <cloud_proc/voxel_filter.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace cloud_proc
{

class VoxelFilterNodelet : public nodelet::Nodelet
{
protected:
  VoxelFilter<float, int> filter_;
  ros::Publisher cloud_pub_;
  ros::Subscriber cloud_sub_;
  ~VoxelFilterNodelet() override = default;
public:
  void updateParams()
  {
    getPrivateNodeHandle().param("field", filter_.field_, filter_.field_);
    getPrivateNodeHandle().param("grid", filter_.grid_, filter_.grid_);
//    getPrivateNodeHandle().param("keep", filter_.keep_, filter_.keep_);
  }
  void advertise()
  {
    cloud_pub_ = getNodeHandle().advertise<sensor_msgs::PointCloud2>("output", 2);
  }
  void subscribe()
  {
    cloud_sub_ = getNodeHandle().subscribe("input", 2, &VoxelFilterNodelet::onCloud, this);
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
    NODELET_DEBUG("Voxel filter kept %u / %u points: %f s.",
                  output->height * output->width, msg->height * msg->width, t.secondsElapsed());
  }
};

}

PLUGINLIB_EXPORT_CLASS(cloud_proc::VoxelFilterNodelet, nodelet::Nodelet);
