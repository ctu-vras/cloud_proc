#include <cloud_proc/destagger.h>
#include <cloud_proc/projection.h>
#include <cloud_proc/timer.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace cloud_proc
{

class ProjectionNodelet : public nodelet::Nodelet
{
protected:
    Projection<float> projection_;
    ros::Publisher cloud_pub_;
    ros::Subscriber cloud_sub_;
    ~ProjectionNodelet() override = default;
public:
    void update_params()
    {
      getPrivateNodeHandle().param("height", projection_.height_, projection_.height_);
      getPrivateNodeHandle().param("width", projection_.width_, projection_.width_);
      getPrivateNodeHandle().param("f_azimuth", projection_.f_azimuth_, projection_.f_azimuth_);
      getPrivateNodeHandle().param("f_elevation", projection_.f_elevation_, projection_.f_elevation_);
      getPrivateNodeHandle().param("c_azimuth", projection_.c_azimuth_, projection_.c_azimuth_);
      getPrivateNodeHandle().param("c_elevation", projection_.c_elevation_, projection_.c_elevation_);
      getPrivateNodeHandle().param("keep", projection_.keep_, projection_.keep_);
      getPrivateNodeHandle().param("azimuth_only", projection_.azimuth_only_, projection_.azimuth_only_);
    }
    void advertise()
    {
      cloud_pub_ = getNodeHandle().advertise<sensor_msgs::PointCloud2>("output", 2);
    }
    void subscribe()
    {
      cloud_sub_ = getNodeHandle().subscribe("input", 2, &ProjectionNodelet::onCloud, this);
    }
    void onInit() override
    {
      update_params();
      advertise();
      subscribe();
    }
    void onCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
      Timer t;
      auto output = boost::make_shared<sensor_msgs::PointCloud2>();
      projection_.process(*msg, *output);
      cloud_pub_.publish(output);
      NODELET_INFO("Projected %lu points: %f s.", num_points(*msg), t.secondsElapsed());
    }
};

}

PLUGINLIB_EXPORT_CLASS(cloud_proc::ProjectionNodelet, nodelet::Nodelet);
