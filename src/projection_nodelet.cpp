#include <cloud_proc/destagger.h>
#include <cloud_proc/projection.h>
#include <cloud_proc/timer.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace cloud_proc
{

class ProjectionNodelet : public Projection<float>, public nodelet::Nodelet
{
protected:
    bool fallback_passthrough_ = true;
    ros::Publisher cloud_pub_;
    ros::Subscriber cloud_sub_;
    ~ProjectionNodelet() override = default;
public:
    void update_params()
    {
      getPrivateNodeHandle().param("height", height_, height_);
      getPrivateNodeHandle().param("width", width_, width_);
      getPrivateNodeHandle().param("f_azimuth", f_azimuth_, f_azimuth_);
      getPrivateNodeHandle().param("f_elevation", f_elevation_, f_elevation_);
      getPrivateNodeHandle().param("c_azimuth", c_azimuth_, c_azimuth_);
      getPrivateNodeHandle().param("c_elevation", c_elevation_, c_elevation_);
      getPrivateNodeHandle().param("keep", keep_, keep_);
      getPrivateNodeHandle().param("azimuth_only", azimuth_only_, azimuth_only_);
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
      if (!process(*msg, *output))
      {
        NODELET_WARN_THROTTLE(10.0, "Point cloud could not be processed.");
        if (!fallback_passthrough_)
          return;
      }
      cloud_pub_.publish(output);
      NODELET_DEBUG("Projected %i points: %f s.", msg->height * msg->width, t.secondsElapsed());
    }
};

}

PLUGINLIB_EXPORT_CLASS(cloud_proc::ProjectionNodelet, nodelet::Nodelet);
