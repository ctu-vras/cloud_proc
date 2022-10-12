#include <cloud_proc/projection.h>
#include <cloud_proc/timer.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace cloud_proc
{

class ProjectionNodelet : public nodelet::Nodelet
{
protected:
    Projection<float> projection_;
    std::string frame_;
    double timeout_ = 0.0;
    tf2_ros::Buffer tf_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    ros::Publisher cloud_pub_;
    ros::Subscriber cloud_sub_;
    ~ProjectionNodelet() override = default;
public:
    void update_params()
    {
      getPrivateNodeHandle().param("height", projection_.height_, projection_.height_);
      getPrivateNodeHandle().param("width", projection_.width_, projection_.width_);
      getPrivateNodeHandle().param("fov_azimuth", projection_.fov_azimuth_, projection_.fov_azimuth_);
      getPrivateNodeHandle().param("fov_elevation", projection_.fov_elevation_, projection_.fov_elevation_);
      getPrivateNodeHandle().param("f_azimuth", projection_.f_azimuth_, projection_.f_azimuth_);
      getPrivateNodeHandle().param("f_elevation", projection_.f_elevation_, projection_.f_elevation_);
      getPrivateNodeHandle().param("c_azimuth", projection_.c_azimuth_, projection_.c_azimuth_);
      getPrivateNodeHandle().param("c_elevation", projection_.c_elevation_, projection_.c_elevation_);
      getPrivateNodeHandle().param("keep", projection_.keep_, projection_.keep_);
      getPrivateNodeHandle().param("azimuth_only", projection_.azimuth_only_, projection_.azimuth_only_);
      getPrivateNodeHandle().param("frame", frame_, frame_);
      getPrivateNodeHandle().param("timeout", timeout_, timeout_);
      NODELET_INFO("Height: %i", projection_.height_);
      NODELET_INFO("Width: %i", projection_.width_);
      NODELET_INFO("Field of view in azimuth: %f", projection_.fov_azimuth_);
      NODELET_INFO("Field of view in elevation: %f", projection_.fov_elevation_);
      NODELET_INFO("Focal length azimuth: %f", projection_.f_azimuth_);
      NODELET_INFO("Focal length elevation: %f", projection_.f_elevation_);
      NODELET_INFO("Principal point azimuth: %f", projection_.c_azimuth_);
      NODELET_INFO("Principal point elevation: %f", projection_.c_elevation_);
      NODELET_INFO("Keep: %i", projection_.keep_);
      NODELET_INFO("Azimuth only: %i", projection_.azimuth_only_);
      NODELET_INFO("Use rotation to frame: %s", frame_.c_str());
      NODELET_INFO("Timeout: %.3f s", timeout_);
    }
    void advertise()
    {
      cloud_pub_ = getNodeHandle().advertise<sensor_msgs::PointCloud2>("output", 2);
    }
    void subscribe()
    {
      if (!frame_.empty())
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_);
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
      geometry_msgs::TransformStamped tf;
      tf.transform.rotation.w = 1.0;
      if (!frame_.empty())
      {
        try
        {
          tf = tf_.lookupTransform(frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(timeout_));
          tf.transform.translation.x = 0;
          tf.transform.translation.y = 0;
          tf.transform.translation.z = 0;
        }
        catch (tf2::TransformException& ex)
        {
          NODELET_ERROR("Could not transform %s to %s: %s.", msg->header.frame_id.c_str(), frame_.c_str(), ex.what());
          return;
        }
        NODELET_INFO("Waited for transform: %f s.", t.secondsElapsed());
      }
      t.reset();
      auto output = boost::make_shared<sensor_msgs::PointCloud2>();
      projection_.process(*msg, tf.transform, *output);
      cloud_pub_.publish(output);
      NODELET_INFO("Projected %lu points: %f s.", num_points(*msg), t.secondsElapsed());
    }
};

}

PLUGINLIB_EXPORT_CLASS(cloud_proc::ProjectionNodelet, nodelet::Nodelet);
