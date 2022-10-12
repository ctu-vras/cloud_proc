#include <cloud_proc/ortho_stats.h>
#include <cloud_proc/timer.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace cloud_proc
{

class OrthoStatsNodelet : public nodelet::Nodelet
{
protected:
    OrthoStats<float> stats_;
    std::string frame_;
    double timeout_ = 0.0;
    tf2_ros::Buffer tf_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    ros::Publisher cloud_pub_;
    ros::Subscriber cloud_sub_;
    ~OrthoStatsNodelet() override = default;
public:
    void update_params()
    {
        getPrivateNodeHandle().param("height", stats_.height_, stats_.height_);
        getPrivateNodeHandle().param("width", stats_.width_, stats_.width_);
        getPrivateNodeHandle().param("fx", stats_.fx_, stats_.fx_);
        getPrivateNodeHandle().param("fy", stats_.fy_, stats_.fy_);
        getPrivateNodeHandle().param("cx", stats_.cx_, stats_.cx_);
        getPrivateNodeHandle().param("cy", stats_.cy_, stats_.cy_);
        getPrivateNodeHandle().param("mode", stats_.mode_, stats_.mode_);
        getPrivateNodeHandle().param("output_z", stats_.output_z_, stats_.output_z_);
        getPrivateNodeHandle().param("frame", frame_, frame_);
        getPrivateNodeHandle().param("timeout", timeout_, timeout_);
        NODELET_INFO("Height: %i", stats_.height_);
        NODELET_INFO("Width: %i", stats_.width_);
        NODELET_INFO("Focal length x: %f", stats_.fx_);
        NODELET_INFO("Focal length y: %f", stats_.fy_);
        NODELET_INFO("Principal point x: %f", stats_.cx_);
        NODELET_INFO("Principal point y: %f", stats_.cy_);
        NODELET_INFO("Mode: %i", stats_.mode_);
        NODELET_INFO("Output z: %i", stats_.output_z_);
        NODELET_INFO("Frame: %s", frame_.c_str());
        NODELET_INFO("Timeout: %f", timeout_);
    }
    void advertise()
    {
      cloud_pub_ = getNodeHandle().advertise<sensor_msgs::PointCloud2>("output", 2);
    }
    void subscribe()
    {
      if (!frame_.empty())
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_);
      cloud_sub_ = getNodeHandle().subscribe("input", 2, &OrthoStatsNodelet::onCloud, this);
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
      stats_.process(*msg, tf.transform, *output);
      cloud_pub_.publish(output);
      NODELET_INFO("Projected %lu points: %f s.", num_points(*msg), t.secondsElapsed());
    }
};

}

PLUGINLIB_EXPORT_CLASS(cloud_proc::OrthoStatsNodelet, nodelet::Nodelet);
