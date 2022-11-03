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
    std::string target_frame_;
    bool use_only_orientation_ = false;
    float timeout_ = 0.0;
    tf2_ros::Buffer tf_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    ros::Publisher cloud_pub_;
    ros::Subscriber cloud_sub_;
    ~OrthoStatsNodelet() override = default;
public:
    void update_params()
    {
        // Read high-level parameters.
        int size = 0;
        getPrivateNodeHandle().param("size", size, size);
        if (size > 0)
        {
            NODELET_INFO("Size: %i", size);
            stats_.height_ = size;
            stats_.width_ = size;
        }

        float extent = 0.0;
        getPrivateNodeHandle().param("extent", extent, extent);
        if (extent > 0)
            NODELET_INFO("Extent [m]: %f", extent);

        float grid = 0.0;
        getPrivateNodeHandle().param("grid", grid, grid);
        if (grid > 0)
            NODELET_INFO("Grid [m]: %f", grid);

        // Compute derived parameters.
        if (size == 0 && extent > 0.0 && grid > 0.0)
        {
            size = int(std::ceil(extent / grid));
            NODELET_INFO("Size set to %i from extent %f and grid %f.", size, extent, grid);
        }
        else if (size > 0 && extent == 0.0 && grid > 0.0)
        {
            extent = size * grid;
            NODELET_INFO("Extent set to %f from size %i and grid %f.", extent, size, grid);
        }
        if (size > 0 && extent > 0.0)
        {
            stats_.fx_ = stats_.fy_ = size / extent;
            stats_.cx_ = stats_.cy_ = float(size - 1) / 2;
            NODELET_INFO("Focal length and principal point set to %f and %f, respectively, from size %i and extent %f.",
                         stats_.fx_, stats_.cx_, size, extent);
        }

        // Read low-level parameters with derived defaults.
        getPrivateNodeHandle().param("height", stats_.height_, stats_.height_);
        getPrivateNodeHandle().param("width", stats_.width_, stats_.width_);
        getPrivateNodeHandle().param("fx", stats_.fx_, stats_.fx_);
        getPrivateNodeHandle().param("fy", stats_.fy_, stats_.fy_);
        getPrivateNodeHandle().param("cx", stats_.cx_, stats_.cx_);
        getPrivateNodeHandle().param("cy", stats_.cy_, stats_.cy_);
        getPrivateNodeHandle().param("mode", stats_.mode_, stats_.mode_);
        getPrivateNodeHandle().param("output_z", stats_.output_z_, stats_.output_z_);
        getPrivateNodeHandle().param("eigenvalues", stats_.eigenvalues_, stats_.eigenvalues_);
        getPrivateNodeHandle().param("target_frame", target_frame_, target_frame_);
        getPrivateNodeHandle().param("use_only_orientation", use_only_orientation_, use_only_orientation_);
        getPrivateNodeHandle().param("min_z", stats_.min_z_, stats_.min_z_);
        getPrivateNodeHandle().param("max_z", stats_.max_z_, stats_.max_z_);
        getPrivateNodeHandle().param("zero_valid", stats_.zero_valid_, stats_.zero_valid_);
        getPrivateNodeHandle().param("timeout", timeout_, timeout_);
        NODELET_INFO("Height: %i", stats_.height_);
        NODELET_INFO("Width: %i", stats_.width_);
        NODELET_INFO("Focal length x: %f", stats_.fx_);
        NODELET_INFO("Focal length y: %f", stats_.fy_);
        NODELET_INFO("Principal point x: %f", stats_.cx_);
        NODELET_INFO("Principal point y: %f", stats_.cy_);
        NODELET_INFO("Mode: %i", stats_.mode_);
        NODELET_INFO("Output z: %i", stats_.output_z_);
        NODELET_INFO("Eigenvalues: %i", stats_.eigenvalues_);
        NODELET_INFO("Target frame: %s", target_frame_.c_str());
        NODELET_INFO("Use only orientation: %i", use_only_orientation_);
        if (use_only_orientation_)
            NODELET_WARN("Output cloud data will not be consistent with reported (input) frame.");
        NODELET_INFO("Min z: %f", stats_.min_z_);
        NODELET_INFO("Max z: %f", stats_.max_z_);
        NODELET_INFO("Zero valid: %i", stats_.zero_valid_);
        NODELET_INFO("Timeout: %f", timeout_);
    }
    void advertise()
    {
        cloud_pub_ = getNodeHandle().advertise<sensor_msgs::PointCloud2>("output", 2);
    }
    void subscribe()
    {
        if (!target_frame_.empty())
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
        if (!target_frame_.empty())
        {
            try
            {
                tf = tf_.lookupTransform(target_frame_, msg->header.frame_id,
                                         msg->header.stamp, ros::Duration(timeout_));
                if (use_only_orientation_)
                {
                    tf.transform.translation.x = 0.0;
                    tf.transform.translation.y = 0.0;
                    tf.transform.translation.z = 0.0;
                }
            }
            catch (const tf2::TransformException& ex)
            {
                NODELET_ERROR("Could not transform %s to %s: %s.",
                              msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
                return;
            }
            NODELET_INFO("Waited for transform: %f s.", t.secondsElapsed());
        }
        t.reset();
        auto output = boost::make_shared<sensor_msgs::PointCloud2>();
        stats_.process(*msg, tf.transform, *output);
        if (!target_frame_.empty() && !use_only_orientation_)
            output->header.frame_id = target_frame_;
        cloud_pub_.publish(output);
        NODELET_INFO("Projected %lu points: %f s.", num_points(*msg), t.secondsElapsed());
    }
};

}

PLUGINLIB_EXPORT_CLASS(cloud_proc::OrthoStatsNodelet, nodelet::Nodelet);
