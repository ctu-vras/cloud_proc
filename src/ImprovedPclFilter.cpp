#include <sstream>
#define private public
#include <pcl_ros/filters/filter.h>
#undef private

#include <cloud_proc/ImprovedPclFilter.h>
#include <cras_cpp_common/type_utils.hpp>

namespace cloud_proc
{

void ImprovedFilter::subscribe()
{
    // If we're supposed to look for PointIndices (indices)
    if (this->use_indices_)
    {
        // Subscribe to the input using a filter
        this->sub_input_filter_.subscribe(*this->pnh_, "input", this->max_queue_size_);
        this->sub_indices_filter_.subscribe(*this->pnh_, "indices", this->max_queue_size_);

        auto cb = bind(&ImprovedFilter::input_indices_callback, this, _1, _2);  // NOLINT

        if (this->approximate_sync_)
        {
            this->sync_input_indices_a = std::make_unique<message_filters::Synchronizer<ApproxPolicy>>(this->max_queue_size_);
            this->sync_input_indices_a->connectInput(this->sub_input_filter_, this->sub_indices_filter_);
            this->sync_input_indices_a->registerCallback(cb);
        }
        else
        {
            this->sync_input_indices_e = std::make_unique<message_filters::Synchronizer<ExactPolicy>>(this->max_queue_size_);
            this->sync_input_indices_e->connectInput(this->sub_input_filter_, this->sub_indices_filter_);
            this->sync_input_indices_e->registerCallback(cb);
        }
    }
    else
    {
        // Subscribe in an old fashion to input only (no filters)
        auto cb = bind(&ImprovedFilter::input_indices_callback, this, _1,  // NOLINT
                       pcl_msgs::PointIndicesConstPtr());
        this->sub_input_ = this->pnh_->template subscribe<PointCloud2>("input", this->max_queue_size_, cb);
    }
}

void ImprovedFilter::unsubscribe()
{
    Filter::unsubscribe();

    {
        std::lock_guard<std::mutex> lock(this->lastInputMutex);
        this->lastInput.first.reset();
        this->lastInput.second.reset();
    }
}

void ImprovedFilter::onInit()
{
    // Call the super onInit ()
    PCLNodelet::onInit();  // NOLINT

    auto privateParam = this->paramsForNodeHandle(*this->pnh_);

    // workaround for https://github.com/ros-perception/perception_pcl/issues/283
    const auto disableOrigTfListener = privateParam->getParam("disable_orig_tf_listener", true);
    {
        const auto cacheLength = this->tf_listener_.getCacheLength();
        this->tf_listener_.~TransformListener();
        ros::NodeHandle nh = this->getNodeHandle();
        // we only disable the original TF listener if the nodelet uses the shared TF buffer
        if (disableOrigTfListener && this->usesSharedBuffer())
        {
            ros::M_string remappings = { {"/tf", "nonexistent"}, {"/tf_static", "nonexistent2"} };
            nh = ros::NodeHandle(nh, "", remappings);
            NODELET_DEBUG("[%s::onInit] Original tf listener was disabled", this->getName().c_str());
        }
        new(&this->tf_listener_) tf::TransformListener(nh, cacheLength);
    }

    // Call the child's local init
    bool has_service = false;
    if (!this->child_init(*this->pnh_, has_service))
    {
        NODELET_ERROR("[%s::onInit] Initialization failed.", this->getName().c_str());
        return;
    }

    // params
    this->publishPeriodically = privateParam->getParam("publish_periodically", false);
    this->fixedFrame = privateParam->getParam("fixed_frame", "");

    if (this->publishPeriodically && this->fixedFrame.empty())
    {
        NODELET_ERROR("[%s::onInit] For periodic publishing, a fixed frame has to be set",
                      this->getName().c_str());
        return;
    }

    this->tfWaitTimeout = privateParam->getParam("tf_wait_timeout", ros::Duration(1), "s");

    const auto produceDiagnostics = privateParam->getParam("produce_diagnostics", true);

    const auto transformChannelsPoint = privateParam->getParam("transform_channels_point", std::vector<std::string>({"vp_"}));
    const auto transformChannelsDirection = privateParam->getParam("transform_channels_direction", std::vector<std::string>({"normal_"}));
    for (const auto& channel : transformChannelsPoint)
      this->transformChannels[channel] = cras::CloudChannelType::POINT;
    for (const auto& channel : transformChannelsDirection)
      this->transformChannels[channel] = cras::CloudChannelType::DIRECTION;

    this->pub_output_ = this->template advertise<PointCloud2>(*this->pnh_, "output", this->max_queue_size_);

    // Enable the dynamic reconfigure service
    if (!has_service)
    {
        this->srv = std::make_unique<dynamic_reconfigure::Server<pcl_ros::FilterConfig>>(*this->pnh_);
        auto f = boost::bind(&Filter::config_callback, this, _1, _2);  // NOLINT
        this->srv->setCallback(f);
    }

    // diagnostics
    this->inputDiag = std::make_unique<cras::TopicDiagnostic>(
        this->pnh_->resolveName("input"), this->getDiagUpdater(), privateParam->paramsInNamespace("receive"));
    this->pubOutputDiag = std::make_unique<cras::DiagnosedPublisher<PointCloud2>>(
        this->pub_output_, this->getDiagUpdater(), privateParam->paramsInNamespace("publish"));

    if (this->publishPeriodically && this->pubOutputDiag->getDesiredRate().expectedCycleTime() == ros::Duration(-1))
    {
        NODELET_ERROR("[%s::onInit] For periodic publishing, publish/rate has to be set",
                      this->getName().c_str());
        return;
    }

    if (produceDiagnostics)
    {
        this->getDiagUpdater().add("TF", this, &ImprovedFilter::produceTfDiag);
        this->startDiagTimer(*this->pnh_);
    }

    if (this->publishPeriodically && this->pubOutputDiag->getDesiredRate().expectedCycleTime().toSec() > 1e-6)
    {
        this->periodicPublishTimer = this->pnh_->createTimer(this->pubOutputDiag->getDesiredRate(),
            &ImprovedFilter::periodicComputePublish, this);
    }

    NODELET_DEBUG("[%s::onInit] Nodelet successfully created.", this->getName().c_str());
}

void ImprovedFilter::input_indices_callback(const sensor_msgs::PointCloud2_<std::allocator<void>>::ConstPtr& cloud,
                                            const ImprovedFilter::PointIndicesConstPtr& indices)
{
    this->updateThreadName();
    this->inputDiag->tick(cloud->header.stamp);

    // If cloud is given, check if it's valid
    if (!this->isValid(cloud))
    {
        NODELET_ERROR("[%s::input_indices_callback] Invalid input!", this->getName().c_str());
        return;
    }
    // If indices are given, check if they are valid
    if (indices && !this->isValid(indices))
    {
        NODELET_ERROR("[%s::input_indices_callback] Invalid indices!", this->getName().c_str());
        return;
    }

    if (indices)
        NODELET_DEBUG("[%s::input_indices_callback]\n"
                      " - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                      " - PointIndices with %zu values, stamp %f, and frame %s on topic %s received.",
                      this->getName().c_str(), cloud->width * cloud->height,
                      pcl::getFieldsList(*cloud).c_str(), cloud->header.stamp.toSec(),
                      cloud->header.frame_id.c_str(), this->pnh_->resolveName("input").c_str(),
                      indices->indices.size(), indices->header.stamp.toSec(),
                      indices->header.frame_id.c_str(),
                      this->pnh_->resolveName("indices").c_str());
    else
        NODELET_DEBUG("[%s::input_indices_callback] PointCloud with %d data points and frame %s on topic %s received.",
            this->getName().c_str(), cloud->width * cloud->height, cloud->header.frame_id.c_str(),
            this->pnh_->resolveName("input").c_str());

    // Check whether the user has given a different input TF frame
    this->tf_input_orig_frame_ = cloud->header.frame_id;
    auto cloud_tf = this->transformAsInput(cloud);
    if (cloud_tf == nullptr)
        return;

    // Need setInputCloud () here because we have to extract x/y/z
    IndicesPtr vindices;
    if (indices)
        vindices.reset(new std::vector<int>(indices->indices));

    if (this->publishPeriodically)
    {
        std::lock_guard<std::mutex> lock(this->lastInputMutex);
        this->lastInput = std::pair(cloud_tf, vindices);
    }
    else
    {
        this->computePublish(cloud_tf, vindices);
    }
}
void ImprovedFilter::periodicComputePublish(const ros::TimerEvent& event)
{
    PointCloud2::ConstPtr input;
    IndicesPtr indices;

    {
        std::lock_guard<std::mutex> lock(this->lastInputMutex);
        if (this->lastInput.first == nullptr)
            return;
        input = this->lastInput.first;
        indices = this->lastInput.second;
    }

    const auto stamp = event.current_expected;

    // this is unlikely, but it can save some CPU time
    if (stamp == input->header.stamp)
    {
        this->computePublish(input, indices);
        return;
    }

  const std::string& frame = this->tf_input_frame_.empty() ? input->header.frame_id : this->tf_input_frame_;
  auto cloud_tf = this->transformPointCloud(frame, stamp, *input, "periodic update");
    if (cloud_tf == nullptr)
        return;

    this->computePublish(cloud_tf, indices);
}
void ImprovedFilter::produceTfDiag(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    if (this->diagHasTfProblems) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "TF problems.");
    } else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "No TF problems.");
    }
    this->diagHasTfProblems = false;
}

ImprovedFilter::ImprovedFilter()
{

}

}