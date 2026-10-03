#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <gz/msgs/pose_v.pb.h>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

namespace scrobot_simulation
{

class ShuttleActivitySystem:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/) override
  {
    this->worldEntity_ = gz::sim::worldEntity(_entity, _ecm);
    if (this->worldEntity_ == gz::sim::kNullEntity)
      this->worldEntity_ = _entity;

    this->robotName_ = _sdf->Get<std::string>("robot_model", this->robotName_).first;
    this->updateRate_ = _sdf->Get<double>("update_rate", this->updateRate_).first;
    this->groundTruthTopic_ = _sdf->Get<std::string>("ground_truth_topic", this->groundTruthTopic_).first;
    this->collectedTopic_ = _sdf->Get<std::string>("collected_topic", this->collectedTopic_).first;
    this->pickupOffsetX_ = _sdf->Get<double>("pickup_offset_x", this->pickupOffsetX_).first;
    this->pickupHalfLength_ = _sdf->Get<double>("pickup_half_length", this->pickupHalfLength_).first;
    this->pickupHalfWidth_ = _sdf->Get<double>("pickup_half_width", this->pickupHalfWidth_).first;
    this->shuttleCenterOffsetZ_ = _sdf->Get<double>(
      "shuttle_center_offset_z", this->shuttleCenterOffsetZ_).first;
    this->enableCollection_ = _sdf->Get<bool>(
      "enable_collection", this->enableCollection_).first;

    if (this->updateRate_ <= 0.0) this->updateRate_ = 10.0;
    if (this->pickupHalfLength_ <= 0.0) this->pickupHalfLength_ = 0.030;
    if (this->pickupHalfWidth_ <= 0.0) this->pickupHalfWidth_ = 0.150;

    this->updatePeriod_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / this->updateRate_));

    this->groundTruthPublisher_ = this->transportNode_.Advertise<gz::msgs::Pose_V>(this->groundTruthTopic_);
    this->collectedPublisher_ = this->transportNode_.Advertise<gz::msgs::Pose_V>(this->collectedTopic_);
    this->ResolveRobot(_ecm);
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || this->worldEntity_ == gz::sim::kNullEntity)
      return;

    if (!this->timeInitialized_)
    {
      this->lastUpdate_ = _info.simTime;
      this->timeInitialized_ = true;
      return;
    }

    if (_info.simTime - this->lastUpdate_ < this->updatePeriod_)
      return;
    this->lastUpdate_ = _info.simTime;

    if (this->robotEntity_ == gz::sim::kNullEntity || !_ecm.HasEntity(this->robotEntity_))
      this->ResolveRobot(_ecm);
    if (this->robotEntity_ == gz::sim::kNullEntity)
      return;

    const auto robotPose = gz::sim::worldPose(this->robotEntity_, _ecm);
    const double robotYaw = robotPose.Rot().Yaw();
    const double cosYaw = std::cos(robotYaw);
    const double sinYaw = std::sin(robotYaw);

    gz::msgs::Pose_V shuttleGroundTruthMsg;
    gz::msgs::Pose_V collectedMsg;

    _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
      [&](const gz::sim::Entity &_entity,
          const gz::sim::components::Model * /*_modelComp*/,
          const gz::sim::components::Name *_nameComp) -> bool
      {
        if (_entity == this->robotEntity_ || _entity == this->worldEntity_)
          return true;

        gz::sim::Model model(_entity);
        if (model.LinkByName(_ecm, "shuttle_link") == gz::sim::kNullEntity)
          return true;

        const auto shuttlePose = gz::sim::worldPose(_entity, _ecm);

        // Collection reference point:
        //   shuttle model origin + 45 mm along the shuttle's local +Z axis.
        // This point follows the shuttle orientation and approximates the
        // geometric middle of the 90 mm shuttle body.
        const auto shuttleCenterOffsetWorld = shuttlePose.Rot().RotateVector(
          gz::math::Vector3d(0.0, 0.0, this->shuttleCenterOffsetZ_));
        const auto shuttleCollectCenter =
          shuttlePose.Pos() + shuttleCenterOffsetWorld;

        const double worldDx = shuttleCollectCenter.X() - robotPose.Pos().X();
        const double worldDy = shuttleCollectCenter.Y() - robotPose.Pos().Y();
        const double localX = cosYaw * worldDx + sinYaw * worldDy;
        const double localY = -sinYaw * worldDx + cosYaw * worldDy;

        // A shuttle is collected only when its collection-center point is
        // inside the 300 mm x 60 mm rectangle centered at collector_link:
        //   x = +0.165 m, y = 0 in the robot frame.
        const bool centerInsidePickupZone =
          std::abs(localX - this->pickupOffsetX_) <= this->pickupHalfLength_ &&
          std::abs(localY) <= this->pickupHalfWidth_;

        if (this->enableCollection_ && centerInsidePickupZone)
        {
          auto *collectedPose = collectedMsg.add_pose();
          this->FillPoseMessage(*collectedPose, _entity, _nameComp->Data(), shuttlePose);
          _ecm.RequestRemoveEntity(_entity);
          return true;
        }

        auto *poseMsg = shuttleGroundTruthMsg.add_pose();
        this->FillPoseMessage(*poseMsg, _entity, _nameComp->Data(), shuttlePose);

        // Ground truth always reports the actual shuttle model pose. The
        // collection decision above is orientation-aware because it derives the
        // collection-center point from the shuttle's local +Z axis.
        return true;
      });

    this->groundTruthPublisher_.Publish(shuttleGroundTruthMsg);
    if (collectedMsg.pose_size() > 0)
      this->collectedPublisher_.Publish(collectedMsg);
  }

private:
  template<typename PoseType>
  void FillPoseMessage(
    PoseType &_poseMsg,
    const gz::sim::Entity &_entity,
    const std::string &_name,
    const gz::math::Pose3d &_pose)
  {
    _poseMsg.set_name(_name);
    _poseMsg.set_id(static_cast<uint64_t>(_entity));
    _poseMsg.mutable_position()->set_x(_pose.Pos().X());
    _poseMsg.mutable_position()->set_y(_pose.Pos().Y());
    _poseMsg.mutable_position()->set_z(_pose.Pos().Z());
    _poseMsg.mutable_orientation()->set_x(_pose.Rot().X());
    _poseMsg.mutable_orientation()->set_y(_pose.Rot().Y());
    _poseMsg.mutable_orientation()->set_z(_pose.Rot().Z());
    _poseMsg.mutable_orientation()->set_w(_pose.Rot().W());
  }

  void ResolveRobot(gz::sim::EntityComponentManager &_ecm)
  {
    this->robotEntity_ = _ecm.EntityByComponents(
      gz::sim::components::Model(),
      gz::sim::components::Name(this->robotName_));
  }

  gz::sim::Entity worldEntity_{gz::sim::kNullEntity};
  gz::sim::Entity robotEntity_{gz::sim::kNullEntity};

  gz::transport::Node transportNode_;
  gz::transport::Node::Publisher groundTruthPublisher_;
  gz::transport::Node::Publisher collectedPublisher_;

  std::string robotName_{"scrobot"};
  std::string groundTruthTopic_{"/evaluation/shuttle_ground_truth_gz"};
  std::string collectedTopic_{"/evaluation/shuttle_collected_gz"};
  double updateRate_{10.0};
  double pickupOffsetX_{0.165};
  double pickupHalfLength_{0.030};
  double pickupHalfWidth_{0.150};
  double shuttleCenterOffsetZ_{0.045};
  bool enableCollection_{true};

  std::chrono::steady_clock::duration updatePeriod_{};
  std::chrono::steady_clock::duration lastUpdate_{};
  bool timeInitialized_{false};
};

}  // namespace scrobot_simulation

GZ_ADD_PLUGIN(
  scrobot_simulation::ShuttleActivitySystem,
  gz::sim::System,
  scrobot_simulation::ShuttleActivitySystem::ISystemConfigure,
  scrobot_simulation::ShuttleActivitySystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  scrobot_simulation::ShuttleActivitySystem,
  "scrobot_simulation::ShuttleActivitySystem")
