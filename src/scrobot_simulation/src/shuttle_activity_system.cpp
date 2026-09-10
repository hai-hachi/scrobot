#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>

#include <gz/msgs/pose_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Static.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

namespace scrobot_simulation
{

class ShuttleActivitySystem:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
private:
  struct ShuttleState
  {
    std::chrono::steady_clock::duration lastProtectedTime{};
    bool initialized{false};
    bool requestedStatic{false};
  };

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

    this->robotName_ = _sdf->Get<std::string>(
      "robot_model", this->robotName_).first;
    this->updateRate_ = _sdf->Get<double>(
      "update_rate", this->updateRate_).first;
    this->activationDistance_ = _sdf->Get<double>(
      "activation_distance", this->activationDistance_).first;
    this->freezeDistance_ = _sdf->Get<double>(
      "freeze_distance", this->freezeDistance_).first;
    this->settleTime_ = _sdf->Get<double>(
      "settle_time", this->settleTime_).first;
    this->groundTruthTopic_ = _sdf->Get<std::string>(
      "ground_truth_topic", this->groundTruthTopic_).first;

    if (this->updateRate_ <= 0.0)
      this->updateRate_ = 10.0;
    if (this->activationDistance_ < 0.0)
      this->activationDistance_ = 0.0;
    if (this->freezeDistance_ < this->activationDistance_)
      this->freezeDistance_ = this->activationDistance_;
    if (this->settleTime_ < 0.0)
      this->settleTime_ = 0.0;

    this->updatePeriod_ = std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / this->updateRate_));
    this->settleDuration_ = std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(this->settleTime_));

    this->groundTruthPublisher_ =
      this->transportNode_.Advertise<gz::msgs::Pose_V>(this->groundTruthTopic_);

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

    if (this->robotEntity_ == gz::sim::kNullEntity ||
        !_ecm.HasEntity(this->robotEntity_))
    {
      this->ResolveRobot(_ecm);
    }

    if (this->robotEntity_ == gz::sim::kNullEntity)
      return;

    const auto robotPose = gz::sim::worldPose(this->robotEntity_, _ecm);
    gz::msgs::Pose_V shuttleGroundTruthMsg;

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

        auto &state = this->states_[_entity];
        if (!state.initialized)
        {
          state.lastProtectedTime = _info.simTime;
          state.initialized = true;
        }

        const auto shuttlePose = gz::sim::worldPose(_entity, _ecm);

        // Dedicated shuttle-only ground truth. The Gazebo pose name is useful
        // for Gazebo-side debugging, but the ROS bridge converts this Pose_V to
        // a PoseArray, where identity is intentionally not required.
        auto *poseMsg = shuttleGroundTruthMsg.add_pose();
        poseMsg->set_name(_nameComp->Data());
        poseMsg->set_id(static_cast<uint64_t>(_entity));
        poseMsg->mutable_position()->set_x(shuttlePose.Pos().X());
        poseMsg->mutable_position()->set_y(shuttlePose.Pos().Y());
        poseMsg->mutable_position()->set_z(shuttlePose.Pos().Z());
        poseMsg->mutable_orientation()->set_x(shuttlePose.Rot().X());
        poseMsg->mutable_orientation()->set_y(shuttlePose.Rot().Y());
        poseMsg->mutable_orientation()->set_z(shuttlePose.Rot().Z());
        poseMsg->mutable_orientation()->set_w(shuttlePose.Rot().W());

        const double dx = shuttlePose.Pos().X() - robotPose.Pos().X();
        const double dy = shuttlePose.Pos().Y() - robotPose.Pos().Y();
        const double distance = std::hypot(dx, dy);

        const bool staticNow = state.requestedStatic || model.Static(_ecm);

        if (staticNow)
        {
          if (distance <= this->activationDistance_)
          {
            this->SetModelStatic(_entity, false, _ecm);
            state.requestedStatic = false;
            state.lastProtectedTime = _info.simTime;
          }
          return true;
        }

        if (distance <= this->freezeDistance_)
        {
          state.lastProtectedTime = _info.simTime;
          return true;
        }

        if (_info.simTime - state.lastProtectedTime >= this->settleDuration_)
        {
          this->SetModelStatic(_entity, true, _ecm);
          state.requestedStatic = true;
        }

        return true;
      });

    // Publish even when no shuttles exist. An empty PoseArray on the ROS side
    // is a valid "zero shuttles" state and lets consumers distinguish that
    // from a missing ground-truth stream.
    this->groundTruthPublisher_.Publish(shuttleGroundTruthMsg);
  }

private:
  void SetModelStatic(
    const gz::sim::Entity &_entity,
    const bool _static,
    gz::sim::EntityComponentManager &_ecm)
  {
    auto staticComp =
      _ecm.Component<gz::sim::components::Static>(_entity);

    if (staticComp)
    {
      staticComp->SetData(
        _static,
        [](const bool &_oldValue, const bool &_newValue)
        {
          return _oldValue != _newValue;
        });
    }
    else
    {
      _ecm.CreateComponent(
        _entity,
        gz::sim::components::Static(_static));
    }
  }

  void ResolveRobot(gz::sim::EntityComponentManager &_ecm)
  {
    this->robotEntity_ = _ecm.EntityByComponents(
      gz::sim::components::Model(),
      gz::sim::components::Name(this->robotName_));
  }

  gz::sim::Entity worldEntity_{gz::sim::kNullEntity};
  gz::sim::Entity robotEntity_{gz::sim::kNullEntity};
  std::unordered_map<gz::sim::Entity, ShuttleState> states_;

  gz::transport::Node transportNode_;
  gz::transport::Node::Publisher groundTruthPublisher_;

  std::string robotName_{"scrobot"};
  std::string groundTruthTopic_{"/evaluation/shuttle_ground_truth_gz"};
  double updateRate_{10.0};
  double activationDistance_{1.2};
  double freezeDistance_{1.6};
  double settleTime_{1.25};

  std::chrono::steady_clock::duration updatePeriod_{};
  std::chrono::steady_clock::duration settleDuration_{};
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
