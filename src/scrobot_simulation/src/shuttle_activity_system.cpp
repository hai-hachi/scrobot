#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

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
    this->shuttleEntity_ = _entity;
    this->shuttleModel_ = gz::sim::Model(_entity);

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

    // Resolve immediately if the robot already exists. If not, PreUpdate will
    // keep trying; this also supports shuttles spawned before the robot.
    this->ResolveRobot(_ecm);
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || this->shuttleEntity_ == gz::sim::kNullEntity)
      return;

    if (!this->timeInitialized_)
    {
      this->lastUpdate_ = _info.simTime;
      this->lastProtectedTime_ = _info.simTime;
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

    double robotDistance = 1.0e9;
    if (this->robotEntity_ != gz::sim::kNullEntity)
    {
      const auto shuttlePose = gz::sim::worldPose(this->shuttleEntity_, _ecm);
      const auto robotPose = gz::sim::worldPose(this->robotEntity_, _ecm);
      const double dx = shuttlePose.Pos().X() - robotPose.Pos().X();
      const double dy = shuttlePose.Pos().Y() - robotPose.Pos().Y();
      robotDistance = std::hypot(dx, dy);
    }

    const bool staticNow = this->requestedStatic_ ||
      this->shuttleModel_.Static(_ecm);

    if (staticNow)
    {
      // Wake well before the robot can physically touch the shuttle.
      if (robotDistance <= this->activationDistance_)
      {
        this->shuttleModel_.SetStatic(_ecm, false);
        this->requestedStatic_ = false;
        this->lastProtectedTime_ = _info.simTime;
      }
      return;
    }

    // While the robot is nearby, never freeze. This timestamp also creates a
    // settle delay after the robot moves away following a push.
    if (robotDistance <= this->freezeDistance_)
    {
      this->lastProtectedTime_ = _info.simTime;
      return;
    }

    // Far-away dynamic shuttles are allowed to settle, then converted to
    // static bodies. Their visual and world pose are preserved, but physics no
    // longer integrates them until the robot comes near again.
    if (_info.simTime - this->lastProtectedTime_ >= this->settleDuration_)
    {
      this->shuttleModel_.SetStatic(_ecm, true);
      this->requestedStatic_ = true;
    }
  }

private:
  void ResolveRobot(gz::sim::EntityComponentManager &_ecm)
  {
    this->robotEntity_ = _ecm.EntityByComponents(
      gz::sim::components::Model(),
      gz::sim::components::Name(this->robotName_));
  }

  gz::sim::Entity shuttleEntity_{gz::sim::kNullEntity};
  gz::sim::Entity robotEntity_{gz::sim::kNullEntity};
  gz::sim::Model shuttleModel_;

  std::string robotName_{"scrobot"};
  double updateRate_{10.0};
  double activationDistance_{1.2};
  double freezeDistance_{1.6};
  double settleTime_{1.25};

  std::chrono::steady_clock::duration updatePeriod_{};
  std::chrono::steady_clock::duration settleDuration_{};
  std::chrono::steady_clock::duration lastUpdate_{};
  std::chrono::steady_clock::duration lastProtectedTime_{};

  bool timeInitialized_{false};
  bool requestedStatic_{false};
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
