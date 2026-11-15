// Planner Lab only: smooth Gazebo-time A<->B motion for a static
// camera-visible, collision-enabled cylinder. No ROS nodes/subprocesses.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include <sdf/Element.hh>

namespace scrobot_simulation
{
class Nav2MovingCylinderSystem :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &,
      gz::sim::EventManager &) override
  {
    this->model_ = gz::sim::Model(_entity);
    this->ax_ = _sdf->Get<double>("ax", 0.0).first;
    this->ay_ = _sdf->Get<double>("ay", 0.0).first;
    this->bx_ = _sdf->Get<double>("bx", 0.0).first;
    this->by_ = _sdf->Get<double>("by", 0.0).first;
    this->speed_ = _sdf->Get<double>("speed_mps", 0.6).first;
    this->length_ = std::hypot(this->bx_ - this->ax_,
                              this->by_ - this->ay_);
    this->valid_ = std::isfinite(this->length_) &&
      this->length_ >= 0.2 && std::isfinite(this->speed_) &&
      this->speed_ > 0.0 && this->speed_ <= 2.0;
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || !this->valid_ || !this->model_.Valid(_ecm))
      return;
    const double now =
      std::chrono::duration<double>(_info.simTime).count();
    if (!this->initialized_ || now < this->startSim_)
    {
      this->startSim_ = now;
      this->lastUpdate_ = now - 1.0;
      this->initialized_ = true;
    }

    // At most 60 fixed pose updates per simulated second, independent
    // of the physics step rate. This is smooth without issuing 1000+
    // pose commands/second for each moving obstacle.
    if (now - this->lastUpdate_ < 1.0 / 60.0)
      return;
    this->lastUpdate_ = now;

    // The specified speed is the average traversal speed on each leg.
    // Cosine easing makes the cylinder decelerate to zero at both ends,
    // instead of instantaneously reversing direction. Each pose command
    // is applied by Gazebo on simulation update (no CLI/clock JSON jitter).
    const double secondsPerLeg = this->length_ / this->speed_;
    const double phase = std::fmod(
      std::max(0.0, now - this->startSim_) / secondsPerLeg, 2.0);
    const double along = phase <= 1.0 ? phase : 2.0 - phase;
    const double eased = 0.5 * (1.0 - std::cos(3.141592653589793 * along));
    const double x = this->ax_ + (this->bx_ - this->ax_) * eased;
    const double y = this->ay_ + (this->by_ - this->ay_) * eased;

    this->model_.SetWorldPoseCmd(_ecm, gz::math::Pose3d(x, y, 0, 0, 0, 0));
  }

private:
  gz::sim::Model model_;
  double ax_{0}, ay_{0}, bx_{0}, by_{0}, speed_{0.6};
  double length_{0};
  double startSim_{0};
  double lastUpdate_{-1.0};
  bool initialized_{false};
  bool valid_{false};
};
}

GZ_ADD_PLUGIN(
  scrobot_simulation::Nav2MovingCylinderSystem,
  gz::sim::System,
  scrobot_simulation::Nav2MovingCylinderSystem::ISystemConfigure,
  scrobot_simulation::Nav2MovingCylinderSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  scrobot_simulation::Nav2MovingCylinderSystem,
  "scrobot_simulation::Nav2MovingCylinderSystem")
