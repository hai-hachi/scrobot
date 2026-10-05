#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <gz/msgs/boolean.pb.h>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/WorldPoseCmd.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

namespace scrobot_debug
{

class DatasetSceneFreezerSystem:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &) override
  {
    this->modelPrefix_ =
      _sdf->Get<std::string>("model_prefix", this->modelPrefix_).first;
    this->service_ =
      _sdf->Get<std::string>("service", this->service_).first;

    this->node_.Advertise(
      this->service_,
      &DatasetSceneFreezerSystem::OnFreeze,
      this);
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (this->freezeRequested_.exchange(false))
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->frozen_.clear();

      _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Model *,
            const gz::sim::components::Name *_name) -> bool
        {
          if (_name->Data().rfind(this->modelPrefix_, 0) != 0)
            return true;

          this->frozen_[_entity] = gz::sim::worldPose(_entity, _ecm);
          return true;
        });

      this->active_ = !this->frozen_.empty();
    }

    if (!this->active_)
      return;

    std::lock_guard<std::mutex> lock(this->mutex_);
    for (const auto &[entity, pose] : this->frozen_)
    {
      if (!_ecm.HasEntity(entity))
        continue;

      auto poseCmd =
        _ecm.Component<gz::sim::components::WorldPoseCmd>(entity);
      if (poseCmd)
        poseCmd->SetData(pose, [](const auto &, const auto &) {return false;});
      else
        _ecm.CreateComponent(
          entity, gz::sim::components::WorldPoseCmd(pose));

      gz::sim::Model model(entity);
      const auto link = model.LinkByName(_ecm, "shuttle_link");
      if (link == gz::sim::kNullEntity)
        continue;

      const gz::math::Vector3d zero{0.0, 0.0, 0.0};

      auto linear =
        _ecm.Component<gz::sim::components::LinearVelocityCmd>(link);
      if (linear)
        linear->SetData(zero, [](const auto &, const auto &) {return false;});
      else
        _ecm.CreateComponent(
          link, gz::sim::components::LinearVelocityCmd(zero));

      auto angular =
        _ecm.Component<gz::sim::components::AngularVelocityCmd>(link);
      if (angular)
        angular->SetData(zero, [](const auto &, const auto &) {return false;});
      else
        _ecm.CreateComponent(
          link, gz::sim::components::AngularVelocityCmd(zero));
    }
  }

private:
  bool OnFreeze(
    const gz::msgs::Boolean &_req,
    gz::msgs::Boolean &_rep)
  {
    if (_req.data())
      this->freezeRequested_.store(true);
    _rep.set_data(true);
    return true;
  }

  gz::transport::Node node_;
  std::string modelPrefix_{"yolo_shuttle_"};
  std::string service_{"/yolo/freeze_shuttles"};

  std::atomic<bool> freezeRequested_{false};
  bool active_{false};
  std::mutex mutex_;
  std::unordered_map<gz::sim::Entity, gz::math::Pose3d> frozen_;
};

}  // namespace scrobot_debug

GZ_ADD_PLUGIN(
  scrobot_debug::DatasetSceneFreezerSystem,
  gz::sim::System,
  scrobot_debug::DatasetSceneFreezerSystem::ISystemConfigure,
  scrobot_debug::DatasetSceneFreezerSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  scrobot_debug::DatasetSceneFreezerSystem,
  "scrobot_debug::DatasetSceneFreezerSystem")
