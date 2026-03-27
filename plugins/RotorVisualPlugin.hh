#ifndef GZ_SIM_SYSTEMS_ROTORVISUALPLUGIN_HH_
#define GZ_SIM_SYSTEMS_ROTORVISUALPLUGIN_HH_

#include <gz/sim/System.hh>

namespace gz {
namespace sim {
namespace systems {

  /// Receives motor commands over UDP and spins rotor joints for visualization.
  /// Used with ExternalPosePlugin in visualization-only worlds — the rotors
  /// spin visually without affecting the drone's externally-driven pose.
  class RotorVisualPlugin:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
  {
    public: RotorVisualPlugin();
    public: ~RotorVisualPlugin() override;

    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) override;

    private: struct Impl;
    private: std::unique_ptr<Impl> impl;
  };

}  // namespace systems
}  // namespace sim
}  // namespace gz

#endif
