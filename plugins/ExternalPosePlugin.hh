#ifndef GZ_SIM_SYSTEMS_EXTERNALPOSEPLUGIN_HH_
#define GZ_SIM_SYSTEMS_EXTERNALPOSEPLUGIN_HH_

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
namespace systems
{
  /// \brief Receives VisualPose packets over UDP and teleports the attached
  /// model each simulation step. Used for visualization-only worlds where
  /// physics are computed externally (e.g. by bf_sim_bridge + Simulink).
  ///
  /// SDF parameters:
  ///   <listen_port>  UDP port to listen on (default: 9010)
  class ExternalPosePlugin:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
  {
    public: ExternalPosePlugin();
    public: ~ExternalPosePlugin() override;

    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) override;

    private: struct Impl;
    private: std::unique_ptr<Impl> impl;
  };
}
}
}
#endif
