/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef GZ_SIM_SYSTEMS_BETAFLIGHTPLUGIN_HH_
#define GZ_SIM_SYSTEMS_BETAFLIGHTPLUGIN_HH_

#include <memory>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
namespace systems
{
  // Forward declare private data class
  class BetaflightPluginPrivate;

  /// \brief Interface Betaflight from betaflight stack
  /// modeled after SITL/SIM_*
  ///
  /// The plugin requires the following parameters:
  /// <rotor>       rotor description block
  ///    id         attribute rotor id
  ///    <vel_p_gain>       velocity pid p gain
  ///    <vel_i_gain>       velocity pid i gain
  ///    <vel_d_gain>       velocity pid d gain
  ///    <vel_i_max>        velocity pid max integral correction
  ///    <vel_i_min>        velocity pid min integral correction
  ///    <vel_cmd_max>      velocity pid max command torque
  ///    <vel_cmd_min>      velocity pid min command torque
  ///    <jointName>        rotor motor joint, torque applied here
  ///    <turningDirection> turning direction, 'cw' or 'ccw'
  ///    <rotorVelocitySlowdownSim> experimental, not needed
  /// <imuName>     scoped name for the imu sensor
  /// <connectionTimeoutMaxCount> timeout before giving up on
  ///                             controller synchronization
  class BetaflightPlugin:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
  {
    /// \brief Constructor.
    public: BetaflightPlugin();

    /// \brief Destructor.
    public: ~BetaflightPlugin() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    private: std::unique_ptr<BetaflightPluginPrivate> dataPtr;
  };
}
}
}
#endif
