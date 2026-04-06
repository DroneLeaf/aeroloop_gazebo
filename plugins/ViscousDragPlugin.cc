/*
 * ViscousDragPlugin — linear velocity-proportional drag for gz-sim 8.
 *
 * Applies F = -b_linear * v  and  τ = -b_angular * ω  to a link every
 * simulation step.  This models low-speed aerodynamic damping (prop wash,
 * H-force, blade flapping) that flat-plate quadratic LiftDrag plugins
 * cannot capture.
 *
 * SDF usage (inside a <model> element):
 *
 *   <plugin filename="ViscousDragPlugin"
 *           name="gz::sim::systems::ViscousDragPlugin">
 *     <link_name>base_link</link_name>
 *     <linear_damping>0.6</linear_damping>    <!-- N·s/m -->
 *     <angular_damping>0.3</angular_damping>   <!-- N·m·s/rad -->
 *   </plugin>
 */

#include <string>

#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>

namespace gz::sim::systems
{

class ViscousDragPlugin
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate
{
public:
    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &) override
    {
        auto sdf = std::const_pointer_cast<sdf::Element>(_sdf);

        model_ = Model(_entity);
        if (!model_.Valid(_ecm)) {
            gzerr << "[ViscousDragPlugin] Attached to invalid model\n";
            return;
        }

        std::string linkName = "base_link";
        if (sdf->HasElement("link_name"))
            linkName = sdf->Get<std::string>("link_name");

        if (sdf->HasElement("linear_damping"))
            linearDamping_ = sdf->Get<double>("linear_damping");

        if (sdf->HasElement("angular_damping"))
            angularDamping_ = sdf->Get<double>("angular_damping");

        // Resolve the link entity
        linkEntity_ = model_.LinkByName(_ecm, linkName);
        if (linkEntity_ == kNullEntity) {
            gzerr << "[ViscousDragPlugin] Link '" << linkName << "' not found\n";
            return;
        }

        link_ = Link(linkEntity_);
        link_.EnableVelocityChecks(_ecm, true);

        gzmsg << "[ViscousDragPlugin] " << linkName
               << "  linear_damping=" << linearDamping_
               << "  angular_damping=" << angularDamping_ << "\n";
    }

    void PreUpdate(const UpdateInfo &_info,
                   EntityComponentManager &_ecm) override
    {
        if (_info.paused || linkEntity_ == kNullEntity)
            return;

        auto linVel = link_.WorldLinearVelocity(_ecm);
        auto angVel = link_.WorldAngularVelocity(_ecm);

        if (linVel && angVel)
        {
            gz::math::Vector3d force  = -*linVel * linearDamping_;
            gz::math::Vector3d torque = -*angVel * angularDamping_;

            link_.AddWorldWrench(_ecm, force, torque);
        }
    }

private:
    Model  model_{kNullEntity};
    Entity linkEntity_{kNullEntity};
    Link   link_{kNullEntity};

    double linearDamping_  = 0.5;   // N·s/m   (default)
    double angularDamping_ = 0.2;   // N·m·s/rad
};

}  // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::ViscousDragPlugin,
              gz::sim::System,
              gz::sim::systems::ViscousDragPlugin::ISystemConfigure,
              gz::sim::systems::ViscousDragPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::ViscousDragPlugin,
                    "gz::sim::systems::ViscousDragPlugin")
