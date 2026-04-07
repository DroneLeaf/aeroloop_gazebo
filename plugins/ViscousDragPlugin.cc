/*
 * ViscousDragPlugin — body-frame aerodynamic drag for gz-sim 8.
 *
 * Transforms world-frame velocity into the link's body frame, applies
 * per-axis linear + quadratic drag, then transforms the resulting force
 * back to world frame.  This lets you set different drag for forward (X),
 * lateral (Y), and vertical (Z) independently — matching real airframe
 * asymmetry (low frontal area, high side area).
 *
 *   F_body_i = -(b_lin_i * v_i  +  b_quad_i * |v_i| * v_i)
 *   τ_i      = -(b_angular * ω_i)   [world frame]
 *
 * SDF usage (inside a <model> element):
 *
 *   <plugin filename="ViscousDragPlugin"
 *           name="gz::sim::systems::ViscousDragPlugin">
 *     <link_name>base_link</link_name>
 *     <!-- Body-frame linear drag  (N·s/m) -->
 *     <linear_damping_x>0.1</linear_damping_x>    <!-- forward -->
 *     <linear_damping_y>0.5</linear_damping_y>    <!-- lateral -->
 *     <linear_damping_z>0.3</linear_damping_z>    <!-- vertical -->
 *     <!-- Body-frame quadratic drag  (N·s²/m²) -->
 *     <quadratic_damping_x>0.008</quadratic_damping_x>
 *     <quadratic_damping_y>0.05</quadratic_damping_y>
 *     <quadratic_damping_z>0.03</quadratic_damping_z>
 *     <!-- World-frame angular drag  (N·m·s/rad) -->
 *     <angular_damping>0.05</angular_damping>
 *   </plugin>
 *
 *   <!-- Scalar shorthand (same value for all axes): -->
 *   <linear_damping>0.3</linear_damping>
 *   <quadratic_damping>0.02</quadratic_damping>
 */

#include <cmath>
#include <string>

#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>

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

        // Per-axis linear damping (body frame)
        if (sdf->HasElement("linear_damping")) {
            double v = sdf->Get<double>("linear_damping");
            linDamp_ = {v, v, v};
        }
        if (sdf->HasElement("linear_damping_x"))
            linDamp_.X(sdf->Get<double>("linear_damping_x"));
        if (sdf->HasElement("linear_damping_y"))
            linDamp_.Y(sdf->Get<double>("linear_damping_y"));
        if (sdf->HasElement("linear_damping_z"))
            linDamp_.Z(sdf->Get<double>("linear_damping_z"));

        // Per-axis quadratic damping (body frame)
        if (sdf->HasElement("quadratic_damping")) {
            double v = sdf->Get<double>("quadratic_damping");
            quadDamp_ = {v, v, v};
        }
        if (sdf->HasElement("quadratic_damping_x"))
            quadDamp_.X(sdf->Get<double>("quadratic_damping_x"));
        if (sdf->HasElement("quadratic_damping_y"))
            quadDamp_.Y(sdf->Get<double>("quadratic_damping_y"));
        if (sdf->HasElement("quadratic_damping_z"))
            quadDamp_.Z(sdf->Get<double>("quadratic_damping_z"));

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
               << "  lin=(" << linDamp_.X() << "," << linDamp_.Y() << "," << linDamp_.Z() << ")"
               << "  quad=(" << quadDamp_.X() << "," << quadDamp_.Y() << "," << quadDamp_.Z() << ")"
               << "  ang=" << angularDamping_ << "\n";
    }

    void PreUpdate(const UpdateInfo &_info,
                   EntityComponentManager &_ecm) override
    {
        if (_info.paused || linkEntity_ == kNullEntity)
            return;

        auto linVel = link_.WorldLinearVelocity(_ecm);
        auto angVel = link_.WorldAngularVelocity(_ecm);
        auto poseOpt = link_.WorldPose(_ecm);

        if (!linVel || !angVel || !poseOpt)
            return;

        // Transform world velocity → body frame
        gz::math::Quaterniond q = poseOpt->Rot();
        gz::math::Quaterniond qInv = q.Inverse();
        gz::math::Vector3d vBody = qInv.RotateVector(*linVel);

        // Per-body-axis: F_i = -(b_lin_i * v_i + b_quad_i * |v_i| * v_i)
        gz::math::Vector3d fBody(
            -(linDamp_.X() * vBody.X() + quadDamp_.X() * std::abs(vBody.X()) * vBody.X()),
            -(linDamp_.Y() * vBody.Y() + quadDamp_.Y() * std::abs(vBody.Y()) * vBody.Y()),
            -(linDamp_.Z() * vBody.Z() + quadDamp_.Z() * std::abs(vBody.Z()) * vBody.Z()));

        // Rotate drag force back to world frame
        gz::math::Vector3d fWorld = q.RotateVector(fBody);

        // Angular damping stays in world frame
        gz::math::Vector3d torque = -*angVel * angularDamping_;

        link_.AddWorldWrench(_ecm, fWorld, torque);
    }

private:
    Model  model_{kNullEntity};
    Entity linkEntity_{kNullEntity};
    Link   link_{kNullEntity};

    gz::math::Vector3d linDamp_  {0.3, 0.3, 0.3};   // N·s/m
    gz::math::Vector3d quadDamp_ {0.02, 0.02, 0.02}; // N·s²/m²
    double angularDamping_ = 0.01;                    // N·m·s/rad
};

}  // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::ViscousDragPlugin,
              gz::sim::System,
              gz::sim::systems::ViscousDragPlugin::ISystemConfigure,
              gz::sim::systems::ViscousDragPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::ViscousDragPlugin,
                    "gz::sim::systems::ViscousDragPlugin")
