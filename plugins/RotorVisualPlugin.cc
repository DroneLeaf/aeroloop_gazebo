// RotorVisualPlugin — receives motor commands over UDP and spins rotor joints
// for pure visualization. Does not affect drone dynamics (the drone pose is
// driven by ExternalPosePlugin + bf_sim_bridge).
//
// Uses JointPositionReset to directly set joint angles each frame, avoiding
// physics constraint solving. The angle is accumulated based on motor command
// × max_velocity × dt. This prevents the physics engine from fighting
// ExternalPosePlugin's WorldPoseCmd teleportation on revolute joints.
//
// SDF parameters:
//   <listen_port>9012</listen_port>         UDP port (default 9012)
//   <max_velocity>100.0</max_velocity>      rad/s at full throttle
//   <joint_0>rotor_0_joint</joint_0>        joint names (default: rotor_N_joint)
//   <spin_dir_0>-1</spin_dir_0>             -1 = CW, +1 = CCW (from above)
//
// Motor command packet (16 bytes): 4 × float32, normalized [0,1].
// Indices map directly to rotor joints: motor[0] → rotor_0_joint, etc.

#include "RotorVisualPlugin.hh"

#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/Name.hh>

#include <gz/plugin/Register.hh>

#include <sdf/sdf.hh>

#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <cstring>
#include <array>
#include <string>

using namespace gz;
using namespace sim;
using namespace systems;

// Must match the packet sent by bf_sim_bridge
struct MotorCmdPacket {
    float motor[4];  // normalized [0,1]
};
static_assert(sizeof(MotorCmdPacket) == 16, "MotorCmdPacket size mismatch");

struct RotorVisualPlugin::Impl {
    Entity modelEntity{kNullEntity};
    std::array<Entity, 4> jointEntities;
    std::array<double, 4> spinDir;   // +1.0 (CCW) or -1.0 (CW) viewed from above
    std::array<double, 4> angle;     // accumulated joint angle (rad)
    double maxVel{100.0};            // rad/s at full throttle
    int fd{-1};
    bool received{false};
    float latestMotor[4] = {0, 0, 0, 0};
};

RotorVisualPlugin::RotorVisualPlugin()
    : impl(std::make_unique<Impl>())
{
    impl->jointEntities.fill(kNullEntity);
    impl->angle.fill(0.0);
    // Default spin directions: rotor 0,1 CW; rotor 2,3 CCW
    impl->spinDir = {-1.0, -1.0, +1.0, +1.0};
}

RotorVisualPlugin::~RotorVisualPlugin()
{
    if (impl->fd >= 0)
        ::close(impl->fd);
}

void RotorVisualPlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
    impl->modelEntity = _entity;

    Model model(_entity);
    if (!model.Valid(_ecm)) {
        gzerr << "[RotorVisualPlugin] Must be attached to a model.\n";
        return;
    }

    // Read parameters from SDF
    uint16_t port = 9012;
    if (_sdf->HasElement("listen_port"))
        port = static_cast<uint16_t>(_sdf->Get<int>("listen_port"));

    if (_sdf->HasElement("max_velocity"))
        impl->maxVel = _sdf->Get<double>("max_velocity");

    // Joint names — default rotor_N_joint, overridable via <joint_N> tags
    std::array<std::string, 4> jointNames = {
        "rotor_0_joint", "rotor_1_joint", "rotor_2_joint", "rotor_3_joint"
    };

    for (int i = 0; i < 4; ++i) {
        std::string nameTag = "joint_" + std::to_string(i);
        if (_sdf->HasElement(nameTag))
            jointNames[i] = _sdf->Get<std::string>(nameTag);

        std::string dirTag = "spin_dir_" + std::to_string(i);
        if (_sdf->HasElement(dirTag))
            impl->spinDir[i] = _sdf->Get<double>(dirTag);

        impl->jointEntities[i] = model.JointByName(_ecm, jointNames[i]);
        if (impl->jointEntities[i] == kNullEntity) {
            gzwarn << "[RotorVisualPlugin] Joint '" << jointNames[i]
                   << "' not found in model\n";
        }
    }

    // Create non-blocking UDP socket
    impl->fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (impl->fd < 0) {
        gzerr << "[RotorVisualPlugin] socket() failed\n";
        return;
    }

    int reuse = 1;
    setsockopt(impl->fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (::bind(impl->fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        gzerr << "[RotorVisualPlugin] bind() failed on port " << port << "\n";
        ::close(impl->fd);
        impl->fd = -1;
        return;
    }

    int flags = fcntl(impl->fd, F_GETFL, 0);
    fcntl(impl->fd, F_SETFL, flags | O_NONBLOCK);

    gzmsg << "[RotorVisualPlugin] Listening on UDP " << port
           << " (max_vel=" << impl->maxVel << " rad/s)\n";
}

void RotorVisualPlugin::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
    if (impl->fd < 0)
        return;

    // Drain all pending packets, keep only the latest
    MotorCmdPacket pkt{};
    bool got_packet = false;

    while (true) {
        MotorCmdPacket tmp{};
        ssize_t n = ::recv(impl->fd, &tmp, sizeof(tmp), MSG_DONTWAIT);
        if (n == static_cast<ssize_t>(sizeof(tmp))) {
            pkt = tmp;
            got_packet = true;
        } else {
            break;
        }
    }

    if (got_packet) {
        if (!impl->received) {
            gzmsg << "[RotorVisualPlugin] First motor command received\n";
            impl->received = true;
        }
        std::memcpy(impl->latestMotor, pkt.motor, sizeof(pkt.motor));
    }

    // Compute dt from simulation step size
    double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0.0)
        return;

    // Accumulate joint angle and set via JointPositionReset (kinematic,
    // bypasses physics constraint solver — no forces computed for these joints)
    for (int i = 0; i < 4; ++i) {
        if (impl->jointEntities[i] == kNullEntity)
            continue;

        double cmd = static_cast<double>(impl->latestMotor[i]);
        if (cmd < 0.0) cmd = 0.0;
        if (cmd > 1.0) cmd = 1.0;

        double vel = cmd * impl->maxVel * impl->spinDir[i];
        impl->angle[i] += vel * dt;

        // Wrap angle to [-π, π] to avoid precision loss over time
        impl->angle[i] = std::fmod(impl->angle[i], 2.0 * M_PI);

        auto posReset = _ecm.Component<components::JointPositionReset>(
            impl->jointEntities[i]);
        if (posReset) {
            *posReset = components::JointPositionReset({impl->angle[i]});
        } else {
            _ecm.CreateComponent(impl->jointEntities[i],
                                 components::JointPositionReset({impl->angle[i]}));
        }
    }
}

GZ_ADD_PLUGIN(RotorVisualPlugin,
              System,
              RotorVisualPlugin::ISystemConfigure,
              RotorVisualPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(RotorVisualPlugin,
                    "gz::sim::systems::RotorVisualPlugin")
