// ExternalPosePlugin — receives VisualPose UDP packets from bf_sim_bridge
// and teleports the attached model each PreUpdate step.
//
// VisualPose wire format (72 bytes):
//   uint64_t  seq            (8 bytes)
//   double    t              (8 bytes)
//   double    pos_enu[3]     (24 bytes) — East, North, Up
//   double    quat_wxyz[4]   (32 bytes) — w, x, y, z
//
// Optional joint positions: a datagram of 72 + 8·k bytes appends k doubles
// (rad), applied in order to the model's joints named by <joint> elements
// (e.g. the camera gimbal's roll + pitch joints, bf_sim_bridge --gimbal).
// They ride the pose datagram so the drone pose and the joint angles of the
// SAME bridge step land in the same PreUpdate. The latest values are re-set
// every step (JointPositionReset + a zero JointVelocityReset, kinematic like
// RotorVisualPlugin). A 72-byte datagram leaves the joints where they are.

#include "ExternalPosePlugin.hh"

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <gz/plugin/Register.hh>

#include <sdf/sdf.hh>

#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

using namespace gz;
using namespace sim;
using namespace systems;

// VisualPose packet — must match bf_sim_bridge/visual_publisher.h
struct VisualPosePacket {
    uint64_t seq;
    double   t;
    double   pos_enu[3];
    double   quat_wxyz[4];
};
static_assert(sizeof(VisualPosePacket) == 72, "VisualPosePacket size mismatch");
static constexpr size_t kMaxJoints = 16;

struct ExternalPosePlugin::Impl {
    Entity modelEntity{kNullEntity};
    int fd{-1};
    bool received{false};
    math::Pose3d latestPose;
    // Optional <joint> list (in datagram order) and their latest positions.
    std::vector<std::string> jointNames;
    std::vector<Entity> joints;
    std::vector<double> jointPos;
    bool jointsReceived{false};
};

ExternalPosePlugin::ExternalPosePlugin()
    : impl(std::make_unique<Impl>()) {}

ExternalPosePlugin::~ExternalPosePlugin() {
    if (impl->fd >= 0)
        ::close(impl->fd);
}

void ExternalPosePlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
    impl->modelEntity = _entity;

    Model model(_entity);
    if (!model.Valid(_ecm)) {
        gzerr << "[ExternalPosePlugin] Must be attached to a model.\n";
        return;
    }

    // Optional joints driven by the trailing doubles of the datagram.
    if (_sdf->HasElement("joint")) {
        auto sdfClone = _sdf->Clone();
        for (auto e = sdfClone->GetElement("joint"); e; e = e->GetNextElement("joint")) {
            const std::string name = e->Get<std::string>();
            if (impl->jointNames.size() >= kMaxJoints) break;
            impl->jointNames.push_back(name);
            Entity j = model.JointByName(_ecm, name);
            if (j == kNullEntity)
                gzwarn << "[ExternalPosePlugin] Joint '" << name << "' not found in model\n";
            impl->joints.push_back(j);
        }
        impl->jointPos.assign(impl->joints.size(), 0.0);
        gzmsg << "[ExternalPosePlugin] " << impl->joints.size()
              << " joint(s) driven from the pose datagram\n";
    }

    // Read listen port from SDF, default 9010
    uint16_t port = 9010;
    if (_sdf->HasElement("listen_port"))
        port = static_cast<uint16_t>(_sdf->Get<int>("listen_port"));

    // Create non-blocking UDP socket
    impl->fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (impl->fd < 0) {
        gzerr << "[ExternalPosePlugin] socket() failed\n";
        return;
    }

    int reuse = 1;
    setsockopt(impl->fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (::bind(impl->fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        gzerr << "[ExternalPosePlugin] bind() failed on port " << port << "\n";
        ::close(impl->fd);
        impl->fd = -1;
        return;
    }

    // Set non-blocking
    int flags = fcntl(impl->fd, F_GETFL, 0);
    fcntl(impl->fd, F_SETFL, flags | O_NONBLOCK);

    gzmsg << "[ExternalPosePlugin] Listening on UDP " << port
           << " for VisualPose packets\n";
}

void ExternalPosePlugin::PreUpdate(
    const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
    if (impl->fd < 0)
        return;

    // Drain all pending packets, keep only the latest
    uint8_t buf[sizeof(VisualPosePacket) + 8 * kMaxJoints];
    VisualPosePacket pkt{};
    bool got_packet = false;
    while (true) {
        ssize_t n = ::recv(impl->fd, buf, sizeof(buf), MSG_DONTWAIT);
        if (n < static_cast<ssize_t>(sizeof(VisualPosePacket)))
            break;
        const size_t extra = static_cast<size_t>(n) - sizeof(VisualPosePacket);
        if (extra % 8 != 0)
            continue;   // not a pose datagram
        std::memcpy(&pkt, buf, sizeof(pkt));
        got_packet = true;
        const size_t k = std::min(extra / 8, impl->jointPos.size());
        if (k > 0) {
            std::memcpy(impl->jointPos.data(), buf + sizeof(VisualPosePacket), 8 * k);
            if (!impl->jointsReceived)
                gzmsg << "[ExternalPosePlugin] First joint positions received\n";
            impl->jointsReceived = true;
        }
    }

    if (got_packet) {
        if (!impl->received) {
            gzmsg << "[ExternalPosePlugin] First VisualPose received (seq="
                  << pkt.seq << " t=" << pkt.t << ")\n";
            impl->received = true;
        }

        // Convert VisualPose (ENU) to gz::math::Pose3d
        // Gazebo Harmonic world frame is ENU, so position maps directly.
        math::Vector3d pos(pkt.pos_enu[0], pkt.pos_enu[1], pkt.pos_enu[2]);
        // Quaternion: VisualPose sends [w, x, y, z], gz::math::Quaterniond takes (w, x, y, z)
        math::Quaterniond quat(pkt.quat_wxyz[0], pkt.quat_wxyz[1],
                               pkt.quat_wxyz[2], pkt.quat_wxyz[3]);
        impl->latestPose.Set(pos, quat);

        // Teleport the model via WorldPoseCmd
        auto cmd = _ecm.Component<components::WorldPoseCmd>(impl->modelEntity);
        if (cmd) {
            *cmd = components::WorldPoseCmd(impl->latestPose);
        } else {
            _ecm.CreateComponent(impl->modelEntity,
                                 components::WorldPoseCmd(impl->latestPose));
        }
    }

    // Joints: the latest positions, every step (kinematic; no forces).
    if (!impl->jointsReceived)
        return;
    for (size_t i = 0; i < impl->joints.size(); ++i) {
        const Entity j = impl->joints[i];
        if (j == kNullEntity)
            continue;
        auto posReset = _ecm.Component<components::JointPositionReset>(j);
        if (posReset)
            *posReset = components::JointPositionReset({impl->jointPos[i]});
        else
            _ecm.CreateComponent(j, components::JointPositionReset({impl->jointPos[i]}));
        auto velReset = _ecm.Component<components::JointVelocityReset>(j);
        if (velReset)
            *velReset = components::JointVelocityReset({0.0});
        else
            _ecm.CreateComponent(j, components::JointVelocityReset({0.0}));
    }
}

GZ_ADD_PLUGIN(ExternalPosePlugin,
              System,
              ExternalPosePlugin::ISystemConfigure,
              ExternalPosePlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ExternalPosePlugin,
                    "gz::sim::systems::ExternalPosePlugin")
