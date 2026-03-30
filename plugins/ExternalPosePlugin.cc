// ExternalPosePlugin — receives VisualPose UDP packets from bf_sim_bridge
// and teleports the attached model each PreUpdate step.
//
// VisualPose wire format (72 bytes):
//   uint64_t  seq            (8 bytes)
//   double    t              (8 bytes)
//   double    pos_enu[3]     (24 bytes) — East, North, Up
//   double    quat_wxyz[4]   (32 bytes) — w, x, y, z

#include "ExternalPosePlugin.hh"

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
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
#include <cstring>

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

struct ExternalPosePlugin::Impl {
    Entity modelEntity{kNullEntity};
    int fd{-1};
    bool received{false};
    math::Pose3d latestPose;
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
    VisualPosePacket pkt{};
    bool got_packet = false;

    while (true) {
        VisualPosePacket tmp{};
        ssize_t n = ::recv(impl->fd, &tmp, sizeof(tmp), MSG_DONTWAIT);
        if (n == static_cast<ssize_t>(sizeof(tmp))) {
            pkt = tmp;
            got_packet = true;
        } else {
            break;
        }
    }

    if (!got_packet)
        return;

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

GZ_ADD_PLUGIN(ExternalPosePlugin,
              System,
              ExternalPosePlugin::ISystemConfigure,
              ExternalPosePlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ExternalPosePlugin,
                    "gz::sim::systems::ExternalPosePlugin")
