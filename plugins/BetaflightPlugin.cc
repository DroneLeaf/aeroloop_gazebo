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
#include <functional>
#include <fcntl.h>

#ifdef _WIN32
  #include <Winsock2.h>
  #include <Ws2def.h>
  #include <Ws2ipdef.h>
  #include <Ws2tcpip.h>
  using raw_type = char;
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  using raw_type = void;
#endif

#if defined(_MSC_VER)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

#include <mutex>
#include <string>
#include <vector>
#include <chrono>

#include <sdf/sdf.hh>
#include <gz/math/Filter.hh>
#include <gz/math/PID.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/plugin/Register.hh>

#include "BetaflightPlugin.hh"

#define MAX_MOTORS 255
#define RADSEC2RPM 9.549296585513721

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Obtains a parameter from sdf.
/// \param[in] _sdf Pointer to the sdf object.
/// \param[in] _name Name of the parameter.
/// \param[out] _param Param Variable to write the parameter to.
/// \param[in] _default_value Default value, if the parameter not available.
/// \param[in] _verbose If true, gzerr if the parameter is not available.
/// \return True if the parameter was found in _sdf, false otherwise.
template<class T>
bool getSdfParam(std::shared_ptr<const sdf::Element> _sdf, const std::string &_name,
  T &_param, const T &_defaultValue, const bool &_verbose = false)
{
  if (_sdf->HasElement(_name))
  {
    // Use const_pointer_cast to work with const sdf::Element
    auto element = std::const_pointer_cast<sdf::Element>(
        std::const_pointer_cast<sdf::Element>(_sdf)->GetElement(_name));
    _param = element->Get<T>();
    return true;
  }

  _param = _defaultValue;
  if (_verbose)
  {
    gzerr << "[BetaflightPlugin] Please specify a value for parameter ["
      << _name << "].\n";
  }
  return false;
}

/// \brief A servo packet.
struct ServoPacket
{
  /// \brief Motor speed data.
  float motorSpeed[MAX_MOTORS];
};

/// \brief Flight Dynamics Model packet that is sent back to Betaflight
struct fdmPacket
{
  /// \brief packet timestamp
  double timestamp;

  /// \brief IMU angular velocity
  double imuAngularVelocityRPY[3];

  /// \brief IMU linear acceleration
  double imuLinearAccelerationXYZ[3];

  /// \brief IMU quaternion orientation
  double imuOrientationQuat[4];

  /// \brief Model velocity: NED frame, or ENU (East,North,Up) when
  ///        spherical coordinates are configured (virtual GPS mode)
  double velocityXYZ[3];

  /// \brief Model position: NED frame, or (lon_deg,lat_deg,alt_m) when
  ///        spherical coordinates are configured (virtual GPS mode)
  double positionXYZ[3];

  double escTemperature[4];
  double escVoltage[4];
  double escCurrent[4];
  double escConsumption[4];
  double escRpm[4];
};

/// \brief Rotor class
class Rotor
{
  /// \brief Constructor
  public: Rotor()
  {
    // most of these coefficients are not used yet.
    this->rotorVelocitySlowdownSim = this->kDefaultRotorVelocitySlowdownSim;
    this->frequencyCutoff = this->kDefaultFrequencyCutoff;
    this->samplingRate = this->kDefaultSamplingRate;

    this->pid.Init(0.1, 0, 0, 0, 0, 1.0, -1.0);
  }

  /// \brief rotor id
  public: int id = 0;

  /// \brief Max rotor propeller RPM.
  public: double maxRpm = 838.0;

  /// \brief Next command to be applied to the propeller
  public: double cmd = 0;

  /// \brief Velocity PID for motor control
  public: gz::math::PID pid;

  /// \brief Control propeller joint name.
  public: std::string jointName;

  /// \brief Control propeller joint entity.
  public: Entity jointEntity;

  /// \brief direction multiplier for this rotor
  public: double multiplier = 1;

  /// \brief unused coefficients
  public: double rotorVelocitySlowdownSim;
  public: double frequencyCutoff;
  public: double samplingRate;
  public: gz::math::OnePole<double> velocityFilter;

  public: static double kDefaultRotorVelocitySlowdownSim;
  public: static double kDefaultFrequencyCutoff;
  public: static double kDefaultSamplingRate;
};

double Rotor::kDefaultRotorVelocitySlowdownSim = 10.0;
double Rotor::kDefaultFrequencyCutoff = 5.0;
double Rotor::kDefaultSamplingRate = 0.2;

// Private data class
class gz::sim::systems::BetaflightPluginPrivate
{
  /// \brief Bind to an address and port
  /// \param[in] _address Address to bind to.
  /// \param[in] _port Port to bind to.
  /// \return True on success.
  public: bool Bind(const char *_address, const uint16_t _port)
  {
    struct sockaddr_in sockaddr;
    this->MakeSockAddr(_address, _port, sockaddr);

    if (bind(this->handle, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
    {
      shutdown(this->handle, 0);
      #ifdef _WIN32
      closesocket(this->handle);
      #else
      close(this->handle);
      #endif
      return false;
    }
    return true;
  }

  /// \brief Make a socket
  /// \param[in] _address Socket address.
  /// \param[in] _port Socket port
  /// \param[out] _sockaddr New socket address structure.
  public: void MakeSockAddr(const char *_address, const uint16_t _port,
    struct sockaddr_in &_sockaddr) const
  {
    memset(&_sockaddr, 0, sizeof(_sockaddr));

    #ifdef HAVE_SOCK_SIN_LEN
      _sockaddr.sin_len = sizeof(_sockaddr);
    #endif

    _sockaddr.sin_port = htons(_port);
    _sockaddr.sin_family = AF_INET;
    _sockaddr.sin_addr.s_addr = inet_addr(_address);
  }

  /// \brief Receive data
  /// \param[out] _buf Buffer that receives the data.
  /// \param[in] _size Size of the buffer.
  /// \param[in] _timeoutMS Milliseconds to wait for data.
  public: ssize_t Recv(void *_buf, const size_t _size, uint32_t _timeoutMs)
  {
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(this->handle, &fds);

    tv.tv_sec = _timeoutMs / 1000;
    tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

    if (select(this->handle+1, &fds, NULL, NULL, &tv) != 1)
    {
        return -1;
    }

    #ifdef _WIN32
    return recv(this->handle, reinterpret_cast<char *>(_buf), _size, 0);
    #else
    return recv(this->handle, _buf, _size, 0);
    #endif
  }

  /// \brief Model entity
  public: Entity modelEntity{kNullEntity};

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Link entity for getting velocity
  public: Entity modelLinkEntity{kNullEntity};

  /// \brief array of propellers
  public: std::vector<Rotor> rotors;

  /// \brief keep track of controller update sim-time.
  public: std::chrono::steady_clock::duration lastControllerUpdateTime{0};

  /// \brief Controller update mutex.
  public: std::mutex mutex;

  /// \brief Socket handle
  public: int handle;

  /// \brief IMU sensor entity
  public: Entity imuEntity{kNullEntity};

  /// \brief false before betaflight controller is online
  /// to allow gazebo to continue without waiting
  public: bool betaflightOnline;

  /// \brief number of times Betaflight skips update
  public: int connectionTimeoutCount;

  /// \brief number of times Betaflight skips update
  /// before marking Betaflight offline
  public: int connectionTimeoutMaxCount;

  /// \brief Update the control surfaces controllers.
  public: void OnUpdate(EntityComponentManager &_ecm,
                        const std::chrono::steady_clock::duration &_simTime);

  /// \brief Update PID Joint controllers.
  /// \param[in] _dt time step since last update.
  public: void ApplyMotorForces(const double _dt, EntityComponentManager &_ecm);

  /// \brief Reset PID Joint controllers.
  public: void ResetPIDs();

  /// \brief Receive motor commands from Betaflight
  public: void ReceiveMotorCommand();

  /// \brief Send state to Betaflight
  public: void SendState(EntityComponentManager &_ecm) const;
};

////////////////////////////////////////////////////////////////////////////////
BetaflightPlugin::BetaflightPlugin()
  : dataPtr(new BetaflightPluginPrivate)
{
  // socket
  this->dataPtr->handle = socket(AF_INET, SOCK_DGRAM /*SOCK_STREAM*/, 0);
  #ifndef _WIN32
  // Windows does not support FD_CLOEXEC
  fcntl(this->dataPtr->handle, F_SETFD, FD_CLOEXEC);
  #endif
  int one = 1;
  setsockopt(this->dataPtr->handle, IPPROTO_TCP, TCP_NODELAY,
      reinterpret_cast<const char *>(&one), sizeof(one));

  if (!this->dataPtr->Bind("127.0.0.1", 9002))
  {
    gzerr << "failed to bind with 127.0.0.1:9002, aborting plugin.\n";
    return;
  }

  this->dataPtr->betaflightOnline = false;

  this->dataPtr->connectionTimeoutCount = 0;

  setsockopt(this->dataPtr->handle, SOL_SOCKET, SO_REUSEADDR,
     reinterpret_cast<const char *>(&one), sizeof(one));

  #ifdef _WIN32
  u_long on = 1;
  ioctlsocket(this->dataPtr->handle, FIONBIO,
              reinterpret_cast<u_long FAR *>(&on));
  #else
  fcntl(this->dataPtr->handle, F_SETFL,
      fcntl(this->dataPtr->handle, F_GETFL, 0) | O_NONBLOCK);
  #endif
}

/////////////////////////////////////////////////
BetaflightPlugin::~BetaflightPlugin()
{
  #ifdef _WIN32
  closesocket(this->dataPtr->handle);
  #else
  close(this->dataPtr->handle);
  #endif
}

/////////////////////////////////////////////////
void BetaflightPlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager & /*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);
  this->dataPtr->modelEntity = _entity;

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "BetaflightPlugin should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }

  // Get the canonical link - first link in the model
  this->dataPtr->modelLinkEntity = this->dataPtr->model.CanonicalLink(_ecm);

  // Enable velocity component for the canonical link
  _ecm.CreateComponent(this->dataPtr->modelLinkEntity,
                       components::WorldLinearVelocity());

  // per rotor
  if (_sdf->HasElement("rotor"))
  {
    // Cast away const to call GetElement
    auto sdfNonConst = std::const_pointer_cast<sdf::Element>(_sdf);
    sdf::ElementPtr rotorSDF = sdfNonConst->GetElement("rotor");

    while (rotorSDF)
    {
      Rotor rotor;

      if (rotorSDF->HasAttribute("id"))
      {
        rotor.id = rotorSDF->GetAttribute("id")->Get(rotor.id);
      }
      else
      {
        rotor.id = this->dataPtr->rotors.size();
        gzwarn << "id attribute not specified, use order parsed ["
               << rotor.id << "].\n";
      }

      if (rotorSDF->HasElement("jointName"))
      {
        rotor.jointName = rotorSDF->Get<std::string>("jointName");
      }
      else
      {
        gzerr << "Please specify a jointName,"
          << " where the rotor is attached.\n";
      }

      // Get the pointer to the joint.
      rotor.jointEntity = this->dataPtr->model.JointByName(_ecm, rotor.jointName);
      if (rotor.jointEntity == kNullEntity)
      {
        gzerr << "Couldn't find specified joint ["
            << rotor.jointName << "]. This plugin will not run.\n";
        return;
      }

      if (rotorSDF->HasElement("turningDirection"))
      {
        std::string turningDirection = rotorSDF->Get<std::string>(
            "turningDirection");
        // special cases mimic from rotors_gazebo_plugins
        if (turningDirection == "cw")
          rotor.multiplier = -1;
        else if (turningDirection == "ccw")
          rotor.multiplier = 1;
        else
        {
          gzdbg << "not string, check turningDirection as float\n";
          rotor.multiplier = rotorSDF->Get<double>("turningDirection");
        }
      }
      else
      {
        rotor.multiplier = 1;
        gzerr << "Please specify a turning"
          << " direction multiplier ('cw' or 'ccw'). Default 'ccw'.\n";
      }

      getSdfParam<double>(rotorSDF, "rotorVelocitySlowdownSim",
          rotor.rotorVelocitySlowdownSim, 1);

      if (gz::math::equal(rotor.rotorVelocitySlowdownSim, 0.0))
      {
        gzerr << "rotor for joint [" << rotor.jointName
              << "] rotorVelocitySlowdownSim is zero,"
              << " aborting plugin.\n";
        return;
      }

      getSdfParam<double>(rotorSDF, "frequencyCutoff",
          rotor.frequencyCutoff, rotor.frequencyCutoff);
      getSdfParam<double>(rotorSDF, "samplingRate",
          rotor.samplingRate, rotor.samplingRate);

      // use gz::math::Filter
      rotor.velocityFilter.Fc(rotor.frequencyCutoff, rotor.samplingRate);

      // initialize filter to zero value
      rotor.velocityFilter.Set(0.0);

      // note to use this
      // rotorVelocityFiltered = velocityFilter.Process(rotorVelocityRaw);

      // Overload the PID parameters if they are available.
      double param;
      getSdfParam<double>(rotorSDF, "vel_p_gain", param, rotor.pid.PGain());
      rotor.pid.SetPGain(param);

      getSdfParam<double>(rotorSDF, "vel_i_gain", param, rotor.pid.IGain());
      rotor.pid.SetIGain(param);

      getSdfParam<double>(rotorSDF, "vel_d_gain", param,  rotor.pid.DGain());
      rotor.pid.SetDGain(param);

      getSdfParam<double>(rotorSDF, "vel_i_max", param, rotor.pid.IMax());
      rotor.pid.SetIMax(param);

      getSdfParam<double>(rotorSDF, "vel_i_min", param, rotor.pid.IMin());
      rotor.pid.SetIMin(param);

      getSdfParam<double>(rotorSDF, "vel_cmd_max", param,
          rotor.pid.CmdMax());
      rotor.pid.SetCmdMax(param);

      getSdfParam<double>(rotorSDF, "vel_cmd_min", param,
          rotor.pid.CmdMin());
      rotor.pid.SetCmdMin(param);

      // set pid initial command
      rotor.pid.SetCmd(0.0);

      // Create joint velocity component if it doesn't exist
      if (!_ecm.Component<components::JointVelocity>(rotor.jointEntity))
      {
        _ecm.CreateComponent(rotor.jointEntity, components::JointVelocity());
      }

      // Create joint force command component if it doesn't exist
      if (!_ecm.Component<components::JointForceCmd>(rotor.jointEntity))
      {
        _ecm.CreateComponent(rotor.jointEntity,
                            components::JointForceCmd({0.0}));
      }

      this->dataPtr->rotors.push_back(rotor);
      rotorSDF = rotorSDF->GetNextElement("rotor");
    }
  }

  // Get sensors - find IMU sensor
  std::string imuName;
  getSdfParam<std::string>(_sdf, "imuName", imuName, "imu_sensor");

  // Search for IMU sensor entity
  // The imuName might be a scoped name, so we need to parse it
  std::string sensorName = imuName;
  size_t lastSep = imuName.find_last_of("::");
  if (lastSep != std::string::npos)
  {
    sensorName = imuName.substr(lastSep + 1);
  }

  // Find IMU by iterating through sensor entities
  _ecm.Each<components::Imu, components::Name>(
    [&](const Entity &_entity,
        const components::Imu *,
        const components::Name *_name) -> bool
    {
      if (_name->Data().find(sensorName) != std::string::npos)
      {
        this->dataPtr->imuEntity = _entity;
        return false; // Stop iteration
      }
      return true; // Continue iteration
    });

  if (this->dataPtr->imuEntity == kNullEntity)
  {
    gzerr << "imu_sensor [" << imuName
          << "] not found, abort Betaflight plugin.\n" << "\n";
    return;
  }

  // Controller time control.
  this->dataPtr->lastControllerUpdateTime = std::chrono::steady_clock::duration::zero();

  // Missed update count before we declare betaflightOnline status false
  getSdfParam<int>(_sdf, "connectionTimeoutMaxCount",
    this->dataPtr->connectionTimeoutMaxCount, 10);

  gzlog << "Betaflight ready to fly. The force will be with you" << std::endl;
}

/////////////////////////////////////////////////
void BetaflightPlugin::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  this->dataPtr->OnUpdate(_ecm, _info.simTime);
}

/////////////////////////////////////////////////
void BetaflightPluginPrivate::OnUpdate(EntityComponentManager &_ecm,
    const std::chrono::steady_clock::duration &_simTime)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Update the control surfaces and publish the new state.
  if (_simTime > this->lastControllerUpdateTime)
  {
    this->ReceiveMotorCommand();
    if (this->betaflightOnline)
    {
      auto dt = std::chrono::duration<double>(_simTime - this->lastControllerUpdateTime).count();
      this->ApplyMotorForces(dt, _ecm);
      this->SendState(_ecm);
    }
  }

  this->lastControllerUpdateTime = _simTime;
}

/////////////////////////////////////////////////
void BetaflightPluginPrivate::ResetPIDs()
{
  // Reset velocity PID for rotors
  for (size_t i = 0; i < this->rotors.size(); ++i)
  {
    this->rotors[i].cmd = 0;
    // this->rotors[i].pid.Reset();
  }
}

/////////////////////////////////////////////////
void BetaflightPluginPrivate::ApplyMotorForces(const double _dt,
    EntityComponentManager &_ecm)
{
  // update velocity PID for rotors and apply force to joint
  for (size_t i = 0; i < this->rotors.size(); ++i)
  {
    double velTarget = this->rotors[i].multiplier *
      this->rotors[i].cmd /
      this->rotors[i].rotorVelocitySlowdownSim;

    // Get joint velocity (treat missing/empty data as zero velocity)
    double vel = 0.0;
    auto jointVelComp = _ecm.Component<components::JointVelocity>(
        this->rotors[i].jointEntity);

    if (jointVelComp && !jointVelComp->Data().empty())
      vel = jointVelComp->Data()[0];
    double error = vel - velTarget;
    // Convert dt to chrono::duration for gz::math::PID
    std::chrono::duration<double> dt_chrono(_dt);
    double force = this->rotors[i].pid.Update(error, dt_chrono);

    // Set joint force command
    _ecm.SetComponentData<components::JointForceCmd>(
        this->rotors[i].jointEntity, {force});
  }
}

/////////////////////////////////////////////////
void BetaflightPluginPrivate::ReceiveMotorCommand()
{
  // Added detection for whether Betaflight is online or not.
  // If Betaflight is detected (receive of fdm packet from someone),
  // then socket receive wait time is increased from 1ms to 1 sec
  // to accommodate network jitter.
  // If Betaflight is not detected, receive call blocks for 1ms
  // on each call.
  // Once Betaflight presence is detected, it takes this many
  // missed receives before declaring the FCS offline.

  ServoPacket pkt;
  int waitMs = 1;
  if (this->betaflightOnline)
  {
    // increase timeout for receive once we detect a packet from
    // Betaflight FCS.
    waitMs = 1000;
  }
  else
  {
    // Otherwise skip quickly and do not set control force.
    waitMs = 1;
  }
  ssize_t recvSize = this->Recv(&pkt, sizeof(ServoPacket), waitMs);
  ssize_t expectedPktSize =
    sizeof(pkt.motorSpeed[0])*this->rotors.size();
  if ((recvSize == -1) || (recvSize < expectedPktSize))
  {
    // didn't receive a packet
    if (recvSize != -1)
    {
      gzerr << "received bit size (" << recvSize << ") too small,"
            << " controller expected size (" << expectedPktSize << ").\n";
    }

    std::this_thread::sleep_for(std::chrono::microseconds(100));
    if (this->betaflightOnline)
    {
      gzwarn << "Broken Betaflight connection, count ["
             << this->connectionTimeoutCount
             << "/" << this->connectionTimeoutMaxCount
             << "]\n";
      if (++this->connectionTimeoutCount >
        this->connectionTimeoutMaxCount)
      {
        this->connectionTimeoutCount = 0;
        this->betaflightOnline = false;
        gzwarn << "Broken Betaflight connection, resetting motor control.\n";
        this->ResetPIDs();
      }
    }
  }
  else
  {
    if (!this->betaflightOnline)
    {
      gzdbg << "Betaflight controller online detected.\n";
      // made connection, set some flags
      this->connectionTimeoutCount = 0;
      this->betaflightOnline = true;
    }

    // compute command based on requested motorSpeed
    for (unsigned i = 0; i < this->rotors.size(); ++i)
    {
      if (i < MAX_MOTORS)
      {
        this->rotors[i].cmd = this->rotors[i].maxRpm *
          pkt.motorSpeed[i];
      }
      else
      {
        gzerr << "too many motors, skipping [" << i
              << " > " << MAX_MOTORS << "].\n";
      }
    }
  }
}

/////////////////////////////////////////////////
void BetaflightPluginPrivate::SendState(EntityComponentManager &_ecm) const
{
  // send_fdm
  fdmPacket pkt;

  // Get current simulation time
  // Note: We're using the last update time since we don't have direct access to UpdateInfo here
  pkt.timestamp = std::chrono::duration<double>(this->lastControllerUpdateTime).count();

  // Get IMU linear acceleration
  auto accelComp = _ecm.Component<components::LinearAcceleration>(this->imuEntity);
  if (!accelComp)
  {
    gzerr << "Unable to get IMU linear acceleration\n";
    return;
  }
  gz::math::Vector3d linearAccel = accelComp->Data();

  // copy to pkt
  pkt.imuLinearAccelerationXYZ[0] = linearAccel.X();
  pkt.imuLinearAccelerationXYZ[1] = linearAccel.Y();
  pkt.imuLinearAccelerationXYZ[2] = linearAccel.Z();

  // Get IMU angular velocity
  auto angularVelComp = _ecm.Component<components::AngularVelocity>(this->imuEntity);
  if (!angularVelComp)
  {
    gzerr << "Unable to get IMU angular velocity\n";
    return;
  }
  gz::math::Vector3d angularVel = angularVelComp->Data();

  // copy to pkt
  pkt.imuAngularVelocityRPY[0] = angularVel.X();
  pkt.imuAngularVelocityRPY[1] = angularVel.Y();
  pkt.imuAngularVelocityRPY[2] = angularVel.Z();

  // get inertial pose and velocity
  // gazeboToNED brings us from gazebo model: x-forward, y-right, z-down
  // to the aerospace convention: x-forward, y-left, z-up
  gz::math::Pose3d gazeboToNED(0, 0, 0, GZ_PI, 0, 0);

  // Get model world pose
  auto poseComp = _ecm.Component<components::Pose>(this->modelEntity);
  if (!poseComp)
  {
    gzerr << "Unable to get model pose\n";
    return;
  }
  gz::math::Pose3d modelWorldPose = poseComp->Data();

  // model world pose brings us to model, x-forward, y-left, z-up
  // The gazeboToNED rotation transforms from Gazebo to NED frame
  // Combine the transformations using pose multiplication
  gz::math::Pose3d worldToModel(
      gazeboToNED.Pos() + gazeboToNED.Rot().RotateVector(modelWorldPose.Pos()),
      gazeboToNED.Rot() * modelWorldPose.Rot());

  // get transform from world NED to Model frame
  gz::math::Pose3d NEDToModel(
      worldToModel.Pos() - gazeboToNED.Pos(),
      worldToModel.Rot() * gazeboToNED.Rot().Inverse());

  // N
  pkt.positionXYZ[0] = NEDToModel.Pos().X();

  // E
  pkt.positionXYZ[1] = NEDToModel.Pos().Y();

  // D
  pkt.positionXYZ[2] = NEDToModel.Pos().Z();

  // imuOrientationQuat is the rotation from world NED frame
  // to the quadrotor frame.
  pkt.imuOrientationQuat[0] = NEDToModel.Rot().W();
  pkt.imuOrientationQuat[1] = NEDToModel.Rot().X();
  pkt.imuOrientationQuat[2] = NEDToModel.Rot().Y();
  pkt.imuOrientationQuat[3] = NEDToModel.Rot().Z();

  // Get model velocity in NED frame
  auto velComp = _ecm.Component<components::WorldLinearVelocity>(this->modelLinkEntity);
  if (!velComp)
  {
    gzerr << "Unable to get model velocity\n";
    return;
  }

  gz::math::Vector3d velGazeboWorldFrame = velComp->Data();
  gz::math::Vector3d velNEDFrame =
    gazeboToNED.Rot().RotateVectorReverse(velGazeboWorldFrame);
  pkt.velocityXYZ[0] = velNEDFrame.X();
  pkt.velocityXYZ[1] = velNEDFrame.Y();
  pkt.velocityXYZ[2] = velNEDFrame.Z();

  // Virtual GPS: convert local position to lat/lon/alt when spherical
  // coordinates are configured in the world SDF. Also send velocity in
  // ENU frame as expected by Betaflight's virtual GPS mode.
  auto worldEnt = worldEntity(_ecm);
  auto scComp = _ecm.Component<components::SphericalCoordinates>(worldEnt);
  if (scComp)
  {
    const auto &sc = scComp->Data();

    // Convert local ENU position to lat/lon/alt
    gz::math::Vector3d latLonAlt =
        sc.SphericalFromLocalPosition(modelWorldPose.Pos());

    // Betaflight expects: [0]=longitude_deg, [1]=latitude_deg, [2]=altitude_m
    // SphericalFromLocalPosition returns degrees in gz-math7+
    pkt.positionXYZ[0] = latLonAlt.Y();  // longitude in degrees
    pkt.positionXYZ[1] = latLonAlt.X();  // latitude in degrees
    pkt.positionXYZ[2] = latLonAlt.Z();            // altitude in meters

    // Betaflight expects ENU: [0]=East, [1]=North, [2]=Up
    // Gazebo world frame is already ENU, so use raw velocity
    pkt.velocityXYZ[0] = velGazeboWorldFrame.X();  // East  (m/s)
    pkt.velocityXYZ[1] = velGazeboWorldFrame.Y();  // North (m/s)
    pkt.velocityXYZ[2] = velGazeboWorldFrame.Z();  // Up    (m/s)
  }

  // Emulate ESC Sensor
  pkt.escTemperature[4] = {};
  pkt.escVoltage[4] = {};
  pkt.escCurrent[4]  = {};
  pkt.escConsumption[4] = {};

  for (size_t i = 0; i < this->rotors.size(); ++i)
  {
    auto jointVelComp = _ecm.Component<components::JointVelocity>(
        this->rotors[i].jointEntity);

    if (jointVelComp && !jointVelComp->Data().empty())
    {
      // Angular velocity is returned in rad/s
      pkt.escRpm[i] = jointVelComp->Data()[0];
    }
    else
    {
      pkt.escRpm[i] = 0.0;
    }
  }

  struct sockaddr_in sockaddr;
  this->MakeSockAddr("127.0.0.1", 9003, sockaddr);

  ::sendto(this->handle,
           reinterpret_cast<raw_type *>(&pkt),
           sizeof(pkt), 0,
           (struct sockaddr *)&sockaddr, sizeof(sockaddr));
}

GZ_ADD_PLUGIN(BetaflightPlugin,
              System,
              BetaflightPlugin::ISystemConfigure,
              BetaflightPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(BetaflightPlugin, "gz::sim::systems::BetaflightPlugin")
