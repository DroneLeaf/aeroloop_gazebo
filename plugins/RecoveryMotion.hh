// Tracker Shake Lab wire contract and deterministic trajectories.
// Shared by the renderer, native monitor presenter and offline acceptance test.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>

namespace recovery {
constexpr uint32_t Magic = 0x53484b31;  // SHK1, little endian on the wire
constexpr uint32_t Version = 3;  // Shake-only control; camera rates belong to the simulator.
constexpr double Pi = 3.14159265358979323846;
enum Command : uint32_t { Configure=1, Trigger=2, Reset=3, Release=4, Poll=5 };
// Configure flags: zero always follows the live simulation. Absolute scene
// overrides require an explicit opt-in from the lab's owned-scene mode.
constexpr uint32_t OverrideScene = 1;
// Doubles are named here; Python's protocol.py checks this exact field order.
struct Parameters {
  double velocity=1800, duration=0.5, lowHz=6, highHz=25, displacement=180;
  double horizontal=1, vertical=1, rollDeg=0, seed=1, trajectory=0;
  double cameraX=0, cameraY=0, cameraZ=10, yawDeg=0, pitchDeg=0;
  double targetDistance=80, targetU=0, targetV=0, targetMotion=0;
  double targetAmplitude=100, targetSpeed=120, targetScale=1;
  double referenceWidth=854, referenceHfovDeg=45, pixelScale=1;
};
static_assert(sizeof(Parameters)==25*8, "Update Python wire contract too");
struct Packet {
  uint32_t magic=Magic, version=Version, command=Poll, owner=0, request=0, reserved=0;
  char model[64]{}, sensor[64]{}, target[64]{};
  Parameters p;
};
static_assert(sizeof(Packet)==416, "wire packet");
struct Status {
  uint32_t magic=Magic, version=Version, request=0, code=0;
  uint32_t active=0, shaking=0, trial=0, reserved=0;
  double simTime=0, shakeTime=0, u=0, v=0, peakVelocity=0;
  double captureHz=0, cameraHz=0, focal=0, targetU=0, targetV=0;
  double width=0, height=0, hfovDeg=0;
};
static_assert(sizeof(Status)==136, "wire status");
inline bool valid(const Parameters &p) {
  const double *v = reinterpret_cast<const double *>(&p);
  for (unsigned i=0; i<25; ++i) if (!std::isfinite(v[i])) return false;
  return p.velocity>=0 && p.duration>=0.02 && p.duration<=60 && p.lowHz>0
      && p.highHz>=p.lowHz && p.highHz<1000 && p.displacement>=0
      && p.horizontal>=0 && p.vertical>=0 && p.seed>=0 && p.seed<=4294967295.0
      && p.trajectory>=0 && p.trajectory<=3 && std::floor(p.trajectory)==p.trajectory
      && p.targetMotion>=0 && p.targetMotion<=2 && std::floor(p.targetMotion)==p.targetMotion
      && p.targetDistance>0 && p.targetAmplitude>=0 && p.targetSpeed>=0 && p.targetScale>0
      && p.referenceWidth>0 && p.referenceHfovDeg>0 && p.referenceHfovDeg<179
      && p.pixelScale>0;
}
struct Sample { double u=0,v=0,roll=0; };
class Motion {
 public:
  Parameters p;
  double scale=0, peak=0;
  void prepare(const Parameters &params) {
    p=params;
    std::mt19937 rng(static_cast<uint32_t>(p.seed));
    auto uniform=[&](){return static_cast<double>(rng())/4294967295.0;};
    for (auto &w:waves) w={p.lowHz+(p.highHz-p.lowHz)*uniform(),2*Pi*uniform(),2*Pi*uniform()};
    double maxSpeed=0, maxRadius=0;
    Sample previous=raw(0);
    // >= 64 samples per highest-frequency cycle, including the burst envelope.
    const int count=std::max(4096,static_cast<int>(std::ceil(p.duration*p.highHz*64)));
    const double dt=p.duration/count;
    for(int i=1;i<=count;++i) {
      Sample s=raw(i*dt);
      maxSpeed=std::max(maxSpeed,std::hypot(s.u-previous.u,s.v-previous.v)/dt);
      maxRadius=std::max(maxRadius,std::hypot(s.u,s.v)); previous=s;
    }
    scale=maxSpeed>0 ? p.velocity/p.pixelScale/maxSpeed : 0;
    if(maxRadius>0) scale=std::min(scale,p.displacement/p.pixelScale/maxRadius);
    peak=maxSpeed*scale*p.pixelScale;
  }
  Sample sample(double t) const {
    Sample s=raw(t); s.u*=scale; s.v*=scale; return s;
  }
 private:
  std::array<std::array<double,3>,6> waves{};
  Sample raw(double t) const {
    if(t<=0 || t>=p.duration) return {};
    double x=t/p.duration, e=std::pow(std::sin(Pi*x),2), u=0,v=0;
    if(p.trajectory==0) {
      for(auto w:waves) {u+=std::sin(2*Pi*w[0]*t+w[1]);v+=std::sin(2*Pi*w[0]*t+w[2]);}
    } else if(p.trajectory==1) {
      u=std::sin(2*Pi*p.lowHz*t)*std::exp(-4*x);
      v=std::sin(2*Pi*p.highHz*t)*std::exp(-4*x);
    } else if(p.trajectory==2) {
      u=std::sin(Pi*x); v=std::sin(2*Pi*x)*0.35;
    } else {
      double phase=2*Pi*(p.lowHz*t+0.5*(p.highHz-p.lowHz)*t*t/p.duration);
      u=std::sin(phase); v=std::cos(phase);
    }
    return {u*e*p.horizontal,v*e*p.vertical,p.rollDeg*Pi/180*e*std::sin(2*Pi*p.lowHz*t)};
  }
};
inline std::array<double,2> target(const Parameters &p,double t) {
  double u=p.targetU/p.pixelScale,v=p.targetV/p.pixelScale;
  const double a=p.targetAmplitude/p.pixelScale, speed=p.targetSpeed/p.pixelScale;
  if(a>0 && p.targetMotion>0) {
    double phase=speed/a*t;
    u+=a*std::sin(phase);
    if(p.targetMotion==2) v+=a*(1-std::cos(phase));
  }
  return {u,v};
}
}  // namespace recovery
