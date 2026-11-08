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
constexpr uint32_t Version = 4;  // Native target trajectories; camera rates belong to the simulator.
constexpr double Pi = 3.14159265358979323846;
enum Command : uint32_t { Configure=1, Trigger=2, Reset=3, Release=4, Poll=5 };
// Configure flags: zero always follows the live simulation. Absolute scene
// overrides require an explicit opt-in from the lab's owned-scene mode.
constexpr uint32_t OverrideScene = 1;
// Doubles are named here; Python's protocol.py checks this exact field order.
struct Parameters {
  double velocity=1800, duration=0.5, lowHz=6, highHz=25, displacement=180;
  double horizontal=1, vertical=1, rollDeg=0, seed=1, trajectory=0;
  double cameraX=0, cameraY=0, cameraZ=10, yawDeg=0, pitchDeg=90;
  double targetDistance=140, targetU=0, targetV=0, targetMotion=0;
  double targetAmplitude=100, targetSpeed=120, targetScale=1;
  double referenceWidth=854, referenceHfovDeg=45, pixelScale=1;
  double targetAspect=1, targetPhaseDeg=0, targetAltitude=150;
  double loopEast=0, loopNorth=0, loopEW=40, loopNS=20, loopRadius=10;
  double loopRotationDeg=0, loopStart=0, loopReverse=0, loopSpeed=18;
  double loopPerturb=0, loopLatAmp=10, loopLatHz=.1, loopVertAmp=5, loopVertHz=.1, loopPhaseDeg=0;
};
static_assert(sizeof(Parameters)==43*8, "Update Python wire contract too");
struct Packet {
  uint32_t magic=Magic, version=Version, command=Poll, owner=0, request=0, reserved=0;
  char model[64]{}, sensor[64]{}, target[64]{};
  Parameters p;
};
static_assert(sizeof(Packet)==560, "wire packet");
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
  for (unsigned i=0; i<43; ++i) if (!std::isfinite(v[i])) return false;
  return p.velocity>=0 && p.duration>=0.02 && p.duration<=60 && p.lowHz>0
      && p.highHz>=p.lowHz && p.highHz<1000 && p.displacement>=0
      && p.horizontal>=0 && p.vertical>=0 && p.seed>=0 && p.seed<=4294967295.0
      && p.trajectory>=0 && p.trajectory<=3 && std::floor(p.trajectory)==p.trajectory
      && p.targetMotion>=0 && p.targetMotion<=6 && std::floor(p.targetMotion)==p.targetMotion
      && p.targetDistance>0 && p.targetAmplitude>=0 && p.targetSpeed>=0 && p.targetScale>0
      && p.referenceWidth>0 && p.referenceHfovDeg>0 && p.referenceHfovDeg<179
      && p.pixelScale>0 && std::floor(p.seed)==p.seed && p.targetAspect>=0
      && p.loopEW>=0 && p.loopNS>=0 && p.loopRadius>=0 && p.loopSpeed>=0
      && p.loopStart>=0 && p.loopStart<=1 && (p.loopReverse==0 || p.loopReverse==1)
      && (p.loopPerturb==0 || p.loopPerturb==1) && p.loopLatAmp>=0 && p.loopLatHz>=0
      && p.loopVertAmp>=0 && p.loopVertHz>=0;
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
    double phase=speed/a*t+p.targetPhaseDeg*Pi/180;
    if(p.targetMotion==3) v+=a*std::sin(phase);
    else if(p.targetMotion<6) {
      u+=a*std::sin(phase);
      if(p.targetMotion==2) v+=a*(1-std::cos(phase)); // preserve legacy circle
      if(p.targetMotion==4) v+=a*p.targetAspect*std::cos(phase);
      if(p.targetMotion==5) v+=a*p.targetAspect*std::sin(2*phase);
    }
  }
  return {u,v};
}

// Exact general-loop geometry from betaloop/common.py::trajectory_sample.
// Arc length sets constant speed; rotation pivots about the geometric centre.
inline std::array<double,3> loopLocal(double s,double a,double b,double r) {
  r=std::max(r,1e-3);const double q=Pi/2*r, perimeter=2*a+2*b+4*q;
  double u=std::fmod(std::fmod(s,perimeter)+perimeter,perimeter),ang;
  if(u<a)return {u,0,0};u-=a;
  if(u<q){ang=-Pi/2+u/r;return {a+r*std::cos(ang),r+r*std::sin(ang),ang+Pi/2};}u-=q;
  if(u<b)return {a+r,r+u,Pi/2};u-=b;
  if(u<q){ang=u/r;return {a+r*std::cos(ang),r+b+r*std::sin(ang),ang+Pi/2};}u-=q;
  if(u<a)return {a-u,2*r+b,Pi};u-=a;
  if(u<q){ang=Pi/2+u/r;return {r*std::cos(ang),r+b+r*std::sin(ang),ang+Pi/2};}u-=q;
  if(u<b)return {-r,r+b-u,-Pi/2};u-=b;
  ang=Pi+u/r;return {r*std::cos(ang),r+r*std::sin(ang),ang+Pi/2};
}
// East, North, altitude, heading. Independent of camera pose and pixel calibration.
inline std::array<double,4> worldTarget(const Parameters &p,double t) {
  const double r=std::max(p.loopRadius,1e-3), perimeter=2*p.loopEW+2*p.loopNS+2*Pi*r;
  const auto local=loopLocal(p.loopStart*perimeter+(p.loopReverse?-1:1)*p.loopSpeed*t,p.loopEW,p.loopNS,r);
  const double th=p.loopRotationDeg*Pi/180,c=std::cos(th),s=std::sin(th);
  const double x=local[0]-p.loopEW/2,y=local[1]-r-p.loopNS/2;
  const double yaw=local[2]+th+(p.loopReverse?Pi:0);
  const double lat=p.loopPerturb*p.loopLatAmp*std::sin(2*Pi*p.loopLatHz*t);
  const double vert=p.loopPerturb*p.loopVertAmp*std::sin(2*Pi*p.loopVertHz*t+p.loopPhaseDeg*Pi/180);
  return {p.loopEast+x*c-y*s-lat*std::sin(yaw),p.loopNorth+x*s+y*c+lat*std::cos(yaw),
          p.targetAltitude+vert,yaw};
}
}  // namespace recovery
