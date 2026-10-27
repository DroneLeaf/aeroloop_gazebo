// Opt-in, leased render-only control for Tracker Shake Lab. No model/IMU changes.
#pragma once
#include "RecoveryMotion.hh"
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <deque>

namespace recovery {
class Control {
 public:
  Packet config;
  Motion motion;
  Status status;
  bool enabled=false;
  double start=-1, epoch=0;
  ~Control(){if(fd>=0) close(fd);}
  bool matches(const std::string &model,const std::string &sensor) const {
    return enabled && model==config.model && (sensor==config.sensor || config.sensor[0]=='*');
  }
  std::string referenceSensor() const {return config.sensor+(config.sensor[0]=='*'?1:0);}
  void tick(double simTime) {
    status.simTime=simTime;
    if(!opened) {
      opened=true;
      int port=18950;
      if(const char *e=std::getenv("GZ_RECOVERY_PORT")) port=std::atoi(e);
      if(port<=0 || port>65535) return;
      fd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
      sockaddr_in addr{};addr.sin_family=AF_INET;addr.sin_port=htons(port);
      addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
      if(fd<0 || bind(fd,reinterpret_cast<sockaddr *>(&addr),sizeof(addr))) {
        fprintf(stderr,"[ShakeLab] UDP %d unavailable; control disabled in this server\n",port);
        if(fd>=0)close(fd);fd=-1;return;
      }
      fprintf(stderr,"[ShakeLab] passive control on 127.0.0.1:%d\n",port);
    }
    const auto now=std::chrono::steady_clock::now();
    if(enabled && now-lastSeen>std::chrono::seconds(2)) {enabled=false;start=-1;}
    for(int i=0;fd>=0 && i<32;++i) {
      Packet packet{};sockaddr_in peer{};socklen_t len=sizeof(peer);
      char buffer[sizeof(Packet)+1];
      ssize_t n=recvfrom(fd,buffer,sizeof(buffer),MSG_DONTWAIT,reinterpret_cast<sockaddr *>(&peer),&len);
      if(n<0)break;
      if(n!=sizeof(Packet))continue;
      std::memcpy(&packet,buffer,sizeof(packet));
      if(packet.magic!=Magic || packet.version!=Version || !packet.owner)continue;
      uint32_t code=0;
      if(enabled && packet.owner!=config.owner)code=1; // another live owner
      else if(packet.command==Configure) {
        if(!std::memchr(packet.model,0,64)||!std::memchr(packet.sensor,0,64)
           ||!std::memchr(packet.target,0,64)||!valid(packet.p)
           ||(packet.reserved & ~OverrideScene))code=2;
        else if(packet.request!=lastConfigure || packet.owner!=config.owner) {
          if(packet.owner!=config.owner) recentTriggers.clear();
          config=packet;enabled=true;start=-1;epoch=simTime;
          status.reserved=0;status.width=0;status.height=0;
          status.captureHz=0;status.cameraHz=0;
          motion.prepare(config.p);lastConfigure=packet.request;
        }
      } else if(packet.command==Trigger && enabled) {
        if(std::find(recentTriggers.begin(),recentTriggers.end(),packet.request)==recentTriggers.end()) {
          start=simTime; motion.prepare(config.p); ++status.trial;
          recentTriggers.push_back(packet.request);if(recentTriggers.size()>128)recentTriggers.pop_front();
        }
      } else if(packet.command==Reset && enabled) {start=-1;epoch=simTime;}
      else if(packet.command==Release) {enabled=false;start=-1;}
      else if(packet.command!=Poll) code=3;
      if(!code && enabled)lastSeen=now;
      status.request=packet.request;status.code=code;status.active=enabled;
      status.shaking=enabled && start>=0 && simTime-start<config.p.duration;
      status.shakeTime=start<0?0:simTime-start;
      status.peakVelocity=motion.peak;
      sendto(fd,&status,sizeof(status),MSG_DONTWAIT,reinterpret_cast<sockaddr *>(&peer),len);
    }
  }
 private:
  int fd=-1;bool opened=false;
  uint32_t lastConfigure=0;
  std::deque<uint32_t> recentTriggers;
  std::chrono::steady_clock::time_point lastSeen{};
};
}  // namespace recovery
