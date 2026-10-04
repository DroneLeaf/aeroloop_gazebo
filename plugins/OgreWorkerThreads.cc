// OgreWorkerThreads — LD_PRELOAD shim that caps OgreNext's scene-manager
// worker-thread pool inside the gz server.
//
// gz-rendering8 (Ogre2Scene::CreateContext) creates the scene manager with
//   createSceneManager(ST_GENERIC, Ogre::PlatformInformation::getNumLogicalCores())
// i.e. one culling/update worker per LOGICAL core (12 on a 6c/12t laptop).
// Every camera render fans culling + scene work out to that pool and the
// render thread waits at a barrier for all of them. With small per-camera
// work and a busy, thermally throttled CPU the barrier/wake-up overhead
// dominates. Measured live (2 fleet drones x wide+narrow 854x480 @ 90 Hz, GS
// + LeafFC + trackers running, CPU package at 95 C):
//   workers 12 (stock) 67 fps, gz 200% CPU | 4: 85 | 3: 88 | 2: 85-88, 142% | 1: 69
// gz-rendering exposes no knob for this, so the gz process gets this library
// preloaded (betaloop common.gz_spawn_env, `--ogre-workers`, default 2) and the
// symbol below interposes the cross-library call from libgz-rendering8-ogre2.
//
// GZ_OGRE_WORKER_THREADS=N picks the count; unset/<=0 keeps the stock value.
// Only the gz process is affected: nothing else in it calls this function
// (verified: libgz-rendering8-ogre2 is the only importer in the gz stack).

#include <cstdlib>
#include <unistd.h>

namespace Ogre
{
typedef unsigned int uint32;
class PlatformInformation
{
public:
  static uint32 getNumLogicalCores();
};
}  // namespace Ogre

Ogre::uint32 Ogre::PlatformInformation::getNumLogicalCores()
{
  const char *env = std::getenv("GZ_OGRE_WORKER_THREADS");
  const long n = env ? std::strtol(env, nullptr, 10) : 0;
  if (n > 0)
    return static_cast<Ogre::uint32>(n);
  const long cores = sysconf(_SC_NPROCESSORS_ONLN);
  return static_cast<Ogre::uint32>(cores > 0 ? cores : 1);
}
