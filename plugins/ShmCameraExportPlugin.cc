/*
 * ShmCameraExportPlugin — in-process camera frame export to POSIX shared
 * memory, bypassing gz-transport entirely, with an ASYNCHRONOUS GPU readback.
 *
 * WHY (measured 2026-09-15/16 on the 25-tile baylands world, RTX 2070 S):
 *  - gz-sensors only renders a camera that has an image-topic subscriber, and
 *    publishing serialises ~3.7 MB/frame-pair on the render thread.
 *  - Its readback (gz-rendering Camera::Copy → Ogre AsyncTextureTicket +
 *    CPU pixel conversion) is a SYNCHRONOUS ~450 MB/s staging download: on
 *    the real scene 7–9 ms per camera against ~2 ms of render submission.
 *    Two cameras = ~20 ms per cycle → 45–52 fps, and because the Sensors
 *    system lock-steps the sim to the render thread, RTF fell to 0.48.
 *
 * HOW: a gz-sim MODEL System. On gz::sim::events::PostRender (render thread)
 * it renders each due camera itself (Camera::Update — gz-sensors will not,
 * nobody subscribes) and then, instead of Copy(), issues
 * glGetTextureImage(RenderTextureGLId(), GL_RGB) into a 2-deep PIXEL_PACK
 * buffer ring (non-blocking) and collects the PREVIOUS frame's download,
 * which finished during the intervening render — one frame of latency
 * (1/update_rate) for a readback that costs ~0.5 ms instead of ~8. Frames go
 * straight into `/gz_cam_<model>_<sensor>_raw` using gz_image_bridge's exact
 * 64-byte ShmHeader (seqlock sequence + release fence); the bridge reads it
 * with `--shm-source`. GL entry points come from eglGetProcAddress (the ogre2
 * engine's context is EGL); if the GL path is unavailable, or with
 * SHM_EXPORT_SYNC=1 in the environment, it falls back to the synchronous
 * Camera::Copy path (identical output, slower).
 *
 * Scheduling mirrors gz-sensors: a camera is "due" when sim time crosses its
 * next 1/update_rate boundary (same catch-up rule).
 *
 * Attach inside a <model>:
 *   <plugin filename="ShmCameraExportPlugin"
 *           name="gz::sim::systems::ShmCameraExportPlugin"/>
 * Optional children: <sensor>name</sensor> (repeatable) to export only those
 * cameras; default = every camera sensor of the model.
 *
 * Shutdown rule (learned the hard way): hold NO rendering smart pointers
 * across frames — the ogre2 engine plugin is dlclose'd at shutdown and their
 * control blocks then segfault on release. Everything here is a per-call
 * lookup cached by NAME only; GL objects are simply abandoned at exit (the
 * context may already be gone when the destructor runs).
 */
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Image.hh>
#include <gz/rendering/PixelFormat.hh>
#include <gz/common/Event.hh>
#include <gz/plugin/Register.hh>
#include <sdf/Sensor.hh>
#include <sdf/Element.hh>

#include <GL/gl.h>
#include <GL/glext.h>
#include <dlfcn.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace gz
{
namespace sim
{
namespace systems
{

// Byte-for-byte the layout in gz_image_bridge.cc (static_assert'ed there).
struct ShmHeader {
    uint32_t magic;          // 0x475A4652 = "GZFR"
    uint32_t width;
    uint32_t height;
    uint32_t channels;
    uint32_t stride;
    uint32_t frame_size;
    uint64_t sequence;       // monotonic counter — readers poll this
    uint64_t timestamp_ns;   // steady_clock write time (wall)
    char     pix_fmt[16];    // "rgb24" / "bgr24" / "rgba" / "bgra"
    uint64_t sim_time_ns;    // render sim clock of this frame
};
static_assert(sizeof(ShmHeader) == 64, "ShmHeader must be 64 bytes");

// ── GL entry points, resolved at first use through eglGetProcAddress ──
struct GlFns
{
  PFNGLGENBUFFERSPROC       GenBuffers{nullptr};
  PFNGLBINDBUFFERPROC       BindBuffer{nullptr};
  PFNGLBUFFERDATAPROC       BufferData{nullptr};
  PFNGLGETTEXTUREIMAGEPROC  GetTextureImage{nullptr};
  PFNGLMAPBUFFERRANGEPROC   MapBufferRange{nullptr};
  PFNGLUNMAPBUFFERPROC      UnmapBuffer{nullptr};
  PFNGLFENCESYNCPROC        FenceSync{nullptr};
  PFNGLCLIENTWAITSYNCPROC   ClientWaitSync{nullptr};
  PFNGLDELETESYNCPROC       DeleteSync{nullptr};
  void (*PixelStorei)(GLenum, GLint){nullptr};
  void (*GetIntegerv)(GLenum, GLint *){nullptr};
  GLenum (*GetError)(){nullptr};
  void (*Flush)(){nullptr};
  bool tried{false}, ok{false};

  static void *Proc(const char *_name)
  {
    static void *(*eglGPA)(const char *) = nullptr;
    static bool init = false;
    if (!init)
    {
      init = true;
      void *egl = dlopen("libEGL.so.1", RTLD_NOW | RTLD_GLOBAL);
      if (egl)
        eglGPA = reinterpret_cast<void *(*)(const char *)>(dlsym(egl, "eglGetProcAddress"));
    }
    void *p = eglGPA ? eglGPA(_name) : nullptr;
    if (!p) p = dlsym(RTLD_DEFAULT, _name);
    return p;
  }

  bool Load()
  {
    if (this->tried) return this->ok;
    this->tried = true;
#define GZ_GLFN(field, name, type) \
    this->field = reinterpret_cast<type>(Proc(name)); if (!this->field) { \
      fprintf(stderr, "[ShmCameraExport] GL entry point %s unavailable -> sync fallback\n", name); return false; }
    GZ_GLFN(GenBuffers, "glGenBuffers", PFNGLGENBUFFERSPROC)
    GZ_GLFN(BindBuffer, "glBindBuffer", PFNGLBINDBUFFERPROC)
    GZ_GLFN(BufferData, "glBufferData", PFNGLBUFFERDATAPROC)
    GZ_GLFN(GetTextureImage, "glGetTextureImage", PFNGLGETTEXTUREIMAGEPROC)
    GZ_GLFN(MapBufferRange, "glMapBufferRange", PFNGLMAPBUFFERRANGEPROC)
    GZ_GLFN(UnmapBuffer, "glUnmapBuffer", PFNGLUNMAPBUFFERPROC)
    GZ_GLFN(FenceSync, "glFenceSync", PFNGLFENCESYNCPROC)
    GZ_GLFN(ClientWaitSync, "glClientWaitSync", PFNGLCLIENTWAITSYNCPROC)
    GZ_GLFN(DeleteSync, "glDeleteSync", PFNGLDELETESYNCPROC)
    GZ_GLFN(PixelStorei, "glPixelStorei", void (*)(GLenum, GLint))
    GZ_GLFN(GetIntegerv, "glGetIntegerv", void (*)(GLenum, GLint *))
    GZ_GLFN(GetError, "glGetError", GLenum (*)())
    GZ_GLFN(Flush, "glFlush", void (*)())
#undef GZ_GLFN
    this->ok = true;
    return true;
  }
};
static GlFns g_gl;

class ShmCameraExportPlugin
  : public System,
    public ISystemConfigure,
    public ISystemPostUpdate
{
  struct Cam
  {
    Entity entity{kNullEntity};
    std::string scoped;        // "model::link::sensor" — the rendering camera name
    std::string shortName;     // "fpv_tracker_wide_cam"
    std::string shmName;       // "/gz_cam_<model>_<sensor>_raw"
    std::string resolvedName;  // rendering camera name once found (looked up per frame)
    int64_t periodNs{0};       // 0 = every render pass
    int64_t nextDueNs{0};
    std::atomic<bool> due{false};
    std::atomic<int64_t> dueSimNs{0};
    // sync-fallback copy buffer (allocated by libgz-rendering core, stays loaded)
    rendering::Image image;
    bool imageInit{false};
    // async readback ring
    GLuint pbo[2]{0, 0};
    GLsync fence[2]{nullptr, nullptr};
    int64_t ringSimNs[2]{0, 0};
    int ringIdx{0};
    bool ringInit{false};
    uint32_t ringW{0}, ringH{0};
    // shm
    int fd{-1};
    uint8_t *ptr{nullptr};
    size_t size{0};
    uint64_t seq{0};
    bool warnedNoCam{false};
    // timing stats (render thread only): microseconds accumulated since last report
    double renderUs{0}, copyUs{0}; uint32_t nStat{0};
    double issueUs{0}, waitUs{0}, mapUs{0};   // async phases
    std::atomic<uint32_t> dueCount{0};        // dues raised by the sim thread (skip detector)
  };

  public: ~ShmCameraExportPlugin() override
  {
    // Detach from the render thread FIRST, then the segments. GL objects are
    // deliberately abandoned (the context may already be gone).
    this->postRenderConn.reset();
    for (auto &c : this->cams) this->CloseShm(*c);
  }

  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &_eventMgr) override
  {
    this->model = _entity;
    if (auto *n = _ecm.Component<components::Name>(_entity))
      this->modelName = n->Data();
    if (_sdf)
    {
      auto el = _sdf->FindElement("sensor");
      while (el)
      {
        this->filter.insert(el->Get<std::string>());
        el = el->GetNextElement("sensor");
      }
    }
    const char *sync = std::getenv("SHM_EXPORT_SYNC");
    this->forceSync = sync && *sync && *sync != '0';
    this->postRenderConn = _eventMgr.Connect<events::PostRender>(
        [this]() { this->OnPostRender(); });
    fprintf(stderr, "[ShmCameraExport] attached to model '%s'%s%s\n",
            this->modelName.c_str(),
            this->filter.empty() ? " (all cameras)" : " (filtered)",
            this->forceSync ? " [SYNC readback forced]" : "");
  }

  public: void PostUpdate(const UpdateInfo &_info,
                          const EntityComponentManager &_ecm) override
  {
    if (!this->enumerated.load(std::memory_order_acquire))
      this->Enumerate(_ecm);
    if (_info.paused) return;
    const int64_t simNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
        _info.simTime).count();
    for (auto &c : this->cams)
    {
      if (c->periodNs <= 0)
      {
        c->dueSimNs.store(simNs, std::memory_order_relaxed);
        if (!c->due.exchange(true, std::memory_order_acq_rel))
          c->dueCount.fetch_add(1, std::memory_order_relaxed);
        continue;
      }
      if (simNs >= c->nextDueNs)
      {
        while (c->nextDueNs <= simNs) c->nextDueNs += c->periodNs;
        c->dueSimNs.store(simNs, std::memory_order_relaxed);
        if (!c->due.exchange(true, std::memory_order_acq_rel))
          c->dueCount.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }

  private: void Enumerate(const EntityComponentManager &_ecm)
  {
    std::lock_guard<std::mutex> lk(this->enumMutex);
    if (this->enumerated.load(std::memory_order_acquire)) return;
    _ecm.Each<components::Camera, components::Name>(
      [&](const Entity &_e, const components::Camera *_cam,
          const components::Name *_name) -> bool
      {
        if (topLevelModel(_e, _ecm) != this->model) return true;
        const std::string &sname = _name->Data();
        if (!this->filter.empty() && !this->filter.count(sname)) return true;
        auto c = std::make_unique<Cam>();
        c->entity = _e;
        c->shortName = sname;
        c->scoped = scopedName(_e, _ecm, "::", false);
        c->shmName = "/gz_cam_" + this->modelName + "_" + sname + "_raw";
        for (auto &ch : c->shmName)
          if (ch != '/' && ch != '_' && !isalnum(static_cast<unsigned char>(ch))) ch = '_';
        const double rate = _cam->Data().UpdateRate();
        c->periodNs = rate > 0.0 ? static_cast<int64_t>(1e9 / rate) : 0;
        c->nextDueNs = 0;
        fprintf(stderr, "[ShmCameraExport] camera '%s' -> %s (%.1f Hz)\n",
                c->scoped.c_str(), c->shmName.c_str(), rate);
        this->cams.push_back(std::move(c));
        return true;
      });
    this->enumerated.store(true, std::memory_order_release);
  }

  // Render thread.
  private: void OnPostRender()
  {
    if (!this->enumerated.load(std::memory_order_acquire)) return;
    rendering::ScenePtr scn = rendering::sceneFromFirstRenderEngine();
    if (!scn) return;
    const bool useAsync = !this->forceSync && g_gl.Load();
    // Cycle bookkeeping: interval between PostRender passes vs. our own time
    // in them — the difference is what gz spends per cycle outside the plugin.
    const auto cycleStart = std::chrono::steady_clock::now();
    if (this->lastCycle.time_since_epoch().count())
      this->cycleUs += std::chrono::duration<double, std::micro>(cycleStart - this->lastCycle).count();
    this->lastCycle = cycleStart;
    // Collect the due cameras first so the scene pre/post-render brackets the
    // whole set ONCE (the Scene.hh contract: "PostRender after you're done
    // updating ALL cameras"; Camera::Update() would repeat both per camera,
    // and the scene PostRender is the GPU flush + new-frame boundary).
    // CONSUME the due flag here, atomically, BEFORE rendering. The sim thread
    // (PostUpdate) keeps stepping while this pass runs and may set due=true
    // mid-pass; clearing the flag after the render clobbered that and
    // silently skipped frames (live: wide 79 / narrow 59.5 fps at an 89 Hz
    // pass rate). A camera whose lookup fails keeps its flag for a retry.
    std::vector<std::tuple<Cam *, rendering::CameraPtr, int64_t>> due;
    for (auto &c : this->cams)
    {
      if (!c->due.load(std::memory_order_acquire)) continue;
      rendering::CameraPtr cam = this->FindCamera(*c, *scn);
      if (!cam) continue;                                   // retry next pass
      const int64_t simNs = c->dueSimNs.load(std::memory_order_relaxed);
      c->due.store(false, std::memory_order_release);
      due.emplace_back(c.get(), cam, simNs);
    }
    if (!due.empty()) scn->PreRender();
    for (auto &[cptr, cam, simNs] : due)
    {
      Cam &c_ = *cptr;
      const auto t0 = std::chrono::steady_clock::now();
      // gz-sensors does not render a camera without an image subscriber
      // (verified: raw export stayed black until one attached) — so render.
      cam->Render();
      cam->PostRender();
      const auto t1 = std::chrono::steady_clock::now();
      auto c = cptr;   // keep the loop body below unchanged (uses c->)
      if (useAsync)
      {
        this->AsyncReadback(*c, *cam, simNs);
      }
      else
      {
        if (!c->imageInit)
        {
          c->image = rendering::Image(cam->ImageWidth(), cam->ImageHeight(),
                                      cam->ImageFormat());
          c->imageInit = true;
        }
        cam->Copy(c->image);
        this->WriteShm(*c, static_cast<const uint8_t *>(c->image.Data()),
                       c->image.MemorySize(), cam->ImageWidth(),
                       cam->ImageHeight(), cam->ImageFormat(), simNs);
      }
      const auto t2 = std::chrono::steady_clock::now();
      c->renderUs += std::chrono::duration<double, std::micro>(t1 - t0).count();
      c->copyUs   += std::chrono::duration<double, std::micro>(t2 - t1).count();
      c->nStat++;
    }
    if (!due.empty() && !scn->LegacyAutoGpuFlush()) scn->PostRender();
    const auto now = std::chrono::steady_clock::now();
    this->inPluginUs += std::chrono::duration<double, std::micro>(now - cycleStart).count();
    this->nCycles++;
    if (now - this->lastStat > std::chrono::seconds(5))
    {
      const double dt = std::chrono::duration<double>(now - this->lastStat).count();
      this->lastStat = now;
      for (auto &c : this->cams)
      {
        if (!c->nStat) continue;
        const uint32_t dues = c->dueCount.exchange(0, std::memory_order_relaxed);
        fprintf(stderr, "[ShmCameraExport] %s: %.1f fps, render %.2f ms + readback(%s) %.2f ms per frame"
                        " [issue %.2f, fence-wait %.2f, map+convert %.2f] due %u / rendered %u%s\n",
                c->shortName.c_str(), c->nStat / dt, c->renderUs / c->nStat / 1000.0,
                useAsync ? "async" : "sync", c->copyUs / c->nStat / 1000.0,
                c->issueUs / c->nStat / 1000.0, c->waitUs / c->nStat / 1000.0, c->mapUs / c->nStat / 1000.0,
                dues, c->nStat, (dues > c->nStat + 1) ? "  <-- SKIPPING" : "");
        c->renderUs = c->copyUs = c->issueUs = c->waitUs = c->mapUs = 0; c->nStat = 0;
      }
      if (this->nCycles)
        fprintf(stderr, "[ShmCameraExport] cycle: %.2f ms between PostRender passes, %.2f ms in plugin -> %.2f ms outside\n",
                this->cycleUs / this->nCycles / 1000.0, this->inPluginUs / this->nCycles / 1000.0,
                (this->cycleUs - this->inPluginUs) / this->nCycles / 1000.0);
      this->cycleUs = this->inPluginUs = 0; this->nCycles = 0;
    }
  }

  // Issue this frame's download into the ring, collect the previous one.
  private: void AsyncReadback(Cam &_c, rendering::Camera &_cam, int64_t _simNs)
  {
    const uint32_t w = _cam.ImageWidth(), h = _cam.ImageHeight();
    const GLuint tex = _cam.RenderTextureGLId();
    // Download NATIVE RGBA: asking the driver for GL_RGB from an RGBA texture
    // silently turns the transfer into a synchronous CPU repack (measured:
    // no gain over Camera::Copy). RGBA DMA's asynchronously; we drop the
    // alpha ourselves while copying into the segment.
    const GLsizeiptr bytes = static_cast<GLsizeiptr>(w) * h * 4;
    GLint prevPbo = 0, prevAlign = 4;
    g_gl.GetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &prevPbo);
    g_gl.GetIntegerv(GL_PACK_ALIGNMENT, &prevAlign);
    if (!_c.ringInit || _c.ringW != w || _c.ringH != h)
    {
      if (!_c.ringInit) g_gl.GenBuffers(2, _c.pbo);
      for (int i = 0; i < 2; ++i)
      {
        g_gl.BindBuffer(GL_PIXEL_PACK_BUFFER, _c.pbo[i]);
        g_gl.BufferData(GL_PIXEL_PACK_BUFFER, bytes, nullptr, GL_STREAM_READ);
        if (_c.fence[i]) { g_gl.DeleteSync(_c.fence[i]); _c.fence[i] = nullptr; }
      }
      _c.ringInit = true; _c.ringW = w; _c.ringH = h; _c.ringIdx = 0;
      while (g_gl.GetError() != GL_NO_ERROR) {}
    }
    g_gl.PixelStorei(GL_PACK_ALIGNMENT, 1);            // rows of w*3 bytes, unpadded
    // 1. issue: this frame → pbo[cur] (returns immediately)
    const int cur = _c.ringIdx, prev = cur ^ 1;
    const auto ta = std::chrono::steady_clock::now();
    g_gl.BindBuffer(GL_PIXEL_PACK_BUFFER, _c.pbo[cur]);
    g_gl.GetTextureImage(tex, 0, GL_RGBA, GL_UNSIGNED_BYTE, static_cast<GLsizei>(bytes), nullptr);
    if (_c.fence[cur]) g_gl.DeleteSync(_c.fence[cur]);
    _c.fence[cur] = g_gl.FenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    // Kick the GPU NOW. Without this the driver keeps the render + download
    // queued until the next flush, and the "async" fence wait on the next
    // pass ends up executing them synchronously (measured 2.6-3.1 ms even
    // on a trivial scene). With it, the work overlaps the next camera's CPU
    // submission and the next sim step.
    g_gl.Flush();
    _c.ringSimNs[cur] = _simNs;
    const auto tb = std::chrono::steady_clock::now();
    // 2. collect: previous frame's download finished during this render
    if (_c.fence[prev])
    {
      g_gl.ClientWaitSync(_c.fence[prev], GL_SYNC_FLUSH_COMMANDS_BIT, 1000000000ull);
      g_gl.DeleteSync(_c.fence[prev]); _c.fence[prev] = nullptr;
      const auto tc = std::chrono::steady_clock::now();
      g_gl.BindBuffer(GL_PIXEL_PACK_BUFFER, _c.pbo[prev]);
      const void *p = g_gl.MapBufferRange(GL_PIXEL_PACK_BUFFER, 0, bytes, GL_MAP_READ_BIT);
      if (p)
      {
        // Native RGBA straight into the segment (one memcpy); the bridge's
        // --shm-source reader strips alpha in ITS process, off the render thread.
        this->WriteShm(_c, static_cast<const uint8_t *>(p), static_cast<size_t>(bytes),
                       w, h, rendering::PF_R8G8B8A8, _c.ringSimNs[prev]);
        g_gl.UnmapBuffer(GL_PIXEL_PACK_BUFFER);
      }
      const auto td = std::chrono::steady_clock::now();
      _c.waitUs += std::chrono::duration<double, std::micro>(tc - tb).count();
      _c.mapUs  += std::chrono::duration<double, std::micro>(td - tc).count();
    }
    _c.issueUs += std::chrono::duration<double, std::micro>(tb - ta).count();
    _c.ringIdx = prev;
    g_gl.BindBuffer(GL_PIXEL_PACK_BUFFER, static_cast<GLuint>(prevPbo));
    g_gl.PixelStorei(GL_PACK_ALIGNMENT, prevAlign);
    const GLenum err = g_gl.GetError();
    if (err != GL_NO_ERROR && !_c.warnedNoCam)
    {
      fprintf(stderr, "[ShmCameraExport] GL error 0x%x in async readback of '%s'\n",
              err, _c.shortName.c_str());
      _c.warnedNoCam = true;
    }
  }

  private: rendering::CameraPtr FindCamera(Cam &_c, rendering::Scene &_scene)
  {
    rendering::SensorPtr s;
    if (!_c.resolvedName.empty())
      s = _scene.SensorByName(_c.resolvedName);
    if (!s)
    {
      s = _scene.SensorByName(_c.scoped);
      if (!s)
      {
        const std::string suffix = "::" + _c.shortName;
        for (unsigned int i = 0; i < _scene.SensorCount(); ++i)
        {
          auto cand = _scene.SensorByIndex(i);
          if (!cand) continue;
          const std::string &n = cand->Name();
          if (n == _c.shortName ||
              (n.size() > suffix.size() &&
               n.compare(n.size() - suffix.size(), suffix.size(), suffix) == 0))
          { s = cand; break; }
        }
      }
      if (!s)
      {
        if (!_c.warnedNoCam)
        {
          fprintf(stderr, "[ShmCameraExport] rendering camera '%s' not found yet\n",
                  _c.scoped.c_str());
          _c.warnedNoCam = true;
        }
        return nullptr;
      }
      _c.resolvedName = s->Name();
    }
    auto cam = std::dynamic_pointer_cast<rendering::Camera>(s);
    if (!cam && !_c.warnedNoCam)
    {
      fprintf(stderr, "[ShmCameraExport] '%s' is not a rendering::Camera\n",
              _c.scoped.c_str());
      _c.warnedNoCam = true;
    }
    return cam;
  }

  private: static bool FormatInfo(rendering::PixelFormat _f, uint32_t &_ch,
                                  const char *&_name)
  {
    switch (_f)
    {
      case rendering::PF_R8G8B8:   _ch = 3; _name = "rgb24"; return true;
      case rendering::PF_B8G8R8:   _ch = 3; _name = "bgr24"; return true;
      case rendering::PF_R8G8B8A8: _ch = 4; _name = "rgba";  return true;
      default: return false;
    }
  }

  private: bool OpenShm(Cam &_c, uint32_t _w, uint32_t _h, rendering::PixelFormat _f)
  {
    uint32_t ch = 0; const char *fmt = nullptr;
    if (!FormatInfo(_f, ch, fmt))
    {
      fprintf(stderr, "[ShmCameraExport] '%s': unsupported pixel format %d\n",
              _c.scoped.c_str(), static_cast<int>(_f));
      return false;
    }
    const uint32_t stride = _w * ch, frame = stride * _h;
    _c.size = sizeof(ShmHeader) + frame;
    shm_unlink(_c.shmName.c_str());
    _c.fd = shm_open(_c.shmName.c_str(), O_CREAT | O_RDWR, 0666);
    if (_c.fd < 0) { perror("[ShmCameraExport] shm_open"); return false; }
    if (ftruncate(_c.fd, static_cast<off_t>(_c.size)) < 0)
    {
      perror("[ShmCameraExport] ftruncate");
      close(_c.fd); _c.fd = -1; shm_unlink(_c.shmName.c_str()); return false;
    }
    void *p = mmap(nullptr, _c.size, PROT_READ | PROT_WRITE, MAP_SHARED, _c.fd, 0);
    if (p == MAP_FAILED)
    {
      perror("[ShmCameraExport] mmap");
      close(_c.fd); _c.fd = -1; shm_unlink(_c.shmName.c_str()); return false;
    }
    _c.ptr = static_cast<uint8_t *>(p);
    auto *hdr = reinterpret_cast<ShmHeader *>(_c.ptr);
    std::memset(hdr, 0, sizeof(ShmHeader));
    hdr->magic = 0x475A4652u;
    hdr->width = _w; hdr->height = _h; hdr->channels = ch;
    hdr->stride = stride; hdr->frame_size = frame;
    std::strncpy(hdr->pix_fmt, fmt, sizeof(hdr->pix_fmt) - 1);
    fprintf(stderr, "[ShmCameraExport] %s: %ux%u %s (%u bytes/frame)\n",
            _c.shmName.c_str(), _w, _h, fmt, frame);
    return true;
  }

  // RGBA (mapped PBO) -> rgb24 segment, converting in the copy loop.
  private: void WriteShmFromRGBA(Cam &_c, const uint8_t *_rgba, uint32_t _w,
                                 uint32_t _h, int64_t _simNs)
  {
    if (!_c.ptr && !this->OpenShm(_c, _w, _h, rendering::PF_R8G8B8)) return;
    auto *hdr = reinterpret_cast<ShmHeader *>(_c.ptr);
    uint8_t *dst = _c.ptr + sizeof(ShmHeader);
    const size_t n = static_cast<size_t>(_w) * _h;
    if (hdr->frame_size < n * 3) return;
    for (size_t i = 0; i < n; ++i)
    {
      dst[3 * i]     = _rgba[4 * i];
      dst[3 * i + 1] = _rgba[4 * i + 1];
      dst[3 * i + 2] = _rgba[4 * i + 2];
    }
    __atomic_thread_fence(__ATOMIC_RELEASE);
    hdr->timestamp_ns = static_cast<uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    hdr->sim_time_ns = static_cast<uint64_t>(_simNs);
    hdr->sequence = ++_c.seq;
  }

  private: void WriteShm(Cam &_c, const uint8_t *_data, size_t _n,
                         uint32_t _w, uint32_t _h, rendering::PixelFormat _f,
                         int64_t _simNs)
  {
    if (!_c.ptr && !this->OpenShm(_c, _w, _h, _f)) return;
    auto *hdr = reinterpret_cast<ShmHeader *>(_c.ptr);
    std::memcpy(_c.ptr + sizeof(ShmHeader), _data, std::min<size_t>(_n, hdr->frame_size));
    __atomic_thread_fence(__ATOMIC_RELEASE);
    hdr->timestamp_ns = static_cast<uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    hdr->sim_time_ns = static_cast<uint64_t>(_simNs);
    hdr->sequence = ++_c.seq;
  }

  private: void CloseShm(Cam &_c)
  {
    if (_c.ptr) { munmap(_c.ptr, _c.size); _c.ptr = nullptr; }
    if (_c.fd >= 0) { close(_c.fd); _c.fd = -1; }
    if (!_c.shmName.empty()) shm_unlink(_c.shmName.c_str());
  }

  private: Entity model{kNullEntity};
  private: std::string modelName;
  private: std::set<std::string> filter;
  private: bool forceSync{false};
  private: std::vector<std::unique_ptr<Cam>> cams;
  private: std::mutex enumMutex;
  private: std::atomic<bool> enumerated{false};
  private: common::ConnectionPtr postRenderConn;
  private: std::chrono::steady_clock::time_point lastStat{std::chrono::steady_clock::now()};
  private: std::chrono::steady_clock::time_point lastCycle{};
  private: double cycleUs{0}, inPluginUs{0}; uint32_t nCycles{0};
};

}  // namespace systems
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(gz::sim::systems::ShmCameraExportPlugin,
              gz::sim::System,
              gz::sim::systems::ShmCameraExportPlugin::ISystemConfigure,
              gz::sim::systems::ShmCameraExportPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::ShmCameraExportPlugin,
                    "gz::sim::systems::ShmCameraExportPlugin")
