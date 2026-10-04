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
 * <warp> children carry gz_image_bridge's --warp-fisheye spec per sensor:
 * the fisheye warp then runs on the GPU right after the render (GpuWarp) and
 * the FINAL output is read back. If the GPU warp is unavailable or fails, or
 * with SHM_EXPORT_SYNC=1 / SHM_EXPORT_NO_GPU_WARP=1, the same LUT warp runs
 * on the CPU here (CpuWarp, byte-identical) — the bridge does not warp these
 * feeds, so a warp camera never exports an un-warped frame.
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
#include <cmath>
#include <map>
#include <sstream>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <algorithm>
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

// ── GPU fisheye warp (the <warp> elements) ───────────────────────────────────
// gz_image_bridge's --warp-fisheye, moved onto the GPU: the camera renders the
// rectilinear warp source; a fragment pass maps every OUTPUT pixel through a
// per-pixel LUT (built here with the bridge's exact double-precision maths:
// source texel + 8.8 fixed-point bilinear weights) and blends the four texels
// with the bridge's integer arithmetic — so the warped frame is the same bytes
// the CPU path produced, without a CPU core per camera (measured: the bridges'
// CPU warp cost 52.8 -> 84.4 fps on a loaded 4-camera fleet).
struct GlWarpFns
{
  PFNGLCREATESHADERPROC CreateShader{nullptr};
  PFNGLSHADERSOURCEPROC ShaderSource{nullptr};
  PFNGLCOMPILESHADERPROC CompileShader{nullptr};
  PFNGLGETSHADERIVPROC GetShaderiv{nullptr};
  PFNGLGETSHADERINFOLOGPROC GetShaderInfoLog{nullptr};
  PFNGLCREATEPROGRAMPROC CreateProgram{nullptr};
  PFNGLATTACHSHADERPROC AttachShader{nullptr};
  PFNGLLINKPROGRAMPROC LinkProgram{nullptr};
  PFNGLGETPROGRAMIVPROC GetProgramiv{nullptr};
  PFNGLGETPROGRAMINFOLOGPROC GetProgramInfoLog{nullptr};
  PFNGLUSEPROGRAMPROC UseProgram{nullptr};
  PFNGLGETUNIFORMLOCATIONPROC GetUniformLocation{nullptr};
  PFNGLUNIFORM1IPROC Uniform1i{nullptr};
  PFNGLGENFRAMEBUFFERSPROC GenFramebuffers{nullptr};
  PFNGLBINDFRAMEBUFFERPROC BindFramebuffer{nullptr};
  PFNGLFRAMEBUFFERTEXTURE2DPROC FramebufferTexture2D{nullptr};
  PFNGLCHECKFRAMEBUFFERSTATUSPROC CheckFramebufferStatus{nullptr};
  PFNGLGENVERTEXARRAYSPROC GenVertexArrays{nullptr};
  PFNGLBINDVERTEXARRAYPROC BindVertexArray{nullptr};
  PFNGLACTIVETEXTUREPROC ActiveTexture{nullptr};
  PFNGLGENSAMPLERSPROC GenSamplers{nullptr};
  PFNGLBINDSAMPLERPROC BindSampler{nullptr};
  PFNGLSAMPLERPARAMETERIPROC SamplerParameteri{nullptr};
  PFNGLGETTEXTURELEVELPARAMETERIVPROC GetTextureLevelParameteriv{nullptr};
  PFNGLGETTEXTUREPARAMETERIVPROC GetTextureParameteriv{nullptr};
  PFNGLTEXTUREPARAMETERIPROC TextureParameteri{nullptr};
  PFNGLTEXTUREVIEWPROC TextureView{nullptr};
  void (*GenTextures)(GLsizei, GLuint *){nullptr};
  void (*BindTexture)(GLenum, GLuint){nullptr};
  void (*TexImage2D)(GLenum, GLint, GLint, GLsizei, GLsizei, GLint, GLenum, GLenum, const void *){nullptr};
  void (*TexParameteri)(GLenum, GLenum, GLint){nullptr};
  void (*Viewport)(GLint, GLint, GLsizei, GLsizei){nullptr};
  void (*DrawArrays)(GLenum, GLint, GLsizei){nullptr};
  void (*Enable)(GLenum){nullptr};
  void (*Disable)(GLenum){nullptr};
  GLboolean (*IsEnabled)(GLenum){nullptr};
  void (*ColorMask)(GLboolean, GLboolean, GLboolean, GLboolean){nullptr};
  void (*GetBooleanv)(GLenum, GLboolean *){nullptr};
  bool tried{false}, ok{false};
  GLuint prog{0}, vao{0}, sampler{0};
  GLint locSrc{-1}, locLut{-1};

  bool Load()
  {
    if (this->tried) return this->ok;
    this->tried = true;
#define GZ_WFN(field, name, type) \
    this->field = reinterpret_cast<type>(GlFns::Proc(name)); if (!this->field) { \
      fprintf(stderr, "[ShmCameraExport] GL entry point %s unavailable -> no GPU warp\n", name); return false; }
    GZ_WFN(CreateShader, "glCreateShader", PFNGLCREATESHADERPROC)
    GZ_WFN(ShaderSource, "glShaderSource", PFNGLSHADERSOURCEPROC)
    GZ_WFN(CompileShader, "glCompileShader", PFNGLCOMPILESHADERPROC)
    GZ_WFN(GetShaderiv, "glGetShaderiv", PFNGLGETSHADERIVPROC)
    GZ_WFN(GetShaderInfoLog, "glGetShaderInfoLog", PFNGLGETSHADERINFOLOGPROC)
    GZ_WFN(CreateProgram, "glCreateProgram", PFNGLCREATEPROGRAMPROC)
    GZ_WFN(AttachShader, "glAttachShader", PFNGLATTACHSHADERPROC)
    GZ_WFN(LinkProgram, "glLinkProgram", PFNGLLINKPROGRAMPROC)
    GZ_WFN(GetProgramiv, "glGetProgramiv", PFNGLGETPROGRAMIVPROC)
    GZ_WFN(GetProgramInfoLog, "glGetProgramInfoLog", PFNGLGETPROGRAMINFOLOGPROC)
    GZ_WFN(UseProgram, "glUseProgram", PFNGLUSEPROGRAMPROC)
    GZ_WFN(GetUniformLocation, "glGetUniformLocation", PFNGLGETUNIFORMLOCATIONPROC)
    GZ_WFN(Uniform1i, "glUniform1i", PFNGLUNIFORM1IPROC)
    GZ_WFN(GenFramebuffers, "glGenFramebuffers", PFNGLGENFRAMEBUFFERSPROC)
    GZ_WFN(BindFramebuffer, "glBindFramebuffer", PFNGLBINDFRAMEBUFFERPROC)
    GZ_WFN(FramebufferTexture2D, "glFramebufferTexture2D", PFNGLFRAMEBUFFERTEXTURE2DPROC)
    GZ_WFN(CheckFramebufferStatus, "glCheckFramebufferStatus", PFNGLCHECKFRAMEBUFFERSTATUSPROC)
    GZ_WFN(GenVertexArrays, "glGenVertexArrays", PFNGLGENVERTEXARRAYSPROC)
    GZ_WFN(BindVertexArray, "glBindVertexArray", PFNGLBINDVERTEXARRAYPROC)
    GZ_WFN(ActiveTexture, "glActiveTexture", PFNGLACTIVETEXTUREPROC)
    GZ_WFN(GenSamplers, "glGenSamplers", PFNGLGENSAMPLERSPROC)
    GZ_WFN(BindSampler, "glBindSampler", PFNGLBINDSAMPLERPROC)
    GZ_WFN(SamplerParameteri, "glSamplerParameteri", PFNGLSAMPLERPARAMETERIPROC)
    GZ_WFN(GetTextureLevelParameteriv, "glGetTextureLevelParameteriv", PFNGLGETTEXTURELEVELPARAMETERIVPROC)
    GZ_WFN(GetTextureParameteriv, "glGetTextureParameteriv", PFNGLGETTEXTUREPARAMETERIVPROC)
    GZ_WFN(TextureParameteri, "glTextureParameteri", PFNGLTEXTUREPARAMETERIPROC)
    GZ_WFN(TextureView, "glTextureView", PFNGLTEXTUREVIEWPROC)
    GZ_WFN(GenTextures, "glGenTextures", void (*)(GLsizei, GLuint *))
    GZ_WFN(BindTexture, "glBindTexture", void (*)(GLenum, GLuint))
    GZ_WFN(TexImage2D, "glTexImage2D", void (*)(GLenum, GLint, GLint, GLsizei, GLsizei, GLint, GLenum, GLenum, const void *))
    GZ_WFN(TexParameteri, "glTexParameteri", void (*)(GLenum, GLenum, GLint))
    GZ_WFN(Viewport, "glViewport", void (*)(GLint, GLint, GLsizei, GLsizei))
    GZ_WFN(DrawArrays, "glDrawArrays", void (*)(GLenum, GLint, GLsizei))
    GZ_WFN(Enable, "glEnable", void (*)(GLenum))
    GZ_WFN(Disable, "glDisable", void (*)(GLenum))
    GZ_WFN(IsEnabled, "glIsEnabled", GLboolean (*)(GLenum))
    GZ_WFN(ColorMask, "glColorMask", void (*)(GLboolean, GLboolean, GLboolean, GLboolean))
    GZ_WFN(GetBooleanv, "glGetBooleanv", void (*)(GLenum, GLboolean *))
#undef GZ_WFN
    if (!this->BuildProgram()) return false;
    this->ok = true;
    return true;
  }

  GLuint Compile(GLenum type, const char *src)
  {
    GLuint sh = this->CreateShader(type);
    this->ShaderSource(sh, 1, &src, nullptr);
    this->CompileShader(sh);
    GLint okc = 0;
    this->GetShaderiv(sh, GL_COMPILE_STATUS, &okc);
    if (!okc)
    {
      char log[2048] = {0};
      this->GetShaderInfoLog(sh, sizeof(log) - 1, nullptr, log);
      fprintf(stderr, "[ShmCameraExport] GPU warp shader compile failed: %s\n", log);
      return 0;
    }
    return sh;
  }

  bool BuildProgram()
  {
    // Fullscreen triangle from gl_VertexID (no vertex buffers).
    static const char *vs =
      "#version 330 core\n"
      "void main() {\n"
      "  vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);\n"
      "  gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);\n"
      "}\n";
    // Output texel (x, y) <- LUT(x, y) = (ix, iy, wx, wy): the bridge's
    // integer bilinear, value = (p00*w00 + p10*w10 + p01*w01 + p11*w11) >> 16.
    // Rows are texel rows = the image rows the readback writes, so no flips.
    static const char *fs =
      "#version 330 core\n"
      "uniform sampler2D src;\n"
      "uniform usampler2D lut;\n"
      "out vec4 o;\n"
      "uvec4 px(ivec2 q) { return uvec4(round(texelFetch(src, q, 0) * 255.0)); }\n"
      "void main() {\n"
      "  uvec4 t = texelFetch(lut, ivec2(gl_FragCoord.xy), 0);\n"
      "  ivec2 s = ivec2(t.xy);\n"
      "  uint wx = t.z, wy = t.w;\n"
      "  uint w00 = (256u - wx) * (256u - wy), w10 = wx * (256u - wy);\n"
      "  uint w01 = (256u - wx) * wy,         w11 = wx * wy;\n"
      "  uvec4 v = (px(s) * w00 + px(s + ivec2(1, 0)) * w10\n"
      "           + px(s + ivec2(0, 1)) * w01 + px(s + ivec2(1, 1)) * w11) >> 16u;\n"
      "  o = vec4(vec3(v.rgb) / 255.0, 1.0);\n"
      "}\n";
    GLuint v = this->Compile(GL_VERTEX_SHADER, vs);
    GLuint f = this->Compile(GL_FRAGMENT_SHADER, fs);
    if (!v || !f) return false;
    this->prog = this->CreateProgram();
    this->AttachShader(this->prog, v);
    this->AttachShader(this->prog, f);
    this->LinkProgram(this->prog);
    GLint okl = 0;
    this->GetProgramiv(this->prog, GL_LINK_STATUS, &okl);
    if (!okl)
    {
      char log[2048] = {0};
      this->GetProgramInfoLog(this->prog, sizeof(log) - 1, nullptr, log);
      fprintf(stderr, "[ShmCameraExport] GPU warp program link failed: %s\n", log);
      return false;
    }
    this->locSrc = this->GetUniformLocation(this->prog, "src");
    this->locLut = this->GetUniformLocation(this->prog, "lut");
    this->GenVertexArrays(1, &this->vao);
    // Our own NEAREST sampler on both units: Ogre binds sampler objects (and
    // its textures may carry mipmap filters without mip levels) — either can
    // make a texture "incomplete", and texelFetch on that returns zeros.
    this->GenSamplers(1, &this->sampler);
    this->SamplerParameteri(this->sampler, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    this->SamplerParameteri(this->sampler, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    this->SamplerParameteri(this->sampler, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    this->SamplerParameteri(this->sampler, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    return true;
  }
};
static GlWarpFns g_wgl;

// gz_image_bridge's --warp-fisheye spec + the output size.
struct WarpSpec
{
  double c1{1}, c2{1}, c3{0};
  char fun{'i'};
  double outHfovDeg{90}, virtH{0}, srcHfovDeg{90};
  uint32_t srcW{0}, srcH{0};
  double ppDx{0}, ppDy{0};
  uint32_t outW{0}, outH{0};
};

static bool ParseWarpSpec(const std::string &_s, WarpSpec &_w)
{
  // "c1,c2,c3,fun,out_hfov,virt_h,src_hfov,src_w,src_h[,pp_dx,pp_dy]"
  std::vector<std::string> f;
  std::stringstream ss(_s);
  std::string tok;
  while (std::getline(ss, tok, ',')) f.push_back(tok);
  if (f.size() != 9 && f.size() != 11) return false;
  try
  {
    _w.c1 = std::stod(f[0]); _w.c2 = std::stod(f[1]); _w.c3 = std::stod(f[2]);
    _w.fun = f[3].empty() ? 'i' : (f[3][0] == 's' ? 's' : f[3][0] == 't' ? 't' : 'i');
    _w.outHfovDeg = std::stod(f[4]); _w.virtH = std::stod(f[5]);
    _w.srcHfovDeg = std::stod(f[6]);
    _w.srcW = static_cast<uint32_t>(std::stoul(f[7]));
    _w.srcH = static_cast<uint32_t>(std::stoul(f[8]));
    if (f.size() == 11) { _w.ppDx = std::stod(f[9]); _w.ppDy = std::stod(f[10]); }
  }
  catch (...) { return false; }
  return true;
}

// The bridge's LUT, line for line (gz_image_bridge.cc warpFisheyeFromRect):
// RGBA16UI texels (ix, iy, wx, wy), wx/wy = 8.8 weights.
static std::vector<uint16_t> BuildWarpLut(const WarpSpec &_w)
{
  const uint32_t src_w = _w.srcW, src_h = _w.srcH, dst_w = _w.outW, dst_h = _w.outH;
  std::vector<uint16_t> lut(static_cast<size_t>(dst_w) * dst_h * 4, 0);
  const double th_e = (_w.outHfovDeg * M_PI / 180.0) / 2.0;
  auto fwd = [](double a, char fn) {
    return fn == 's' ? sin(a) : fn == 't' ? tan(a) : a; };
  auto inv = [](double r, char fn) {
    return fn == 's' ? asin(r < -1 ? -1 : (r > 1 ? 1 : r))
         : fn == 't' ? atan(r) : r; };
  const double f_fish = (dst_w / 2.0) / (_w.c1 * fwd(th_e / _w.c2 + _w.c3, _w.fun));
  const double f_src = (src_w / 2.0) / tan((_w.srcHfovDeg * M_PI / 180.0) / 2.0);
  const double virt_h = _w.virtH > 0 ? _w.virtH : dst_h;
  const double cx_d = (dst_w - 1) / 2.0 + _w.ppDx;
  const double cy_d = (dst_h - 1) / 2.0 + _w.ppDy;
  const double cx_s = (src_w - 1) / 2.0, cy_s = (src_h - 1) / 2.0;
  for (uint32_t y = 0; y < dst_h; y++)
  {
    const double vy = (y - cy_d) * (virt_h / dst_h);
    for (uint32_t x = 0; x < dst_w; x++)
    {
      const double vx = x - cx_d;
      const double r = sqrt(vx * vx + vy * vy);
      double sx, sy;
      if (r < 1e-9) { sx = cx_s; sy = cy_s; }
      else
      {
        const double th = _w.c2 * (inv(r / (_w.c1 * f_fish), _w.fun) - _w.c3);
        const double t = tan(th < 0 ? 0 : th);
        sx = cx_s + f_src * t * (vx / r);
        sy = cy_s + f_src * t * (vy / r);
      }
      if (sx < 0) sx = 0; if (sx > src_w - 1.001) sx = src_w - 1.001;
      if (sy < 0) sy = 0; if (sy > src_h - 1.001) sy = src_h - 1.001;
      const uint32_t ix = static_cast<uint32_t>(sx);
      const uint32_t iy = static_cast<uint32_t>(sy);
      uint16_t *t = &lut[(static_cast<size_t>(y) * dst_w + x) * 4];
      t[0] = static_cast<uint16_t>(ix);
      t[1] = static_cast<uint16_t>(iy);
      t[2] = static_cast<uint16_t>((sx - ix) * 256.0);
      t[3] = static_cast<uint16_t>((sy - iy) * 256.0);
    }
  }
  return lut;
}

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
    // GPU fisheye warp (<warp> element): LUT + output texture + FBO
    bool hasWarp{false};
    WarpSpec warp;
    GLuint lutTex{0}, outTex{0}, fbo{0};
    GLuint srcView{0}, srcViewOf{0};   // GL_RGBA8 view of the sRGB camera texture
    bool gpuReady{false}, gpuFailed{false};
    double warpUs{0};
    // CPU fallback for the same warp (GPU warp failed, or SHM_EXPORT_SYNC):
    // the bridge no longer warps these feeds, so the plugin must never export
    // an un-warped frame for a warp camera.
    std::vector<uint16_t> cpuLut;
    std::vector<uint8_t> cpuOut;
    bool ringWarped[2]{false, false};   // async slot already GPU-warped?
    bool cpuWarpBad{false};             // source size != spec: frames dropped
  };

  // ── one render pass for EVERY instance ───────────────────────────────
  // A fleet world carries one instance per drone model. Each used to bracket
  // its own cameras with a full Scene::PreRender()/PostRender() (scene update
  // + GPU flush) — N brackets per pass for N drones. Now the FIRST registered
  // instance (the leader) renders all instances' due cameras inside ONE
  // bracket; the others' PostRender handlers return at once. The registry
  // mutex is held for the whole pass, so an instance being destroyed (it
  // unregisters under the same lock) can never be torn down mid-render; if
  // the leader goes, the next instance leads.
  private: static std::mutex &RegMutex() { static std::mutex m; return m; }
  private: static std::vector<ShmCameraExportPlugin *> &Registry()
  {
    static std::vector<ShmCameraExportPlugin *> r;
    return r;
  }

  public: ~ShmCameraExportPlugin() override
  {
    // Detach from the render thread FIRST, then the segments. GL objects are
    // deliberately abandoned (the context may already be gone).
    this->postRenderConn.reset();
    {
      std::lock_guard<std::mutex> lk(RegMutex());
      auto &r = Registry();
      r.erase(std::remove(r.begin(), r.end(), this), r.end());
    }
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
      auto wel = _sdf->FindElement("warp");
      while (wel)
      {
        WarpSpec w;
        const std::string sensor = wel->GetAttribute("sensor")
            ? wel->GetAttribute("sensor")->GetAsString() : "";
        const std::string spec = wel->Get<std::string>();
        auto aw = wel->GetAttribute("out_w");
        auto ah = wel->GetAttribute("out_h");
        try
        {
          w.outW = aw ? static_cast<uint32_t>(std::stoul(aw->GetAsString())) : 0;
          w.outH = ah ? static_cast<uint32_t>(std::stoul(ah->GetAsString())) : 0;
        }
        catch (...) { w.outW = w.outH = 0; }
        if (!sensor.empty() && w.outW && w.outH && ParseWarpSpec(spec, w))
          this->warps[sensor] = w;
        else
          fprintf(stderr, "[ShmCameraExport] ignoring malformed <warp> '%s' on '%s'\n",
                  spec.c_str(), sensor.c_str());
        wel = wel->GetNextElement("warp");
      }
    }
    const char *sync = std::getenv("SHM_EXPORT_SYNC");
    this->forceSync = sync && *sync && *sync != '0';
    const char *nogw = std::getenv("SHM_EXPORT_NO_GPU_WARP");
    this->noGpuWarp = nogw && *nogw && *nogw != '0';
    {
      std::lock_guard<std::mutex> lk(RegMutex());
      Registry().push_back(this);
    }
    this->postRenderConn = _eventMgr.Connect<events::PostRender>(
        [this]() { this->OnPostRender(); });
    fprintf(stderr, "[ShmCameraExport] attached to model '%s'%s%s\n",
            this->modelName.c_str(),
            this->filter.empty() ? " (all cameras)" : " (filtered)",
            this->forceSync ? " [SYNC readback forced]" : "");
    if (this->noGpuWarp && !this->warps.empty())
      fprintf(stderr, "[ShmCameraExport] SHM_EXPORT_NO_GPU_WARP: '%s' warps on the CPU\n",
              this->modelName.c_str());
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
        auto wit = this->warps.find(sname);
        if (wit != this->warps.end())
        {
          c->hasWarp = true;
          c->warp = wit->second;
          fprintf(stderr, "[ShmCameraExport] camera '%s': GPU fisheye warp %ux%u -> %ux%u\n",
                  sname.c_str(), c->warp.srcW, c->warp.srcH, c->warp.outW, c->warp.outH);
        }
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
    std::lock_guard<std::mutex> lk(RegMutex());
    auto &reg = Registry();
    if (reg.empty() || reg.front() != this) return;      // not the leader
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
    std::vector<std::tuple<ShmCameraExportPlugin *, Cam *, rendering::CameraPtr, int64_t>> due;
    for (auto *inst : reg)
    {
      if (!inst->enumerated.load(std::memory_order_acquire)) continue;
      for (auto &c : inst->cams)
      {
        if (!c->due.load(std::memory_order_acquire)) continue;
        rendering::CameraPtr cam = inst->FindCamera(*c, *scn);
        if (!cam) continue;                                 // retry next pass
        const int64_t simNs = c->dueSimNs.load(std::memory_order_relaxed);
        c->due.store(false, std::memory_order_release);
        due.emplace_back(inst, c.get(), cam, simNs);
      }
    }
    if (!due.empty()) scn->PreRender();
    for (auto &[inst, cptr, cam, simNs] : due)
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
        GLuint tex = cam->RenderTextureGLId();
        uint32_t w = cam->ImageWidth(), h = cam->ImageHeight();
        bool warped = false;
        if (c->hasWarp && !c->gpuFailed && !inst->noGpuWarp)
        {
          const auto tw = std::chrono::steady_clock::now();
          if (inst->GpuWarp(*c, tex, w, h))
          { tex = c->outTex; w = c->warp.outW; h = c->warp.outH; warped = true; }
          c->warpUs += std::chrono::duration<double, std::micro>(
              std::chrono::steady_clock::now() - tw).count();
        }
        inst->AsyncReadback(*c, tex, w, h, simNs, warped);
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
        const uint8_t *data = static_cast<const uint8_t *>(c->image.Data());
        if (c->hasWarp)
        {
          const uint32_t ch = static_cast<uint32_t>(
              rendering::PixelUtil::ChannelCount(cam->ImageFormat()));
          if (inst->CpuWarp(*c, data, cam->ImageWidth(), cam->ImageHeight(), ch))
            inst->WriteShm(*c, c->cpuOut.data(), c->cpuOut.size(), c->warp.outW,
                           c->warp.outH, cam->ImageFormat(), simNs);
        }
        else
          inst->WriteShm(*c, data, c->image.MemorySize(), cam->ImageWidth(),
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
      for (auto *inst : reg)
      for (auto &c : inst->cams)
      {
        if (!c->nStat) continue;
        const uint32_t dues = c->dueCount.exchange(0, std::memory_order_relaxed);
        fprintf(stderr, "[ShmCameraExport] %s/%s: %.1f fps, render %.2f ms + %sreadback(%s) %.2f ms per frame"
                        " [issue %.2f, fence-wait %.2f, map+convert %.2f] due %u / rendered %u%s\n",
                inst->modelName.c_str(), c->shortName.c_str(), c->nStat / dt, c->renderUs / c->nStat / 1000.0,
                c->hasWarp ? ((c->gpuFailed || inst->noGpuWarp || !useAsync) ? "CPU-warp(fallback)+" : "gpu-warp+") : "",
                useAsync ? "async" : "sync", c->copyUs / c->nStat / 1000.0,
                c->issueUs / c->nStat / 1000.0, c->waitUs / c->nStat / 1000.0, c->mapUs / c->nStat / 1000.0,
                dues, c->nStat, (dues > c->nStat + 1) ? "  <-- SKIPPING" : "");
        c->renderUs = c->copyUs = c->issueUs = c->waitUs = c->mapUs = c->warpUs = 0; c->nStat = 0;
      }
      if (this->nCycles)
        fprintf(stderr, "[ShmCameraExport] cycle (%zu model(s)): %.2f ms between PostRender passes, %.2f ms in plugin -> %.2f ms outside\n",
                reg.size(), this->cycleUs / this->nCycles / 1000.0, this->inPluginUs / this->nCycles / 1000.0,
                (this->cycleUs - this->inPluginUs) / this->nCycles / 1000.0);
      this->cycleUs = this->inPluginUs = 0; this->nCycles = 0;
    }
  }

  // Warp _srcTex (the rectilinear camera image) into _c.outTex on the GPU.
  // Render thread, Ogre's context current. Every piece of GL state touched is
  // saved and restored exactly — Ogre caches GL state, so it must find the
  // context as it left it. False = warp unavailable (logged; the camera then
  // exports its raw image and the failure shows in the stats line).
  private: bool GpuWarp(Cam &_c, GLuint _srcTex, uint32_t _srcW, uint32_t _srcH)
  {
    if (!g_wgl.Load()) { _c.gpuFailed = true; return false; }
    const WarpSpec &W = _c.warp;
    if (_srcW != W.srcW || _srcH != W.srcH)
    {
      fprintf(stderr, "[ShmCameraExport] '%s': sensor is %ux%u but the warp expects %ux%u "
              "-> GPU warp disabled (re-render the models)\n",
              _c.shortName.c_str(), _srcW, _srcH, W.srcW, W.srcH);
      _c.gpuFailed = true;
      return false;
    }
    // Errors already pending belong to Ogre, not to this pass — clear them so
    // the end-of-pass check only sees ours.
    while (g_gl.GetError() != GL_NO_ERROR) {}
    // ── save state ──
    GLint prevProg = 0, prevDrawFb = 0, prevReadFb = 0, prevVao = 0, prevActive = 0;
    GLint prevVp[4] = {0, 0, 0, 0}, prevTex[2] = {0, 0}, prevSmp[2] = {0, 0};
    GLint prevUnpackBuf = 0, prevUnpackAlign = 4;
    // Ogre leaves its own unpack layout set (row length / skips): the LUT
    // upload must see a tightly packed buffer.
    const GLenum unpackParams[] = {GL_UNPACK_ROW_LENGTH, GL_UNPACK_SKIP_ROWS,
                                   GL_UNPACK_SKIP_PIXELS, GL_UNPACK_IMAGE_HEIGHT,
                                   GL_UNPACK_SKIP_IMAGES, GL_UNPACK_SWAP_BYTES,
                                   GL_UNPACK_LSB_FIRST};
    GLint prevUnpack[sizeof(unpackParams) / sizeof(unpackParams[0])] = {0};
    GLboolean prevMask[4] = {GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE};
    // Anything Ogre may leave enabled that could drop or alter our fragments:
    // clip distances (our VS writes none -> undefined), the sample mask,
    // alpha-to-coverage, depth clamp, logic ops, plus the obvious ones.
    const GLenum caps[] = {GL_BLEND, GL_DEPTH_TEST, GL_SCISSOR_TEST, GL_CULL_FACE,
                           GL_STENCIL_TEST, GL_FRAMEBUFFER_SRGB, GL_RASTERIZER_DISCARD,
                           GL_SAMPLE_MASK, GL_SAMPLE_ALPHA_TO_COVERAGE, GL_DEPTH_CLAMP,
                           GL_COLOR_LOGIC_OP, GL_POLYGON_OFFSET_FILL,
                           GL_CLIP_DISTANCE0, GL_CLIP_DISTANCE1, GL_CLIP_DISTANCE2,
                           GL_CLIP_DISTANCE3, GL_CLIP_DISTANCE4, GL_CLIP_DISTANCE5,
                           GL_CLIP_DISTANCE6, GL_CLIP_DISTANCE7};
    GLboolean prevCap[sizeof(caps) / sizeof(caps[0])];
    g_gl.GetIntegerv(GL_CURRENT_PROGRAM, &prevProg);
    g_gl.GetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &prevDrawFb);
    g_gl.GetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &prevReadFb);
    g_gl.GetIntegerv(GL_VERTEX_ARRAY_BINDING, &prevVao);
    g_gl.GetIntegerv(GL_VIEWPORT, prevVp);
    g_gl.GetIntegerv(GL_ACTIVE_TEXTURE, &prevActive);
    g_gl.GetIntegerv(GL_PIXEL_UNPACK_BUFFER_BINDING, &prevUnpackBuf);
    g_gl.GetIntegerv(GL_UNPACK_ALIGNMENT, &prevUnpackAlign);
    for (size_t i = 0; i < sizeof(unpackParams) / sizeof(unpackParams[0]); ++i)
      g_gl.GetIntegerv(unpackParams[i], &prevUnpack[i]);
    g_wgl.GetBooleanv(GL_COLOR_WRITEMASK, prevMask);
    for (int u = 0; u < 2; ++u)
    {
      g_wgl.ActiveTexture(GL_TEXTURE0 + u);
      g_gl.GetIntegerv(GL_TEXTURE_BINDING_2D, &prevTex[u]);
      g_gl.GetIntegerv(GL_SAMPLER_BINDING, &prevSmp[u]);
    }
    for (size_t i = 0; i < sizeof(caps) / sizeof(caps[0]); ++i)
      prevCap[i] = g_wgl.IsEnabled(caps[i]);

    bool ok = true;
    // ── one-time per camera: LUT + output texture + FBO ──
    if (!_c.gpuReady)
    {
      std::vector<uint16_t> lut = BuildWarpLut(W);
      g_gl.BindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
      g_gl.PixelStorei(GL_UNPACK_ALIGNMENT, 1);
      for (size_t i = 0; i < sizeof(unpackParams) / sizeof(unpackParams[0]); ++i)
        g_gl.PixelStorei(unpackParams[i], 0);
      g_wgl.ActiveTexture(GL_TEXTURE1);
      g_wgl.GenTextures(1, &_c.lutTex);
      g_wgl.BindTexture(GL_TEXTURE_2D, _c.lutTex);
      g_wgl.TexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16UI, W.outW, W.outH, 0,
                       GL_RGBA_INTEGER, GL_UNSIGNED_SHORT, lut.data());
      g_wgl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      g_wgl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      g_wgl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
      g_wgl.GenTextures(1, &_c.outTex);
      g_wgl.BindTexture(GL_TEXTURE_2D, _c.outTex);
      g_wgl.TexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, W.outW, W.outH, 0,
                       GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
      g_wgl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      g_wgl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      g_wgl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
      g_wgl.GenFramebuffers(1, &_c.fbo);
      g_wgl.BindFramebuffer(GL_FRAMEBUFFER, _c.fbo);
      g_wgl.FramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                                 _c.outTex, 0);
      const GLenum st = g_wgl.CheckFramebufferStatus(GL_FRAMEBUFFER);
      if (st != GL_FRAMEBUFFER_COMPLETE)
      {
        fprintf(stderr, "[ShmCameraExport] '%s': warp FBO incomplete (0x%x)\n",
                _c.shortName.c_str(), st);
        ok = false;
      }
      GLint fmt = 0;
      g_wgl.GetTextureLevelParameteriv(_srcTex, 0, GL_TEXTURE_INTERNAL_FORMAT, &fmt);
      fprintf(stderr, "[ShmCameraExport] '%s': GPU warp ready (source internal format 0x%x)\n",
              _c.shortName.c_str(), fmt);
      _c.gpuReady = ok;
    }

    GLuint readTex = _srcTex;
    if (ok)
    {
      // The camera texture is sRGB (GL_SRGB8_ALPHA8, measured) and texelFetch
      // DECODES it to linear — measured on NVIDIA even with SKIP_DECODE set on
      // texture and sampler. The CPU path saw the STORED bytes (glGetTexImage),
      // so read through a GL_RGBA8 view of the same storage: raw bytes, no
      // decode, no copy. (Views need immutable storage — Ogre's RTTs are.)
      GLint fmt = 0;
      g_wgl.GetTextureLevelParameteriv(_srcTex, 0, GL_TEXTURE_INTERNAL_FORMAT, &fmt);
      if (fmt == GL_SRGB8_ALPHA8)
      {
        if (_c.srcViewOf != _srcTex)
        {
          GLint immutable = 0;
          g_wgl.GetTextureParameteriv(_srcTex, GL_TEXTURE_IMMUTABLE_FORMAT, &immutable);
          if (!immutable)
          {
            fprintf(stderr, "[ShmCameraExport] '%s': sRGB camera texture is not immutable "
                    "-> no raw view, GPU warp disabled\n", _c.shortName.c_str());
            ok = false;
          }
          else
          {
            g_wgl.GenTextures(1, &_c.srcView);
            g_wgl.TextureView(_c.srcView, GL_TEXTURE_2D, _srcTex, GL_RGBA8, 0, 1, 0, 1);
            _c.srcViewOf = _srcTex;
          }
        }
        if (ok) readTex = _c.srcView;
      }
    }
    if (ok)
    {
      g_wgl.BindFramebuffer(GL_FRAMEBUFFER, _c.fbo);
      g_wgl.Viewport(0, 0, static_cast<GLsizei>(W.outW), static_cast<GLsizei>(W.outH));
      for (size_t i = 0; i < sizeof(caps) / sizeof(caps[0]); ++i) g_wgl.Disable(caps[i]);
      g_wgl.ColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
      g_wgl.UseProgram(g_wgl.prog);
      g_wgl.ActiveTexture(GL_TEXTURE0);
      g_wgl.BindTexture(GL_TEXTURE_2D, readTex);
      g_wgl.BindSampler(0, g_wgl.sampler);
      g_wgl.ActiveTexture(GL_TEXTURE1);
      g_wgl.BindTexture(GL_TEXTURE_2D, _c.lutTex);
      g_wgl.BindSampler(1, g_wgl.sampler);
      g_wgl.Uniform1i(g_wgl.locSrc, 0);
      g_wgl.Uniform1i(g_wgl.locLut, 1);
      g_wgl.BindVertexArray(g_wgl.vao);
      g_wgl.DrawArrays(GL_TRIANGLES, 0, 3);
    }

    // ── restore state ──
    g_wgl.BindVertexArray(static_cast<GLuint>(prevVao));
    g_wgl.UseProgram(static_cast<GLuint>(prevProg));
    for (int u = 0; u < 2; ++u)
    {
      g_wgl.ActiveTexture(GL_TEXTURE0 + u);
      g_wgl.BindTexture(GL_TEXTURE_2D, static_cast<GLuint>(prevTex[u]));
      g_wgl.BindSampler(u, static_cast<GLuint>(prevSmp[u]));
    }
    g_wgl.ActiveTexture(static_cast<GLenum>(prevActive));
    g_wgl.BindFramebuffer(GL_DRAW_FRAMEBUFFER, static_cast<GLuint>(prevDrawFb));
    g_wgl.BindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(prevReadFb));
    g_wgl.Viewport(prevVp[0], prevVp[1], prevVp[2], prevVp[3]);
    g_wgl.ColorMask(prevMask[0], prevMask[1], prevMask[2], prevMask[3]);
    for (size_t i = 0; i < sizeof(caps) / sizeof(caps[0]); ++i)
      prevCap[i] ? g_wgl.Enable(caps[i]) : g_wgl.Disable(caps[i]);
    g_gl.BindBuffer(GL_PIXEL_UNPACK_BUFFER, static_cast<GLuint>(prevUnpackBuf));
    g_gl.PixelStorei(GL_UNPACK_ALIGNMENT, prevUnpackAlign);
    for (size_t i = 0; i < sizeof(unpackParams) / sizeof(unpackParams[0]); ++i)
      g_gl.PixelStorei(unpackParams[i], prevUnpack[i]);
    const GLenum err = g_gl.GetError();
    if (err != GL_NO_ERROR)
    {
      fprintf(stderr, "[ShmCameraExport] '%s': GL error 0x%x in GPU warp -> disabled\n",
              _c.shortName.c_str(), err);
      ok = false;
    }
    if (!ok) _c.gpuFailed = true;
    return ok;
  }

  // The GPU warp's CPU twin: same LUT, same 8.8 integer bilinear as the
  // shader (and as gz_image_bridge --warp-fisheye), so the output is
  // byte-identical. Render thread; only runs when the GPU warp is unavailable
  // or SHM_EXPORT_SYNC is set. False = frame dropped (source size mismatch:
  // exporting it un-warped would feed the tracker wrong geometry).
  private: bool CpuWarp(Cam &_c, const uint8_t *_src, uint32_t _w, uint32_t _h, uint32_t _ch)
  {
    const WarpSpec &W = _c.warp;
    if (_w != W.srcW || _h != W.srcH || _ch < 3)
    {
      if (!_c.cpuWarpBad)
        fprintf(stderr, "[ShmCameraExport] '%s': sensor is %ux%u but the warp expects %ux%u "
                "-> frames DROPPED (re-render the models)\n",
                _c.shortName.c_str(), _w, _h, W.srcW, W.srcH);
      _c.cpuWarpBad = true;
      return false;
    }
    if (_c.cpuLut.empty())
    {
      _c.cpuLut = BuildWarpLut(W);
      fprintf(stderr, "[ShmCameraExport] '%s': fisheye warp on the CPU (render thread)\n",
              _c.shortName.c_str());
    }
    const size_t n = static_cast<size_t>(W.outW) * W.outH;
    _c.cpuOut.resize(n * _ch);
    const size_t row = static_cast<size_t>(_w) * _ch;
    const uint16_t *t = _c.cpuLut.data();
    uint8_t *d = _c.cpuOut.data();
    for (size_t i = 0; i < n; ++i, t += 4, d += _ch)
    {
      const uint32_t wx = t[2], wy = t[3];
      const uint32_t w00 = (256u - wx) * (256u - wy), w10 = wx * (256u - wy);
      const uint32_t w01 = (256u - wx) * wy, w11 = wx * wy;
      const uint8_t *p00 = _src + t[1] * row + static_cast<size_t>(t[0]) * _ch;
      const uint8_t *p10 = p00 + _ch, *p01 = p00 + row, *p11 = p01 + _ch;
      for (uint32_t k = 0; k < _ch; ++k)
        d[k] = static_cast<uint8_t>((p00[k] * w00 + p10[k] * w10 + p01[k] * w01 +
                                     p11[k] * w11) >> 16);
    }
    return true;
  }

  // Issue this frame's download into the ring, collect the previous one.
  private: void AsyncReadback(Cam &_c, GLuint tex, uint32_t w, uint32_t h, int64_t _simNs,
                              bool _warped)
  {
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
    _c.ringWarped[cur] = _warped;
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
        if (_c.hasWarp && !_c.ringWarped[prev])
        {
          if (this->CpuWarp(_c, static_cast<const uint8_t *>(p), w, h, 4))
            this->WriteShm(_c, _c.cpuOut.data(), _c.cpuOut.size(), _c.warp.outW,
                           _c.warp.outH, rendering::PF_R8G8B8A8, _c.ringSimNs[prev]);
        }
        else
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
  private: std::map<std::string, WarpSpec> warps;   // sensor -> <warp> spec
  private: bool forceSync{false};
  private: bool noGpuWarp{false};   // SHM_EXPORT_NO_GPU_WARP=1: CPU warp (A/B, fallback test)
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
