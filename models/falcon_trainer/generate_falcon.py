"""Generate the falcon_trainer RC-plane mesh (OBJ + MTL, committed asset).

Modeled on the HobbyKing Falcon Trainer 20cc (1860 mm span / 1550 mm length,
high wing, taildragger) in its red / black-checkerboard scheme — matching the
user's tracker screenshot of the real aircraft.

Geometry is built from LOFTED convex solids (not plain boxes): the fuselage
tapers toward the tail boom, wing + elevator taper toward the tips, and the
wing / elevator / fin leading edges are ROUNDED (semi-circular arc ribs).
Checkers are separate red/black solids (no textures -> no loader deps, and
they double as high-contrast tracker features). Face winding is auto-corrected
per face against the solid centroid (all solids are convex), so the mesh is
safe for single-sided rendering.

Body frame: +X = nose, +Y = left (span), +Z = up; AABB centred at the origin.
Extents exactly: length 1.550, span 1.860, height 0.500 m.
Output: meshes/falcon_trainer.obj (+ .mtl) next to this script.
"""
import math
import os

R, K, W, A = "red", "black", "white", "alu"

verts = []            # global vertex list
solids = []           # (mat, vstart, nverts, faces_local) — faces as local idx tuples

def add_solid(mat, pts, faces):
    """Register a convex solid; winding fixed per-face vs the solid centroid."""
    base = len(verts)
    verts.extend(pts)
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    cz = sum(p[2] for p in pts) / len(pts)
    fixed = []
    for f in faces:
        p0, p1, p2 = pts[f[0]], pts[f[1]], pts[f[2]]
        u = (p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2])
        v = (p2[0]-p0[0], p2[1]-p0[1], p2[2]-p0[2])
        n = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
        fc = (sum(pts[i][0] for i in f)/len(f) - cx,
              sum(pts[i][1] for i in f)/len(f) - cy,
              sum(pts[i][2] for i in f)/len(f) - cz)
        if n[0]*fc[0] + n[1]*fc[1] + n[2]*fc[2] < 0:
            f = tuple(reversed(f))
        fixed.append(f)
    solids.append((mat, base, fixed))

def box(mat, x0, x1, y0, y1, z0, z1):
    pts = [(x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0),
           (x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)]
    faces = [(0,3,2,1),(4,5,6,7),(0,1,5,4),(3,7,6,2),(1,2,6,5),(0,4,7,3)]
    add_solid(mat, pts, faces)

def loft(mat, poly_a, poly_b):
    """Prism between two same-length 3D polygons (side quads + end caps)."""
    n = len(poly_a)
    assert len(poly_b) == n
    pts = list(poly_a) + list(poly_b)
    faces = [tuple(range(n))]                       # cap A
    faces.append(tuple(range(n, 2*n)))              # cap B
    for i in range(n):
        j = (i + 1) % n
        faces.append((i, j, n + j, n + i))          # side quad
    add_solid(mat, pts, faces)

def wheel(mat, cx, cy, cz, r, hw, n=12):
    ring = [(cx + r*math.cos(2*math.pi*i/n), cz + r*math.sin(2*math.pi*i/n))
            for i in range(n)]
    loft(mat, [(px, cy - hw, pz) for px, pz in ring],
              [(px, cy + hw, pz) for px, pz in ring])

# ── airfoil-ish rib profiles (rounded leading edge, thin trailing edge) ─────
def rib_xz(y, xle, chord, zc, t, te_h, n_arc=6):
    """Full rib polygon at span station y: points in 3D, rounded LE at xle.
    Returns (full, front, rear) split at mid-chord for 2-row checkers."""
    r = t / 2.0
    xte = xle - chord
    cxa = xle - r                                   # LE arc centre x
    arc = [(cxa + r*math.cos(math.radians(a)), zc + r*math.sin(math.radians(a)))
           for a in [90 - 180.0*i/n_arc for i in range(n_arc + 1)]]  # 90 -> -90
    full2d = [(xte, zc - te_h/2.0), (xte, zc + te_h/2.0)] + arc
    xm = (cxa + xte) / 2.0                          # checker split station
    def z_at(x, sign):                              # straight surface LE->TE
        f = (cxa - x) / (cxa - xte)
        return zc + sign * (r + f * (te_h/2.0 - r))
    front2d = arc + [(xm, z_at(xm, -1)), (xm, z_at(xm, +1))]
    rear2d = [(xm, z_at(xm, +1)), (xm, z_at(xm, -1)),
              (xte, zc - te_h/2.0), (xte, zc + te_h/2.0)]
    to3 = lambda poly: [(px, y, pz) for px, pz in poly]
    return to3(full2d), to3(front2d), to3(rear2d)

def rib_xy(z, xle, chord, yc, t, te_h, n_arc=6):
    """Fin rib polygon at height z (profile in the x-y plane)."""
    r = t / 2.0
    xte = xle - chord
    cxa = xle - r
    arc = [(cxa + r*math.cos(math.radians(a)), yc + r*math.sin(math.radians(a)))
           for a in [90 - 180.0*i/n_arc for i in range(n_arc + 1)]]
    poly = [(xte, yc - te_h/2.0), (xte, yc + te_h/2.0)] + arc
    return [(px, py, z) for px, py in poly]

# ── fuselage: nose section + tapering tail boom (lofted rectangles) ─────────
def rect_yz(x, hy, z0, z1):
    return [(x, -hy, z0), (x, hy, z0), (x, hy, z1), (x, -hy, z1)]

loft(R, rect_yz(0.720, 0.062, -0.062, 0.072),     # blends into the cowl
        rect_yz(0.420, 0.075, -0.075, 0.085))     # full section
loft(R, rect_yz(0.420, 0.075, -0.075, 0.085),
        rect_yz(0.100, 0.075, -0.075, 0.085))     # constant under the wing
loft(R, rect_yz(0.100, 0.075, -0.075, 0.085),
        rect_yz(-0.700, 0.028, -0.005, 0.048))    # taper to the tail boom
box(K, 0.720, 0.752, -0.058, 0.058, -0.058, 0.058)  # cowl
box(K, 0.752, 0.775, -0.025, 0.025, -0.025, 0.025)  # spinner
box(K, 0.746, 0.758, -0.012, 0.012, -0.150, 0.150)  # prop blades

# ── high wing: 8 span columns x 2 chord rows, tapered + rounded LE ──────────
W_XLE, W_ZC, W_T, W_TEH = 0.370, 0.110, 0.050, 0.016
W_CROOT, W_CTIP, W_HALF = 0.350, 0.230, 0.930
def w_chord(y):
    return W_CROOT - (W_CROOT - W_CTIP) * (abs(y) / W_HALF)
cols = 8
for c in range(cols):
    y0 = -W_HALF + c * (2.0 * W_HALF / cols)
    y1 = y0 + 2.0 * W_HALF / cols
    _, f0, r0 = rib_xz(y0, W_XLE, w_chord(y0), W_ZC, W_T, W_TEH)
    _, f1, r1 = rib_xz(y1, W_XLE, w_chord(y1), W_ZC, W_T, W_TEH)
    loft(R if c % 2 == 0 else K, f0, f1)          # front row
    loft(K if c % 2 == 0 else R, r0, r1)          # rear row (alternated)

# ── elevator (horizontal stab): 4 columns, tapered + rounded LE ─────────────
S_XLE, S_ZC, S_T, S_TEH = -0.585, 0.0225, 0.032, 0.010
S_CROOT, S_CTIP, S_HALF = 0.190, 0.130, 0.335
def s_chord(y):
    return S_CROOT - (S_CROOT - S_CTIP) * (abs(y) / S_HALF)
for c in range(4):
    y0 = -S_HALF + c * (2.0 * S_HALF / 4)
    y1 = y0 + 2.0 * S_HALF / 4
    full0, _, _ = rib_xz(y0, S_XLE, s_chord(y0), S_ZC, S_T, S_TEH)
    full1, _, _ = rib_xz(y1, S_XLE, s_chord(y1), S_ZC, S_T, S_TEH)
    loft(R if c % 2 == 0 else K, full0, full1)

# NOTE: the stab root chord must keep the overall tail at x = -0.775
assert abs((S_XLE - S_CROOT) - (-0.775)) < 1e-9

# ── vertical stabilizer: swept rounded LE, red base / black top ─────────────
F_T, F_TEH = 0.028, 0.010
# Root sits INSIDE the tapered tail boom (boom top is ~0.048 at the tail) so
# the fin never floats above it.
def fin_rib(z):
    f = (z - 0.040) / (0.260 - 0.040)
    chord = 0.190 - (0.190 - 0.120) * f           # TE fixed at -0.775
    return rib_xy(z, -0.775 + chord, chord, 0.0, F_T, F_TEH)
loft(R, fin_rib(0.040), fin_rib(0.165))
loft(K, fin_rib(0.165), fin_rib(0.260))

# ── taildragger gear ────────────────────────────────────────────────────────
for s in (-1, 1):
    y0, y1 = sorted((s * 0.140, s * 0.180))
    box(A, 0.240, 0.280, y0, y1, -0.175, -0.075)
    wheel(K, 0.260, s * 0.200, -0.175, 0.065, 0.018)
box(A, -0.690, -0.670, -0.012, 0.012, -0.115, 0.010)   # reaches the tapered boom
wheel(K, -0.680, 0.0, -0.130, 0.030, 0.010)

# ── AABB + centring + emit ──────────────────────────────────────────────────
xs, ys, zs = (sorted(v[i] for v in verts) for i in range(3))
ext = (xs[-1]-xs[0], ys[-1]-ys[0], zs[-1]-zs[0])
ctr = ((xs[0]+xs[-1])/2.0, (ys[0]+ys[-1])/2.0, (zs[0]+zs[-1])/2.0)
print(f"extents LxSxH = {ext[0]:.3f} x {ext[1]:.3f} x {ext[2]:.3f}  centre = "
      f"({ctr[0]:.4f},{ctr[1]:.4f},{ctr[2]:.4f})")
assert abs(ext[0]-1.550) < 1e-9 and abs(ext[1]-1.860) < 1e-9 \
    and abs(ext[2]-0.500) < 1e-9
verts = [(x-ctr[0], y-ctr[1], z-ctr[2]) for x, y, z in verts]

obj = ["# falcon_trainer — generated RC-plane mesh (HobbyKing Falcon Trainer",
       "# 20cc scheme; regenerate with generate_falcon.py). Body frame:",
       "# +X nose, +Y span, +Z up. Lofted solids: tapered fuselage/wing/stab,",
       "# rounded leading edges.",
       "mtllib falcon_trainer.mtl", ""]
for p in verts:
    obj.append("v %.4f %.4f %.4f" % p)
by_mat, nrm_lines, nidx = {}, [], 0
for mat, base, faces in solids:
    by_mat.setdefault(mat, [])
    for f in faces:
        p0, p1, p2 = (verts[base+f[0]], verts[base+f[1]], verts[base+f[2]])
        u = (p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2])
        v = (p2[0]-p0[0], p2[1]-p0[1], p2[2]-p0[2])
        n = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
        ln = math.sqrt(sum(c*c for c in n)) or 1.0
        nrm_lines.append("vn %.4f %.4f %.4f" % (n[0]/ln, n[1]/ln, n[2]/ln))
        nidx += 1
        by_mat[mat].append(
            "f " + " ".join(f"{base+i+1}//{nidx}" for i in f))
obj.extend(nrm_lines)
for mat, faces in by_mat.items():
    obj.append(f"usemtl {mat}")
    obj.extend(faces)
obj.append("")

dst = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes") + os.sep
open(dst + "falcon_trainer.obj", "w").write("\n".join(obj))
open(dst + "falcon_trainer.mtl", "w").write("""# falcon_trainer materials
newmtl red
Ka 0.35 0.03 0.03
Kd 0.78 0.07 0.06
Ks 0.30 0.30 0.30
Ns 60
newmtl black
Ka 0.03 0.03 0.03
Kd 0.07 0.07 0.07
Ks 0.20 0.20 0.20
Ns 40
newmtl white
Ka 0.45 0.45 0.45
Kd 0.92 0.92 0.92
Ks 0.30 0.30 0.30
Ns 60
newmtl alu
Ka 0.30 0.30 0.32
Kd 0.62 0.63 0.66
Ks 0.50 0.50 0.50
Ns 90
""")
print(f"wrote OBJ: {len(verts)} verts, "
      f"{sum(len(f) for f in by_mat.values())} faces")
