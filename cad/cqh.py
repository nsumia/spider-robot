"""Small CadQuery helpers (Shape API, mm)."""
import math
import cadquery as cq

V = cq.Vector


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl(r, h, base=(0, 0, 0), d=(0, 0, 1)):
    return cq.Solid.makeCylinder(r, h, V(*base), V(*d))


def cyl_y(r, y0, y1, x, z):
    """cylinder along +Y from y0 to y1 centred at (x, z)"""
    return cyl(r, y1 - y0, (x, y0, z), (0, 1, 0))


def hexprism_y(af, y0, y1, x, z):
    """hex prism (across flats af) along Y between y0..y1 at (x, z); flats parallel to X"""
    pl = cq.Plane(origin=(x, y0, z), xDir=(1, 0, 0), normal=(0, 1, 0))
    wp = cq.Workplane(pl).polygon(6, af / math.cos(math.radians(30))).extrude(y1 - y0)
    return wp.val()


def prism_xz(pts, y0, y1):
    """polygon given in (x, z) extruded along Y from y0 to y1"""
    wp = cq.Workplane("XZ").polyline(pts).close().extrude(y1 - y0)   # spans y in [-(y1-y0), 0]
    return wp.val().translate(V(0, y1, 0))


def hull_pts(c1, r1, c2, r2):
    """polygon (4 pts) of the outer tangent hull of two circles in a plane"""
    dx, dz = c2[0] - c1[0], c2[1] - c1[1]
    d = math.hypot(dx, dz)
    u = (dx / d, dz / d)
    beta = math.acos(max(-1.0, min(1.0, (r1 - r2) / d)))
    out = []
    for sgn in (+1, -1):
        b = sgn * beta
        ux = u[0] * math.cos(b) - u[1] * math.sin(b)
        uz = u[0] * math.sin(b) + u[1] * math.cos(b)
        out.append(((c1[0] + r1 * ux, c1[1] + r1 * uz), (c2[0] + r2 * ux, c2[1] + r2 * uz)))
    (p1a, p2a), (p1b, p2b) = out
    return [p1a, p2a, p2b, p1b]


def hull_plate(c1, r1, c2, r2, y0, y1):
    s = prism_xz(hull_pts(c1, r1, c2, r2), y0, y1)
    s = s.fuse(cyl_y(r1, y0, y1, *c1)).fuse(cyl_y(r2, y0, y1, *c2))
    return s


def fuse_all(shapes):
    out = shapes[0]
    for s in shapes[1:]:
        out = out.fuse(s)
    return out.clean()


def cut_all(base, cutters):
    out = base
    for c in cutters:
        out = out.cut(c)
    return out.clean()


def rot(shape, axis_pt, axis_dir, deg):
    p = V(*axis_pt)
    return shape.rotate(p, p + V(*axis_dir), deg)


def single_solid(shape):
    sols = shape.Solids()
    if len(sols) == 1:
        return sols[0]
    return cq.Compound.makeCompound(sols)
