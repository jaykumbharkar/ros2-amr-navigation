#!/usr/bin/env python3
"""Generate worlds/hospital.world: a single-floor hospital for Gazebo Classic.

Run from anywhere:
    python3 src/skid_steer_amr/scripts/generate_hospital_world.py

Layout (top view, metres, building 32 x 20 m, origin at the centre):

  y=10 +---------+------+------+------+------+------+
       | elevator| WC   | N1   | N2   | N3   | N4   | N5      <- patient rooms
       |  lobby  +------+------+------+------+------+------+
  y=3  +---  ---+       |          north corridor          |
       |         |   +--+----------+----------+--------+   |
       |  main   |  ward | med room | nurse    | staff  |   |
       |  lobby  |  doors|          | station  | lounge |   |
       |         |   +--+----------+----------+--------+   |
       |reception|       |          south corridor          |
       |         +------+------+------+------+------+------+
       | waiting | S1 storage  | S2   | S3   | S4 two-bed  |  <- rooms
  y=-10+---[entrance]---+------+------+------+-------------+
     x=-16             x=-6                               x=16

Real-world dimensions: 2.4 m corridors, 1.2 m room doors, 2.0 m ward
double doors, 2.8 m wall height, ~4.4 x 4.8 m single patient rooms.

The 2D LiDAR scans at ~0.22 m above the floor. Objects are modelled so
the LiDAR sees what a real one would: bed bases, cabinet bodies, chair
and table legs, cart bodies. Handrails (0.9 m), mattresses and table
tops are above the scan plane, as in a real hospital.
"""
import math
import os

WALL_H = 2.8        # wall height
DOOR_H = 2.1        # door opening height (a lintel fills the rest)
EXT_T = 0.2         # exterior wall thickness
INT_T = 0.15        # interior wall thickness

# Shapes get a collision only if they reach below this height. The robot is
# 0.25 m tall and its LiDAR scans at 0.22 m, so higher parts (mattresses,
# table tops, monitors, handrails) only need visuals; cameras still see them.
# Fewer collision shapes keeps the simulation at real-time speed.
COLLISION_MAX_Z = 0.4

# ----------------------------------------------------------------------
# Colours (r, g, b, a)
# ----------------------------------------------------------------------
WALL = (0.93, 0.93, 0.90, 1)
WALL_EXT = (0.85, 0.85, 0.83, 1)
WALL_ACCENT = (0.55, 0.75, 0.78, 1)   # teal feature wall
DOOR = (0.62, 0.48, 0.34, 1)          # wooden door leaf
STEEL = (0.70, 0.72, 0.75, 1)
METAL = (0.60, 0.62, 0.65, 1)
DARK = (0.15, 0.15, 0.17, 1)
WHITE = (0.97, 0.97, 0.97, 1)
WOOD = (0.72, 0.56, 0.40, 1)
TEAL = (0.20, 0.50, 0.55, 1)
BLUE = (0.25, 0.40, 0.70, 1)
LIGHT_BLUE = (0.55, 0.70, 0.85, 1)
RED = (0.80, 0.10, 0.10, 1)
GREEN = (0.20, 0.55, 0.25, 1)
TERRACOTTA = (0.70, 0.40, 0.25, 1)
CURTAIN = (0.62, 0.80, 0.74, 1)
GLASS = (0.70, 0.85, 0.95, 0.35)
WINDOW = (0.65, 0.82, 0.95, 0.8)

FLOOR_LOBBY = (0.85, 0.80, 0.70, 1)      # light stone
FLOOR_CORRIDOR = (0.60, 0.68, 0.72, 1)   # blue-grey vinyl
FLOOR_ROOM = (0.74, 0.62, 0.48, 1)       # wood-look vinyl
FLOOR_CORE = (0.66, 0.66, 0.68, 1)
FLOOR_MAT = (0.25, 0.25, 0.28, 1)

PAVING = (0.62, 0.62, 0.60, 1)
ASPHALT = (0.25, 0.25, 0.27, 1)
LINE = (0.95, 0.95, 0.95, 1)


def fmt(*v):
    return ' '.join(f'{x:.4f}'.rstrip('0').rstrip('.') if isinstance(x, float) else str(x)
                    for x in v)


def material(c):
    r, g, b, a = c
    return (f'<material><ambient>{fmt(r, g, b, a)}</ambient>'
            f'<diffuse>{fmt(r, g, b, a)}</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular></material>')


class Model:
    """A static model: one link holding many box/cylinder shapes.

    Shape positions are in the model frame; z is the BOTTOM of the shape.
    """

    def __init__(self, name, x=0.0, y=0.0, yaw=0.0):
        self.name, self.x, self.y, self.yaw = name, x, y, yaw
        self.shapes = []

    def _add(self, geom, x, y, zc, rpy, color, collide, bottom):
        collide = collide and bottom < COLLISION_MAX_Z
        self.shapes.append((geom, (x, y, zc) + rpy, color, collide))

    def box(self, x, y, z, sx, sy, sz, color, yaw=0.0, collide=True):
        self._add(f'<box><size>{fmt(sx, sy, sz)}</size></box>',
                  x, y, z + sz / 2, (0, 0, yaw), color, collide, z)

    def cyl(self, x, y, z, r, h, color, axis='z', collide=True):
        """Vertical cylinder: z is its bottom. Horizontal (axis x/y): z is its centre."""
        rpy = {'z': (0, 0, 0), 'x': (0, math.pi / 2, 0), 'y': (math.pi / 2, 0, 0)}[axis]
        zc, bottom = (z + h / 2, z) if axis == 'z' else (z, z - r)
        self._add(f'<cylinder><radius>{fmt(r)}</radius><length>{fmt(h)}</length></cylinder>',
                  x, y, zc, rpy, color, collide, bottom)

    def sphere(self, x, y, zc, r, color, collide=False):
        self._add(f'<sphere><radius>{fmt(r)}</radius></sphere>',
                  x, y, zc, (0, 0, 0), color, collide, zc - r)

    def sdf(self):
        out = [f'    <model name="{self.name}">',
               '      <static>true</static>',
               f'      <pose>{fmt(self.x, self.y, 0, 0, 0, self.yaw)}</pose>',
               '      <link name="link">']
        for i, (geom, pose, color, collide) in enumerate(self.shapes):
            p = fmt(*pose)
            if collide:
                out.append(f'        <collision name="c{i}"><pose>{p}</pose>'
                           f'<geometry>{geom}</geometry></collision>')
            out.append(f'        <visual name="v{i}"><pose>{p}</pose>'
                       f'<geometry>{geom}</geometry>{material(color)}</visual>')
        out += ['      </link>', '    </model>']
        return '\n'.join(out)


MODELS = []


def model(name, x=0.0, y=0.0, yaw=0.0):
    m = Model(name, x, y, yaw)
    MODELS.append(m)
    return m


# ----------------------------------------------------------------------
# Walls
# ----------------------------------------------------------------------
def _spans(a, b, openings):
    """Split [a, b] into solid spans around the opening intervals."""
    spans, cur = [], a
    for o0, o1 in sorted(openings):
        if o0 > cur:
            spans.append((cur, o0))
        cur = max(cur, o1)
    if cur < b:
        spans.append((cur, b))
    return spans


def wall(m, start, end, fixed, along, t=INT_T, openings=(), color=WALL,
         lintel=True, rails=0):
    """Axis-aligned wall from start to end along 'x' or 'y' at coordinate 'fixed'.

    openings: (from, to) intervals along the wall for doors.
    rails:    +1/-1 adds a 0.9 m handrail on the +/- side, 2 on both sides.
    """
    for s0, s1 in _spans(start, end, openings):
        length, mid = s1 - s0, (s0 + s1) / 2
        if along == 'x':
            m.box(mid, fixed, 0, length, t, WALL_H, color)
        else:
            m.box(fixed, mid, 0, t, length, WALL_H, color)
        sides = (1, -1) if rails == 2 else ((rails,) if rails else ())
        for side in (sides if length > 0.3 else ()):
            off = fixed + side * (t / 2 + 0.06)
            if along == 'x':
                m.cyl(mid, off, 0.9, 0.025, length - 0.1, STEEL, axis='x')
            else:
                m.cyl(off, mid, 0.9, 0.025, length - 0.1, STEEL, axis='y')
    if lintel:
        for o0, o1 in openings:
            mid, length = (o0 + o1) / 2, o1 - o0
            if along == 'x':
                m.box(mid, fixed, DOOR_H, length, t, WALL_H - DOOR_H, color)
            else:
                m.box(fixed, mid, DOOR_H, t, length, WALL_H - DOOR_H, color)


def window(m, along, fixed, a, b, face):
    """Visual-only window pane on a wall face (face = +1/-1 side)."""
    off = fixed + face * (EXT_T / 2 + 0.005)
    if along == 'x':
        m.box((a + b) / 2, off, 0.9, b - a, 0.01, 1.3, WINDOW, collide=False)
    else:
        m.box(off, (a + b) / 2, 0.9, 0.01, b - a, 1.3, WINDOW, collide=False)


# ----------------------------------------------------------------------
# Furniture (local frame: +y is the "back"/head end unless noted)
# ----------------------------------------------------------------------
def hospital_bed(name, x, y, yaw):
    m = model(name, x, y, yaw)
    for cx in (-0.38, 0.38):
        for cy in (-0.85, 0.85):
            m.cyl(cx, cy, 0, 0.05, 0.15, DARK)                 # casters
    m.box(0, 0, 0.15, 0.9, 1.9, 0.30, METAL)                    # base (LiDAR sees this)
    m.box(0, 0, 0.45, 0.9, 2.0, 0.17, (0.88, 0.92, 0.96, 1))   # mattress
    m.box(0, 1.05, 0.30, 1.0, 0.06, 0.80, (0.75, 0.78, 0.80, 1))   # headboard
    m.box(0, -1.05, 0.30, 1.0, 0.06, 0.60, (0.75, 0.78, 0.80, 1))  # footboard
    m.box(0, 0.72, 0.62, 0.6, 0.35, 0.12, WHITE)                # pillow
    m.box(0, -0.35, 0.62, 0.92, 1.1, 0.03, LIGHT_BLUE)          # blanket
    for cx in (-0.47, 0.47):
        m.box(cx, 0.2, 0.62, 0.03, 0.8, 0.22, STEEL)            # side rails


def visitor_chair(name, x, y, yaw):
    """Facing -y."""
    m = model(name, x, y, yaw)
    for cx in (-0.23, 0.23):
        for cy in (-0.23, 0.23):
            m.cyl(cx, cy, 0, 0.02, 0.40, METAL)
    m.box(0, 0, 0.40, 0.55, 0.55, 0.08, TEAL)                   # seat
    m.box(0, 0.25, 0.48, 0.55, 0.06, 0.50, TEAL)                # backrest
    for cx in (-0.27, 0.27):
        m.box(cx, 0, 0.48, 0.05, 0.5, 0.20, METAL)              # armrests


def bedside_cabinet(name, x, y, yaw=0.0):
    m = model(name, x, y, yaw)
    m.box(0, 0, 0, 0.45, 0.45, 0.80, WOOD)
    m.box(0, -0.226, 0.45, 0.35, 0.005, 0.12, (0.6, 0.45, 0.32, 1), collide=False)  # drawer


def iv_stand(name, x, y):
    m = model(name, x, y)
    m.cyl(0, 0, 0, 0.28, 0.03, METAL)                           # base
    m.cyl(0, 0, 0.03, 0.015, 1.9, STEEL)                        # pole
    m.box(0, 0, 1.9, 0.4, 0.02, 0.02, STEEL)                    # hook bar
    m.box(0.12, 0, 1.55, 0.10, 0.03, 0.18, (0.85, 0.92, 0.95, 0.7), collide=False)  # IV bag


def overbed_table(name, x, y, yaw):
    """Column at +x, top reaches over the bed towards -x."""
    m = model(name, x, y, yaw)
    m.box(0.15, 0, 0, 0.55, 0.40, 0.05, METAL)                  # base plate
    m.box(0.35, 0, 0.05, 0.05, 0.05, 0.80, STEEL)               # column
    m.box(-0.05, 0, 0.85, 0.85, 0.40, 0.03, WOOD)               # top


def curtain(name, x, y, along, length):
    m = model(name, x, y)
    sx, sy = (length, 0.06) if along == 'x' else (0.06, length)
    m.box(0, 0, 0.35, sx, sy, 1.85, CURTAIN)
    m.box(0, 0, 2.25, sx, sy, 0.03, STEEL, collide=False)       # ceiling track


def wardrobe(name, x, y, yaw=0.0):
    m = model(name, x, y, yaw)
    m.box(0, 0, 0, 0.6, 0.6, 2.0, WOOD)
    m.box(0, -0.301, 0.9, 0.02, 0.005, 0.25, METAL, collide=False)   # handle


def shelving(name, x, y, yaw, length=1.2):
    """Open metal shelving with supplies, back at +y."""
    m = model(name, x, y, yaw)
    for cx in (-length / 2 + 0.02, length / 2 - 0.02):
        for cy in (-0.2, 0.2):
            m.box(cx, cy, 0, 0.04, 0.04, 1.9, STEEL)            # uprights
    for z in (0.15, 0.6, 1.05, 1.5, 1.88):
        m.box(0, 0, z, length, 0.45, 0.02, STEEL)               # shelves
    m.box(0, 0.05, 0.17, length - 0.15, 0.3, 0.30, (0.85, 0.8, 0.7, 1))  # boxes on bottom shelf
    for i, z in enumerate((0.62, 1.07, 1.52)):
        col = (LIGHT_BLUE, WHITE, (0.9, 0.85, 0.6, 1))[i]
        m.box(0, 0.05, z, length - 0.2, 0.3, 0.25, col, collide=False)


def cart(name, x, y, yaw, color, size=(0.8, 0.5, 1.0)):
    sx, sy, sz = size
    m = model(name, x, y, yaw)
    for cx in (-sx / 2 + 0.06, sx / 2 - 0.06):
        for cy in (-sy / 2 + 0.06, sy / 2 - 0.06):
            m.cyl(cx, cy, 0, 0.04, 0.12, DARK)
    m.box(0, 0, 0.12, sx, sy, sz - 0.12, color)
    m.box(0, 0, sz, sx + 0.04, sy + 0.04, 0.03, METAL)
    for i in range(3):                                          # drawer lines
        m.box(0, -sy / 2 - 0.003, 0.3 + i * 0.22, sx - 0.1, 0.005, 0.01, DARK, collide=False)


def wheelchair(name, x, y, yaw):
    """Facing -y."""
    m = model(name, x, y, yaw)
    for cx in (-0.28, 0.28):
        m.cyl(cx, 0.1, 0.3, 0.30, 0.03, DARK, axis='x')         # big wheels
        m.cyl(cx * 0.8, -0.3, 0.0, 0.05, 0.10, DARK)            # front casters
    m.box(0, 0, 0.45, 0.48, 0.45, 0.06, BLUE)                   # seat
    m.box(0, 0.22, 0.51, 0.48, 0.04, 0.45, BLUE)                # back
    m.box(0, 0.30, 0.90, 0.48, 0.03, 0.03, METAL)               # push handle bar
    for cx in (-0.25, 0.25):
        m.box(cx, 0.0, 0.08, 0.03, 0.45, 0.37, METAL)           # side frame (LiDAR sees)
    m.box(0, -0.38, 0.08, 0.40, 0.15, 0.02, METAL)              # footrest


def plant(name, x, y):
    m = model(name, x, y)
    m.cyl(0, 0, 0, 0.25, 0.45, TERRACOTTA)
    m.sphere(0, 0, 0.95, 0.42, GREEN)
    m.sphere(0.15, 0.1, 1.3, 0.25, GREEN)


def beam_seating(name, x, y, yaw, seats=5):
    """Row of connected waiting-room seats, facing -y."""
    m = model(name, x, y, yaw)
    w = 0.55 * seats
    m.box(0, 0, 0.30, w, 0.08, 0.06, METAL)                     # beam
    for cx in (-w / 2 + 0.1, 0.0, w / 2 - 0.1):
        m.box(cx, 0, 0, 0.06, 0.45, 0.30, METAL)                # legs (LiDAR sees)
    for i in range(seats):
        cx = -w / 2 + 0.275 + i * 0.55
        m.box(cx, -0.02, 0.40, 0.50, 0.48, 0.07, TEAL)          # seat
        m.box(cx, 0.22, 0.47, 0.50, 0.05, 0.45, TEAL)           # back


def desk(name, x, y, yaw, w=1.4, d=0.7, monitor=True):
    """Desk with modesty panel at +y, user sits at -y."""
    m = model(name, x, y, yaw)
    m.box(0, 0, 0.72, w, d, 0.03, WOOD)                         # top
    for cx in (-w / 2 + 0.03, w / 2 - 0.03):
        m.box(cx, 0, 0, 0.04, d - 0.04, 0.72, WOOD)             # side panels
    m.box(0, d / 2 - 0.03, 0.15, w - 0.08, 0.02, 0.57, WOOD)    # modesty panel
    if monitor:
        m.box(0, 0.15, 0.75, 0.2, 0.15, 0.02, DARK)             # stand
        m.box(0, 0.17, 0.77, 0.04, 0.04, 0.12, DARK)
        m.box(0, 0.15, 0.85, 0.55, 0.03, 0.33, DARK)            # screen


def office_chair(name, x, y):
    m = model(name, x, y)
    m.cyl(0, 0, 0, 0.30, 0.05, DARK)                            # star base
    m.cyl(0, 0, 0.05, 0.03, 0.40, METAL)                        # gas lift
    m.cyl(0, 0, 0.45, 0.25, 0.08, DARK)                         # seat
    m.box(0, 0.22, 0.55, 0.42, 0.05, 0.50, DARK)                # back


def dining_table(name, x, y, w=1.6, d=0.9):
    m = model(name, x, y)
    m.box(0, 0, 0.72, w, d, 0.04, WHITE)
    for cx in (-w / 2 + 0.06, w / 2 - 0.06):
        for cy in (-d / 2 + 0.06, d / 2 - 0.06):
            m.box(cx, cy, 0, 0.05, 0.05, 0.72, METAL)


def sofa(name, x, y, yaw, w=1.8):
    """Facing -y."""
    m = model(name, x, y, yaw)
    m.box(0, 0, 0.0, w, 0.85, 0.45, TEAL)                       # base + seat
    m.box(0, 0.35, 0.45, w, 0.15, 0.40, TEAL)                   # back
    for cx in (-w / 2 + 0.08, w / 2 - 0.08):
        m.box(cx, 0, 0.45, 0.16, 0.85, 0.18, TEAL)              # arms


def counter(name, x, y, yaw, w, d=0.6, h=0.95, color=WHITE):
    m = model(name, x, y, yaw)
    m.box(0, 0, 0, w, d, h, color)
    m.box(0, 0, h, w + 0.02, d + 0.02, 0.04, (0.35, 0.37, 0.40, 1))  # worktop
    return m


def tall_box(name, x, y, yaw, size, color, front=None):
    """Fridge, vending machine, kiosk... optional front panel colour at -y."""
    sx, sy, sz = size
    m = model(name, x, y, yaw)
    m.box(0, 0, 0, sx, sy, sz, color)
    if front:
        m.box(0, -sy / 2 - 0.003, 0.5, sx * 0.8, 0.005, sz - 0.7, front, collide=False)


def wall_panel(name, x, y, yaw, w, h, z, color, depth=0.04, collide=True):
    """Thin panel flat against a wall (doors, elevator doors, TV, signs). Faces -y."""
    m = model(name, x, y, yaw)
    m.box(0, 0, z, w, depth, h, color, collide=collide)
    return m


def open_door(name, hinge_x, hinge_y, swing_dir_x, swing_dir_y, width=1.1):
    """Door leaf swung 90 deg open: lies from the hinge along (dir_x, dir_y)."""
    m = model(name, hinge_x + swing_dir_x * width / 2, hinge_y + swing_dir_y * width / 2)
    sx, sy = (width, 0.045) if swing_dir_x else (0.045, width)
    m.box(0, 0, 0.0, sx, sy, DOOR_H - 0.02, DOOR)


# ----------------------------------------------------------------------
# Building
# ----------------------------------------------------------------------
def floors():
    """Visual-only floor finishes (no collision: physics uses the ground plane).

    Stacked a few millimetres apart so overlapping areas do not flicker.
    """
    f = model('floor_finishes')

    def area(x0, x1, y0, y1, c, layer=1):
        z = 0.002 * layer
        f.box((x0 + x1) / 2, (y0 + y1) / 2, z, x1 - x0, y1 - y0, 0.001, c, collide=False)

    # Inside
    area(-16, -6, -10, 3, FLOOR_LOBBY)
    area(-16, -6, 3, 10, FLOOR_CORE)
    area(-6, 16, -5.2, 5.2, FLOOR_CORRIDOR)
    area(-6, 16, 5.2, 10, FLOOR_ROOM)
    area(-6, 16, -10, -5.2, FLOOR_ROOM)
    area(-3, 13, -2.8, 2.8, FLOOR_CORE, layer=2)
    area(-12.5, -9.5, -9.8, -8.2, FLOOR_MAT, layer=2)               # entrance mat

    # Outside: forecourt, walkway, car park with bay markings
    area(-17, 17, -12, -10, PAVING)                                  # pavement along facade
    area(-12.5, -9.5, -18, -12, PAVING)                              # walkway to entrance
    area(-4, 16, -22, -13, ASPHALT)                                  # car park
    for i in range(9):
        x = -3.5 + i * 2.5
        area(x - 0.05, x + 0.05, -16.0, -13.2, LINE, layer=2)        # bays, row 1
        area(x - 0.05, x + 0.05, -21.8, -19.0, LINE, layer=2)        # bays, row 2
    area(-30, 30, -26, -22, ASPHALT)                                 # road
    for i in range(12):
        area(-28 + i * 5, -26 + i * 5, -24.06, -23.94, LINE, layer=2)   # centre line


def building():
    b = model('hospital_walls')

    # Exterior shell. Entrance gap x -12.5..-9.5 is closed by glass doors.
    wall(b, -16, 16, -10, 'x', t=EXT_T, openings=[(-12.5, -9.5)], color=WALL_EXT, lintel=True)
    wall(b, -16, 16, 10, 'x', t=EXT_T, color=WALL_EXT)
    wall(b, -10, 10, -16, 'y', t=EXT_T, color=WALL_EXT)
    wall(b, -10, 10, 16, 'y', t=EXT_T, color=WALL_EXT)

    # West zone: lobby / elevator lobby partition, restroom block
    wall(b, -16, -6, 3, 'x', openings=[(-10.0, -7.0)], lintel=False)
    wall(b, 6, 10, -9.5, 'y')
    wall(b, -9.5, -6, 6, 'x')

    # Ward boundary with 2 m double doors into the west cross-corridor
    wall(b, -10, 10, -6, 'y', openings=[(-1.0, 1.0)], color=WALL_ACCENT)

    # Patient-room bands: corridor walls with doors + handrails on corridor side
    north_x = [-6, -1.6, 2.8, 7.2, 11.6, 16]
    south_x = [-6, -1.6, 2.8, 7.2, 16]
    north_doors = [(x1 - 1.7, x1 - 0.5) for x1 in north_x[1:]]
    south_doors = [(x1 - 1.7, x1 - 0.5) for x1 in south_x[1:-1]] + [(10.8, 12.0)]
    wall(b, -6, 16, 5.2, 'x', openings=north_doors, rails=-1)
    wall(b, -6, 16, -5.2, 'x', openings=south_doors, rails=1)
    for x in north_x[1:-1]:
        wall(b, 5.2, 10, x, 'y')
    for x in south_x[1:-1]:
        wall(b, -10, -5.2, x, 'y')

    # Central core: med room | nurse station | staff lounge
    wall(b, -3, 13, 2.8, 'x', openings=[(1.0, 7.0)], rails=1)     # nurse counter fills 1..6
    wall(b, -3, 13, -2.8, 'x', openings=[(-1.5, -0.3), (10.0, 11.2)], rails=-1)
    wall(b, -2.8, 2.8, -3, 'y', rails=-1)
    wall(b, -2.8, 2.8, 13, 'y', rails=1)
    wall(b, -2.8, 2.8, 1, 'y', openings=[(-1.6, -0.5)])
    wall(b, -2.8, 2.8, 7, 'y', openings=[(0.4, 1.5)])

    # Windows on exterior walls
    for x0, x1 in zip(north_x[:-1], north_x[1:]):
        window(b, 'x', 10, x0 + 1.0, x1 - 1.0, -1)
    for x0, x1 in zip(south_x[:-1], south_x[1:]):
        window(b, 'x', -10, x0 + 1.0, x1 - 1.0, 1)
    window(b, 'x', -10, -15.5, -13.0, 1)
    window(b, 'x', -10, -9.0, -6.5, 1)
    window(b, 'y', -16, -4.5, 1.0, 1)

    # Glass entrance doors (closed; LiDAR sees them like a wall)
    g = model('entrance_glass_doors')
    g.box(-11.0, -10.0, 0, 3.0, 0.06, DOOR_H, GLASS)
    g.box(-11.0, -10.0, 0, 0.04, 0.08, DOOR_H, METAL, collide=False)
    return north_x, south_x


def patient_room(tag, x0, x1, side, beds=1, door=None):
    """side = +1 north band (outer wall y=10), -1 south band (outer wall y=-10).

    door: (from, to) x-interval of the room door in the corridor wall.
    """
    s = side
    yc = s * 5.2                       # corridor wall
    yo = s * 10.0                      # outer wall
    yaw = 0.0 if s > 0 else math.pi    # local +y points to the outer wall

    # En-suite bathroom pod next to the corridor at the x0 side
    bw, bd = 1.9, 2.2
    pod = model(f'bathroom_{tag}')
    wall(pod, min(yc, yc + s * bd), max(yc, yc + s * bd), x0 + bw, 'y', t=0.1)
    wall(pod, x0, x0 + bw, yc + s * bd, 'x', t=0.1)
    wall_panel(f'bathroom_door_{tag}', x0 + bw + 0.06, yc + s * 1.2, math.pi / 2,
               0.85, 2.0, 0.0, DOOR, depth=0.02, collide=False)

    # Room door leaf, swung open into the room along the wall
    door = door or (x1 - 1.7, x1 - 0.5)
    open_door(f'door_{tag}', door[1], yc + s * 0.08, 0, s, 1.1)
    wardrobe(f'wardrobe_{tag}', x0 + 2.3, yc + s * 0.4, yaw + math.pi)

    bed_xs = [x0 + 2.5] if beds == 1 else [x0 + 2.3, x0 + 6.3]
    for i, bx in enumerate(bed_xs):
        n = f'{tag}{"" if beds == 1 else chr(ord("a") + i)}'
        by = yo - s * (0.1 + 1.1)
        hospital_bed(f'bed_{n}', bx, by, yaw)
        bedside_cabinet(f'bedside_cabinet_{n}', bx - 0.85, yo - s * 0.4, yaw)
        iv_stand(f'iv_stand_{n}', bx - 0.8, yo - s * 1.45)
        visitor_chair(f'visitor_chair_{n}', bx - 1.45, yo - s * 2.1, math.pi / 2)  # faces the bed
        overbed_table(f'overbed_table_{n}', bx + 0.75, yo - s * 1.5, 0.0)
        curtain(f'curtain_{n}', bx + 1.4, yo - s * 0.7, 'y', 1.2)
    if beds == 2:
        curtain(f'curtain_{tag}_divider', x0 + 4.3, yo - s * 1.4, 'y', 2.6)


def furnish(north_x, south_x):
    # ---------- Patient rooms ----------
    for i, (x0, x1) in enumerate(zip(north_x[:-1], north_x[1:]), start=1):
        patient_room(f'N{i}', x0, x1, +1)
    patient_room('S2', -1.6, 2.8, -1)
    patient_room('S3', 2.8, 7.2, -1)
    patient_room('S4', 7.2, 16.0, -1, beds=2, door=(10.8, 12.0))
    open_door('door_S1', -2.1, -5.28, 0, -1, 1.1)

    # ---------- S1: storage / clean utility ----------
    for i, x in enumerate((-5.2, -3.8, -2.4)):
        shelving(f'storage_shelf_back_{i}', x, -9.55, math.pi)
    for i, y in enumerate((-8.3, -7.0)):
        shelving(f'storage_shelf_side_{i}', -5.55, y, math.pi / 2)
    cart('supply_cart_S1', -3.2, -7.2, 0.3, LIGHT_BLUE)

    # ---------- Main lobby ----------
    counter('reception_front', -10.0, -3.3, 0.0, 3.0, d=0.6, h=1.1, color=WOOD)
    counter('reception_left', -11.65, -2.65, 0.0, 0.3, d=1.9, h=1.1, color=WOOD)
    counter('reception_right', -8.35, -2.65, 0.0, 0.3, d=1.9, h=1.1, color=WOOD)
    desk('reception_desk_1', -10.7, -2.6, math.pi)
    desk('reception_desk_2', -9.3, -2.6, math.pi)
    office_chair('reception_chair_1', -10.7, -1.9)
    office_chair('reception_chair_2', -9.3, -1.9)
    wall_panel('lobby_sign', -10.0, 2.88, 0.0, 2.5, 0.6, 2.0, WALL_ACCENT, collide=False)

    for i, y in enumerate((-8.4, -6.9, -5.4)):
        beam_seating(f'waiting_seats_{i}', -13.8, y, 0.0, seats=5)
    dining_table('waiting_coffee_table', -13.8, -4.3, w=1.2, d=0.5)
    wall_panel('waiting_tv', -15.88, -6.9, -math.pi / 2, 1.2, 0.7, 1.5, DARK, collide=False)
    tall_box('info_kiosk', -7.4, -8.6, 0.0, (0.5, 0.4, 1.3), STEEL, front=DARK)
    tall_box('water_dispenser', -15.6, -2.0, math.pi / 2, (0.35, 0.35, 1.2), WHITE, front=LIGHT_BLUE)
    for i, (x, y) in enumerate(((-15.4, -9.4), (-6.6, -9.4), (-15.4, 2.4), (-6.6, 2.4))):
        plant(f'lobby_plant_{i}', x, y)
    wheelchair('wheelchair_lobby_1', -7.0, -6.5, -math.pi / 2)
    wheelchair('wheelchair_lobby_2', -7.0, -5.7, -math.pi / 2)

    # ---------- Elevator lobby + restrooms ----------
    for i, y in enumerate((4.6, 7.6)):
        wall_panel(f'elevator_door_{i}', -15.88, y, -math.pi / 2, 1.2, 2.2, 0.0, STEEL)
        wall_panel(f'elevator_frame_{i}', -15.895, y, -math.pi / 2, 1.5, 2.45, 0.0, METAL,
                   depth=0.01, collide=False)
        wall_panel(f'elevator_button_{i}', -15.88, y + 0.95, -math.pi / 2, 0.1, 0.18, 1.1,
                   DARK, collide=False)
    for i, y in enumerate((7.0, 9.0)):
        wall_panel(f'restroom_door_{i}', -9.6, y, -math.pi / 2, 0.9, 2.0, 0.0, DOOR,
                   depth=0.02, collide=False)
    tall_box('vending_machine', -12.0, 9.5, 0.0, (0.9, 0.8, 1.8), RED, front=DARK)
    beam_seating('elevator_bench', -12.5, 3.55, math.pi, seats=3)
    plant('elevator_plant', -10.0, 9.4)

    # ---------- Ward entrance double doors (held open) ----------
    open_door('ward_door_left', -5.92, 1.0, 1, 0, 0.95)
    open_door('ward_door_right', -5.92, -1.0, 1, 0, 0.95)

    # ---------- Nurse station ----------
    counter('nurse_counter', 3.5, 2.8, 0.0, 5.0, d=0.5, h=1.1)
    desk('nurse_desk_1', 2.3, 1.8, 0.0)
    desk('nurse_desk_2', 4.7, 1.8, 0.0)
    office_chair('nurse_chair_1', 2.3, 1.1)
    office_chair('nurse_chair_2', 4.7, 1.1)
    shelving('nurse_files', 4.0, -2.45, math.pi, length=1.8)
    tall_box('nurse_printer_table', 6.4, -2.4, 0.0, (0.6, 0.6, 0.75), WOOD)

    # ---------- Medication room ----------
    counter('med_counter', -2.0, 2.4, math.pi, 1.8)
    tall_box('med_fridge', -2.55, -0.6, math.pi / 2, (0.6, 0.6, 1.8), WHITE, front=STEEL)
    shelving('med_shelf_1', -0.4, 2.5, 0.0, length=1.0)
    tall_box('med_dispensing_cabinet', 0.5, 0.6, -math.pi / 2, (0.8, 0.5, 1.6), LIGHT_BLUE, front=DARK)

    # ---------- Staff lounge ----------
    dining_table('lounge_table', 9.5, 0.3)
    for i, (x, y, yaw) in enumerate(((8.9, -0.55, math.pi), (10.1, -0.55, math.pi),
                                     (8.9, 1.15, 0.0), (10.1, 1.15, 0.0))):
        visitor_chair(f'lounge_chair_{i}', x, y, yaw)
    sofa('lounge_sofa', 9.5, 2.25, 0.0)
    counter('lounge_kitchenette', 12.6, 0.0, math.pi / 2, 2.4)
    tall_box('lounge_fridge', 12.6, 1.8, -math.pi / 2, (0.6, 0.6, 1.8), WHITE, front=STEEL)

    # ---------- Corridor equipment (parked along walls, >1.6 m clear width) ----------
    wheelchair('wheelchair_corridor', 8.3, 4.6, 0.0)
    cart('medication_cart', -0.5, 3.25, math.pi, BLUE)
    cart('linen_cart', 8.6, -4.65, math.pi, (0.85, 0.85, 0.80, 1), size=(1.0, 0.55, 1.1))
    cart('crash_cart', 15.55, 2.5, -math.pi / 2, RED, size=(0.7, 0.5, 1.0))
    tall_box('corridor_supply_cabinet', 15.6, -2.5, -math.pi / 2, (1.0, 0.5, 1.9), STEEL)
    wall_panel('emergency_exit_door', 15.88, 0.0, math.pi / 2, 1.2, 2.1, 0.0,
               (0.75, 0.2, 0.2, 1), depth=0.02, collide=False)
    wall_panel('exit_sign', 15.88, 0.0, math.pi / 2, 0.4, 0.15, 2.25,
               (0.1, 0.7, 0.2, 1), depth=0.02, collide=False)
    for i, (x, y, yaw) in enumerate(((-0.5, 5.12, 0.0), (9.0, 5.12, 0.0),
                                     (4.0, -5.12, math.pi), (-3.12, 0.0, math.pi / 2))):
        wall_panel(f'hand_sanitizer_{i}', x, y, yaw, 0.12, 0.25, 1.2, WHITE,
                   depth=0.08, collide=False)
    plant('corridor_plant_east', 15.4, 4.7)


# ----------------------------------------------------------------------
# People: animated Gazebo actors. Cameras see them; the 2D LiDAR does NOT,
# because actors in Gazebo Classic have no collision shape.
# ----------------------------------------------------------------------
ACTORS = []


def _actor(name, dae, waypoints):
    """SDF for one actor. waypoints: [(time_s, x, y, yaw), ...]"""
    wps = '\n'.join(f'          <waypoint><time>{t:.2f}</time>'
                    f'<pose>{fmt(x, y, 0.0, 0.0, 0.0, yaw)}</pose></waypoint>'
                    for t, x, y, yaw in waypoints)
    interp = '<interpolate_x>true</interpolate_x>' if dae == 'walk.dae' else ''
    return (f'    <actor name="{name}">\n'
            f'      <skin><filename>{dae}</filename></skin>\n'
            f'      <animation name="anim"><filename>{dae}</filename>{interp}</animation>\n'
            f'      <script><loop>true</loop><auto_start>true</auto_start>\n'
            f'        <trajectory id="0" type="anim">\n{wps}\n'
            f'        </trajectory>\n      </script>\n    </actor>')


def standing_person(name, x, y, yaw, dae='stand.dae'):
    """A person who stays in one place. dae: stand.dae, talk_a.dae or talk_b.dae"""
    ACTORS.append(_actor(name, dae, [(0.0, x, y, yaw), (100.0, x, y, yaw)]))


def walking_person(name, points, speed=0.8):
    """A person walking along points [(x, y), ...] and back again, forever."""
    path = points + points[-2::-1]                 # there and back
    t, wps = 0.0, []
    for (x0, y0), (x1, y1) in zip(path[:-1], path[1:]):
        yaw = math.atan2(y1 - y0, x1 - x0)
        if wps:
            t += 1.0                               # 1 s to turn towards the next point
        wps.append((t, x0, y0, yaw))
        t += math.hypot(x1 - x0, y1 - y0) / speed  # walking time to the next point
        wps.append((t, x1, y1, yaw))
    ACTORS.append(_actor(name, 'walk.dae', wps))


def people():
    standing_person('visitor_reception', -10.0, -4.3, math.pi / 2)              # at the reception desk
    standing_person('visitor_ward_doors', -4.4, -0.4, math.pi, 'talk_a.dae')   # just inside the ward doors
    standing_person('nurse_at_station', 3.5, 3.5, -math.pi / 2, 'talk_b.dae')  # at the nurse counter
    walking_person('nurse_walking', [(-4.5, 4.4), (7.5, 4.4)])                 # north corridor
    walking_person('porter_walking', [(14.5, -3.9), (-4.5, -3.9)], speed=0.6)  # south corridor



HEADER = '''<?xml version="1.0" ?>
<!-- GENERATED by scripts/generate_hospital_world.py: edit the script, not this file. -->
<sdf version="1.6">
  <world name="hospital">

    <include><uri>model://sun</uri></include>

    <!-- Same collision and friction as the stock ground_plane, grass colour -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface>
            <contact><collide_bitmask>0xffff</collide_bitmask></contact>
            <friction><ode><mu>100</mu><mu2>50</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.35 0.55 0.3 1</ambient><diffuse>0.35 0.55 0.3 1</diffuse>
            <specular>0 0 0 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.85 0.95 1</background>
      <shadows>false</shadows>
    </scene>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <sor>1.0</sor>
        </solver>
        <constraints>
          <cfm>0.0001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <gui>
      <camera name="user_camera">
        <pose>0 -27 26 0 0.78 1.5708</pose>
      </camera>
    </gui>
'''


def main():
    floors()
    north_x, south_x = building()
    furnish(north_x, south_x)
    people()
    body = '\n\n'.join([m.sdf() for m in MODELS] + ACTORS)
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'worlds', 'hospital.world')
    with open(out, 'w') as f:
        f.write(HEADER + '\n' + body + '\n\n  </world>\n</sdf>\n')
    print(f'wrote {os.path.normpath(out)}: {len(MODELS)} models, '
          f'{sum(len(m.shapes) for m in MODELS)} shapes')


if __name__ == '__main__':
    main()
