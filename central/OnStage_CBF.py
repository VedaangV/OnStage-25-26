import math
from OnStage_WifiComms import wifi_write

# ---------------------------------------------------------------------------
# Tunable constants
# ---------------------------------------------------------------------------

ROBOT_RADIUS     = 3.46   # 10*ft
DIST_THRESHOLD   = 6      # arrival threshold (field units)
ENABLE_WIFI      = True  # sync with OnStage_Master.py
baseV            = 7      # max speed (10*ft/s)

PLOT_TYPE = "RADIUS_PLOT" # "RADIUS_PLOT" "BOUNDARY_PLOT"

def _closest_point_on_segment(px, py, ax, ay, bx, by):
    """Closest point on segment AB to point P."""
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax)*dx + (py - ay)*dy) / (dx*dx + dy*dy + 1e-12)))
    return ax + t*dx, ay + t*dy

def _poly_closest(px, py, verts):
    """
    Returns (cx, cy, dist) — closest point on the polygon boundary to (px, py).
    verts : list of (x, y) tuples forming a closed polygon.
    """
    if not verts:
        raise ValueError("_poly_closest requires at least one vertex, got empty list.")
    if len(verts) < 2:
        # Degenerate polygon: single point — return that point
        return verts[0][0], verts[0][1], math.sqrt((px - verts[0][0])**2 + (py - verts[0][1])**2)
    best_d2, best_c = float("inf"), (verts[0][0], verts[0][1])
    n = len(verts)
    for i in range(n):
        ax, ay = verts[i]
        bx, by = verts[(i + 1) % n]
        cx, cy = _closest_point_on_segment(px, py, ax, ay, bx, by)
        d2 = (px - cx)**2 + (py - cy)**2
        if d2 < best_d2:
            best_d2, best_c = d2, (cx, cy)
    return best_c[0], best_c[1], math.sqrt(best_d2)

def _point_in_poly(px, py, verts):
    """Ray-casting point-in-polygon test."""
    inside = False
    n = len(verts)
    j = n - 1
    for i in range(n):
        xi, yi = verts[i];  xj, yj = verts[j]
        if ((yi > py) != (yj > py)) and (px < (xj - xi)*(py - yi)/(yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside

class CBFController:
    """
    Reactive CBF safety filter for single-integrator robots (x_dot = u).

    Barrier functions
    -----------------
    Circular obstacle / robot pair:
        h(x) = ||pos - c||^2 - d_safe^2

    Polygon obstacle (convex or convex hull approximated):
        h(x) = dist_to_boundary - (ROBOT_RADIUS + safety_margin)
        Gradient points from closest boundary point toward robot.

    The CBF condition  grad_h · u >= -γ·h(x)  is enforced by iterative
    projection onto violated half-spaces.

    Parameters
    ----------
    gamma         : CBF class-K gain (0.5 = gentle detours, 3.0 = sharp).
    k_att         : Attraction gain; slows robot proportionally near target.
    safety_margin : Extra clearance (field units) beyond nominal radii.
    max_iter      : Projection iterations per control step.
    """

    def __init__(self, gamma=1.5, k_att=1.2, safety_margin=2.0, max_iter=20):
        self.gamma         = gamma
        self.k_att         = k_att
        self.safety_margin = safety_margin
        self.max_iter      = max_iter

    # -- barrier functions ------------------------------------------------ #

    @staticmethod
    def _circle_constraint(rx, ry, cx, cy, d_safe):
        """h = ||pos-c||^2 - d_safe^2,  grad_h = 2*(pos-c)"""
        h  = (rx - cx)**2 + (ry - cy)**2 - d_safe**2
        gx, gy = 2*(rx - cx), 2*(ry - cy)
        return h, gx, gy

    @staticmethod
    def _polygon_constraint(rx, ry, verts, d_safe):
        """
        h = signed_dist - d_safe  where signed_dist is positive outside.

        Gradient is a soft blend of contributions from ALL edges weighted by
        proximity (weight = 1/d^2), rather than a hard snap to the single
        closest point.  This eliminates the gradient discontinuity at corners
        that caused oscillation / stalling.
        """
        inside = _point_in_poly(rx, ry, verts)
        sign   = -1.0 if inside else 1.0

        # Primary barrier value uses closest-point distance (unchanged)
        bx, by, dist = _poly_closest(rx, ry, verts)
        h = sign * dist - d_safe

        # Blended gradient: accumulate weighted push directions from every edge
        n_v = len(verts)
        gx_acc, gy_acc, w_acc = 0.0, 0.0, 0.0
        BLEND_RADIUS = d_safe * 3.0          # edges beyond this don't contribute
        for i in range(n_v):
            ax, ay = verts[i]
            bvx, bvy = verts[(i + 1) % n_v]
            cx, cy = _closest_point_on_segment(rx, ry, ax, ay, bvx, bvy)
            dx, dy = rx - cx, ry - cy
            ed = math.sqrt(dx*dx + dy*dy) + 1e-9
            if ed > BLEND_RADIUS:
                continue
            w = 1.0 / (ed * ed)             # inverse-square weight
            gx_acc += sign * w * dx / ed
            gy_acc += sign * w * dy / ed
            w_acc  += w

        if w_acc > 1e-12:
            gx = gx_acc / w_acc
            gy = gy_acc / w_acc
            # normalise to unit length
            glen = math.sqrt(gx*gx + gy*gy) + 1e-9
            gx, gy = gx/glen, gy/glen
        else:
            # fallback: direct push from closest point
            n = dist + 1e-9
            gx, gy = sign*(rx - bx)/n, sign*(ry - by)/n

        return h, gx, gy

    # -- helpers ---------------------------------------------------------- #

    @staticmethod
    def _clip(vx, vy, max_speed):
        n = math.sqrt(vx*vx + vy*vy)
        return (vx*max_speed/n, vy*max_speed/n) if n > max_speed else (vx, vy)

    def _nominal(self, robot):
        dx = robot.target.coords.x - robot.coords.x
        dy = robot.target.coords.y - robot.coords.y
        d  = math.sqrt(dx*dx + dy*dy)
        if d < 1e-6:
            return 0.0, 0.0
        speed = min(self.k_att * d, baseV)
        return dx/d * speed, dy/d * speed

    # -- constraint list -------------------------------------------------- #

    def _constraints(self, robot, obstacles, all_robots):
        """
        Builds (h, gx, gy) tuples for all active safety constraints.
        Obstacles may be circular (has .radius) or polygonal (has .border).
        """
        cons = []
        rx, ry = robot.coords.x, robot.coords.y

        for obs in obstacles:
            if hasattr(obs, "border"):
                h, gx, gy = self._polygon_constraint(
                    rx, ry, obs.border, ROBOT_RADIUS + self.safety_margin)
            else:                                              # circular obstacle
                h, gx, gy = self._circle_constraint(
                    rx, ry, obs.coords.x, obs.coords.y,
                    ROBOT_RADIUS + obs.radius + self.safety_margin)
            cons.append((h, gx, gy))

        for other in all_robots:
            if other is robot:
                continue
            h, gx, gy = self._circle_constraint(
                rx, ry, other.coords.x, other.coords.y,
                2*ROBOT_RADIUS + self.safety_margin)
            cons.append((h, gx, gy))

        return cons

    # -- main ------------------------------------------------------------- #

    def compute_safe_velocity(self, robot, obstacles, all_robots):
        """
        Returns a CBF-safe (Vx, Vy) toward robot.target.

        Parameters
        ----------
        robot      : robot instance from OnStage_Master
        obstacles  : list[obstacle] — may mix circular and polygon obstacles
        all_robots : list[robot]

        Returns
        -------
        (Vx, Vy) : tuple[float, float]
        """
        if robot.coords.distance_to(robot.target.coords) < DIST_THRESHOLD:
            return 0.0, 0.0

        Vx, Vy = self._nominal(robot)
        cons   = self._constraints(robot, obstacles, all_robots)

        # Project onto all violated CBF half-spaces.
        # _clip is intentionally OUTSIDE the inner loop so speed-clamping
        # never re-violates a constraint that was just satisfied.
        for _ in range(self.max_iter):
            changed = False
            for h_val, gx, gy in cons:
                rhs = -self.gamma * h_val
                lhs = gx*Vx + gy*Vy
                if lhs < rhs:
                    corr = (rhs - lhs) / (gx*gx + gy*gy + 1e-9)
                    Vx  += corr * gx
                    Vy  += corr * gy
                    changed = True
            if not changed:
                break
        Vx, Vy = self._clip(Vx, Vy, baseV)

        # Anti-stall: if the robot is near a constraint boundary (h < margin)
        # and the net velocity is near zero, inject a tangential perturbation
        # so it can slide around the obstacle rather than freezing.
        speed = math.sqrt(Vx*Vx + Vy*Vy)
        if speed < baseV * 0.05:
            # Find the most violated / tightest active constraint
            worst_gx, worst_gy, worst_h = 0.0, 0.0, float("inf")
            for h_val, gx, gy in cons:
                if h_val < worst_h:
                    worst_h, worst_gx, worst_gy = h_val, gx, gy
            if worst_h < self.safety_margin:
                # Tangent to the constraint gradient: rotate 90°
                tx, ty = -worst_gy, worst_gx
                # Choose the tangent direction that has positive dot with goal
                dx_goal = robot.target.coords.x - robot.coords.x
                dy_goal = robot.target.coords.y - robot.coords.y
                if tx*dx_goal + ty*dy_goal < 0:
                    tx, ty = -tx, -ty
                escape_speed = min(self.k_att * baseV, baseV)
                Vx = tx * escape_speed
                Vy = ty * escape_speed
                Vx, Vy = self._clip(Vx, Vy, baseV)

        return Vx, Vy


# ---------------------------------------------------------------------------
# cbf_follow_path  (replaces pfield_path + followPath)
# ---------------------------------------------------------------------------

def cbf_follow_path(cbf, robot, all_robots, obstacles):
    """
    Single reactive CBF step. Call every main-loop iteration.

    Returns True when the robot has arrived (caller should call
    cbf_stop_robot), False while still en route.
    """
    if robot.coords.distance_to(robot.target.coords) < DIST_THRESHOLD:
        robot.path = []
        return True

    Vx, Vy = cbf.compute_safe_velocity(robot, obstacles, all_robots)
    robot.path = [[robot.target.coords.x, robot.target.coords.y]]

    if ENABLE_WIFI:
        wifi_write(robot.sock, f"vx: {Vx/10:.3f}, vy: {Vy/10:.3f}, r: {robot.rotation:.0f}\n")
    return False


# ---------------------------------------------------------------------------
# cbf_stop_robot  (replaces stopRobot)
# ---------------------------------------------------------------------------

def cbf_stop_robot(robot):
    """Send zero-velocity command and clear path."""
    robot.path = []
    if ENABLE_WIFI:
        wifi_write(robot.sock, "vx: 0, vy: 0, r: 0\n")


# ===========================================================================
# Stand-alone simulation / visualiser
# ===========================================================================

def _offset_polygon(verts, amount):
    """
    Returns a new list of (x, y) vertices that is the input polygon
    expanded outward by `amount` field units.

    Algorithm: at each vertex compute the inward-facing bisector of the
    two adjacent edge normals, then move along the outward bisector by
    amount / sin(half-angle).  Falls back gracefully for near-degenerate
    angles.

    verts  : list of (x, y) — assumed counter-clockwise or clockwise;
             the function normalises direction automatically.
    amount : scalar >= 0
    """
    if amount <= 0:
        return list(verts)

    n = len(verts)

    # Ensure counter-clockwise winding (positive area)
    area2 = sum((verts[i][0] * verts[(i+1) % n][1] -
                 verts[(i+1) % n][0] * verts[i][1])
                for i in range(n))
    if area2 < 0:
        verts = list(reversed(verts))

    # Outward unit normal for each edge (CCW → right-hand normal points outward
    # in standard math / matplotlib coordinates where +y is up)
    def edge_normal(a, b):
        dx, dy = b[0]-a[0], b[1]-a[1]
        L = math.sqrt(dx*dx + dy*dy) + 1e-12
        return dy/L, -dx/L          # outward (right) normal for CCW polygon

    normals = [edge_normal(verts[i], verts[(i+1) % n]) for i in range(n)]

    new_verts = []
    for i in range(n):
        n1 = normals[(i - 1) % n]   # normal of edge ending at vertex i
        n2 = normals[i]             # normal of edge starting at vertex i

        # Bisector direction
        bx, by = n1[0]+n2[0], n1[1]+n2[1]
        blen = math.sqrt(bx*bx + by*by) + 1e-12
        bx, by = bx/blen, by/blen

        # Scale so the offset is exactly `amount` perpendicular to each edge
        sin_half = blen / 2.0          # sin(half interior angle)
        sin_half = max(sin_half, 0.15) # clamp for very sharp corners
        scale = amount / sin_half

        new_verts.append((verts[i][0] + bx*scale,
                          verts[i][1] + by*scale))
    return new_verts

if __name__ == "__main__":
    ENABLE_WIFI      = False
    
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib.patches import Polygon as MplPolygon
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Slider, Button
    import numpy as np

    # ------------------------------------------------------------------ #
    # Minimal stubs (mirror OnStage_Master field names exactly)           #
    # ------------------------------------------------------------------ #

    class Point:
        def __init__(self, x, y):
            self.x, self.y = x, y
        def distance_to(self, o):
            return math.sqrt((self.x - o.x)**2 + (self.y - o.y)**2)
        def __str__(self):
            return f"Point({self.x:.1f}, {self.y:.1f})"

    class SimRobot:
        path = []; state = "None"; haswater = False; rotation = 0; sock = None

        def __init__(self, x, y, gx, gy, tag=0):
            self.tag    = tag
            self.IP     = ""; self.port = 0
            self.coords = Point(x, y)
            self.target = type("T", (), {"coords": Point(gx, gy)})()
            self.path   = []
            self.trail  = [(x, y)]

        def move(self, vx, vy, dt):
            self.coords.x += vx * dt
            self.coords.y += vy * dt
            self.trail.append((self.coords.x, self.coords.y))
            if len(self.trail) > 400:
                self.trail.pop(0)

    class SimObstacle:
        """Circular obstacle."""
        def __init__(self, x, y, radius):
            self.coords = Point(x, y)
            self.radius = radius
            
    class SimPolyObstacle:
        """
        Polygonal obstacle defined by a list of (x, y) vertices.
        The CBF uses closest-point-on-boundary distance as its barrier.
        """
        def __init__(self, border):
            self.border = border   # list of (x, y) tuples

    # ------------------------------------------------------------------ #
    # Simulation parameters                                               #
    # ------------------------------------------------------------------ #

    SIM_FIELD       = 80
    SIM_DT          = 0.05
    STEPS_PER_FRAME = 3

    # ------------------------------------------------------------------ #
    # Scenarios                                                           #
    # ------------------------------------------------------------------ #

    def make_scenario(name):
        F = SIM_FIELD
        if name == "cross":
            r = [SimRobot(10, 40, 70, 40, 0), SimRobot(70, 40, 10, 40, 1),
                 SimRobot(40, 10, 40, 70, 2), SimRobot(40, 70, 40, 10, 3)]
            o = []

        elif name == "cluttered":
            r = [SimRobot(10, 20, 70, 60, 0), SimRobot(10, 60, 70, 20, 1)]
            o = [SimObstacle(35, 40, 5), SimObstacle(45, 30, 4),
                 SimObstacle(45, 50, 4), SimObstacle(55, 40, 5)]
            
        elif name == "onstage":
            r = [SimRobot(10, 20, 70, 60, 0), SimRobot(10, 60, 70, 20, 1)]
            o = [SimObstacle(35, 40, 5), SimObstacle(45, 30, 4),
                 SimObstacle(45, 50, 4), SimObstacle(55, 40, 5)]

        elif name == "circle":
            N, R = 5, 28
            cx, cy = F/2, F/2
            r = [SimRobot(cx + R*math.cos(2*math.pi*i/N),
                          cy + R*math.sin(2*math.pi*i/N),
                          cx - R*math.cos(2*math.pi*i/N),
                          cy - R*math.sin(2*math.pi*i/N), i)
                 for i in range(N)]
            o = []

        elif name == "polygon":
            # Two robots navigate around rectangular and triangular walls
            r = [SimRobot(10, 20, 70, 60, 0)]
            o = [
                # Rectangular wall in the centre
                SimPolyObstacle([(42, 52), (58, 52), (58, 58), (42, 58)]),
                # Triangle near bottom-right
                SimPolyObstacle([(22, 42), (38, 42), (38, 38), (22, 38)]),
            ]

        else:
            r, o = [], []

        return r, o

    # ------------------------------------------------------------------ #
    # Colours                                                             #
    # ------------------------------------------------------------------ #

    ROBOT_COLORS = ["#4fc3f7", "#81c784", "#ffb74d", "#f48fb1",
                    "#ce93d8", "#80cbc4", "#ff8a65", "#a5d6a7"]

    # ------------------------------------------------------------------ #
    # Figure layout                                                       #
    # ------------------------------------------------------------------ #

    fig = plt.figure(figsize=(10, 8), facecolor="#1a1a2e")
    fig.canvas.manager.set_window_title("OnStage CBF Simulation")

    ax = fig.add_axes([0.05, 0.30, 0.65, 0.65])
    ax.set_facecolor("#0d0d1a")
    ax.set_xlim(0, SIM_FIELD); ax.set_ylim(0, SIM_FIELD); ax.set_aspect("equal")
    ax.tick_params(colors="#555")
    for sp in ax.spines.values(): sp.set_edgecolor("#333")
    ax.set_title("OnStage CBF Simulation", color="#ccc", fontsize=11, pad=8)
    ax.set_xlabel("x (field units)", color="#666", fontsize=9)
    ax.set_ylabel("y (field units)", color="#666", fontsize=9)
    for v in range(0, SIM_FIELD + 1, 10):
        ax.axvline(v, color="#ffffff08", linewidth=0.5)
        ax.axhline(v, color="#ffffff08", linewidth=0.5)

    ax_info = fig.add_axes([0.73, 0.30, 0.25, 0.65])
    ax_info.axis("off"); ax_info.set_facecolor("#12122a")
    metric_text = ax_info.text(
        0.05, 0.97, "", transform=ax_info.transAxes,
        color="#aaa", fontsize=8, va="top", fontfamily="monospace", linespacing=1.7)

    # Sliders
    sl_gamma  = Slider(fig.add_axes([0.08, 0.20, 0.55, 0.025], facecolor="#12122a"),
                       "γ  (CBF gain)", 0.2, 5.0, valinit=1.5, color="#4fc3f7")
    sl_katt   = Slider(fig.add_axes([0.08, 0.15, 0.55, 0.025], facecolor="#12122a"),
                       "k_att (goal)",  0.3, 3.0, valinit=1.2, color="#81c784")
    sl_margin = Slider(fig.add_axes([0.08, 0.10, 0.55, 0.025], facecolor="#12122a"),
                       "Safety margin", 0.0, 8.0, valinit=2.0, color="#ffb74d")
    for sl in (sl_gamma, sl_katt, sl_margin):
        sl.label.set_color("#aaa"); sl.valtext.set_color("#fff")

    # Buttons
    SCENARIOS = ("cross", "cluttered", "onstage", "circle", "polygon")
    btn_xpos  = [0.05 + i*0.12 for i in range(len(SCENARIOS))]
    btn_axes  = {n: fig.add_axes([x, 0.04, 0.10, 0.04])
                 for n, x in zip(SCENARIOS, btn_xpos)}
    btn_axes["reset"] = fig.add_axes([0.72, 0.04, 0.10, 0.04])
    buttons = {}
    for name, bax in btn_axes.items():
        bax.set_facecolor("#263238")
        buttons[name] = Button(bax, name.capitalize(),
                               color="#263238", hovercolor="#37474f")
        buttons[name].label.set_color("#ccc"); buttons[name].label.set_fontsize(8)

    # ------------------------------------------------------------------ #
    # Simulation state + artist containers                                #
    # ------------------------------------------------------------------ #

    sim = dict(robots=[], obstacles=[], violations=0, t=0.0, steps=0,
               running=True, scenario="cross",
               cbf=CBFController(gamma=1.5, k_att=1.2, safety_margin=2.0))

    artists = dict(trails=[], robot_bodies=[], goal_markers=[],
                   safety_rings=[], obs_patches=[], obs_rings=[])

    def clear_artists():
        for group in artists.values():
            for a in group: a.remove()
            group.clear()

    def load_scenario(name):
        sim.update(scenario=name, t=0.0, steps=0, violations=0)
        sim["robots"], sim["obstacles"] = make_scenario(name)
        clear_artists()

        for obs in sim["obstacles"]:
            if isinstance(obs, SimPolyObstacle):
                # Filled polygon + dashed offset safety "ring"
                offset_border = _offset_polygon(
                    obs.border, sim["cbf"].safety_margin + ROBOT_RADIUS)
                patch = MplPolygon(obs.border, closed=True,
                                   color="#e05555", alpha=0.55, zorder=3)
                ring  = MplPolygon(offset_border, closed=True,
                                   fill=False, edgecolor="#e0555544",
                                   linewidth=0.8, linestyle="--", zorder=2)
            else:
                cx, cy = obs.coords.x, obs.coords.y
                patch = mpatches.Circle((cx, cy), obs.radius,
                                        color="#e05555", alpha=0.55, zorder=3)
                ring  = mpatches.Circle((cx, cy),
                                        obs.radius + sim["cbf"].safety_margin + ROBOT_RADIUS,
                                        fill=False, edgecolor="#e0555544",
                                        linewidth=0.8, linestyle="--", zorder=2)
            ax.add_patch(patch); ax.add_patch(ring)
            artists["obs_patches"].append(patch)
            artists["obs_rings"].append(ring)

        for i, rb in enumerate(sim["robots"]):
            col = ROBOT_COLORS[i % len(ROBOT_COLORS)]
            trail_line, = ax.plot([], [], color=col, alpha=0.55,
                                  linewidth=1.2, zorder=4)
            body   = mpatches.Circle((rb.coords.x, rb.coords.y), ROBOT_RADIUS,
                                     color=col, alpha=0.9, zorder=6)
            goal_mark, = ax.plot(rb.target.coords.x, rb.target.coords.y,
                                 marker="*", markersize=11, color=col,
                                 markeredgecolor="#fff", markeredgewidth=0.4, zorder=5)
            safety = mpatches.Circle((rb.coords.x, rb.coords.y),
                                     ROBOT_RADIUS + sim["cbf"].safety_margin,
                                     fill=False, edgecolor=col+"44",
                                     linewidth=0.7, linestyle="--", zorder=5)
            ax.add_patch(body); ax.add_patch(safety)
            artists["trails"].append(trail_line)
            artists["robot_bodies"].append(body)
            artists["goal_markers"].append(goal_mark)
            artists["safety_rings"].append(safety)

    load_scenario("cross")

    # ------------------------------------------------------------------ #
    # Callbacks                                                           #
    # ------------------------------------------------------------------ #

    for name in SCENARIOS:
        buttons[name].on_clicked(lambda _e, n=name: load_scenario(n))
    buttons["reset"].on_clicked(lambda _e: load_scenario(sim["scenario"]))

    def update_cbf(_val):
        sim["cbf"].gamma         = sl_gamma.val
        sim["cbf"].k_att         = sl_katt.val
        sim["cbf"].safety_margin = sl_margin.val
        for i, obs in enumerate(sim["obstacles"]):
            if i >= len(artists["obs_rings"]):
                continue
            if isinstance(obs, SimPolyObstacle):
                new_xy = _offset_polygon(
                    obs.border, sim["cbf"].safety_margin + ROBOT_RADIUS)
                artists["obs_rings"][i].set_xy(new_xy)
            else:
                artists["obs_rings"][i].set_radius(
                    obs.radius + sim["cbf"].safety_margin + ROBOT_RADIUS)
        for i in range(len(sim["robots"])):
            if i < len(artists["safety_rings"]):
                artists["safety_rings"][i].set_radius(
                    ROBOT_RADIUS + sim["cbf"].safety_margin)

    for sl in (sl_gamma, sl_katt, sl_margin): sl.on_changed(update_cbf)

    fig.canvas.mpl_connect("key_press_event",
                           lambda e: sim.update(running=not sim["running"])
                           if e.key == " " else None)

    # ------------------------------------------------------------------ #
    # Animation                                                           #
    # ------------------------------------------------------------------ #

    def animate(_frame):
        robots, obstacles, cbf = sim["robots"], sim["obstacles"], sim["cbf"]

        if sim["running"] and robots:
            for _ in range(STEPS_PER_FRAME):
                for rb in robots:
                    if rb.target is None: continue
                    if not cbf_follow_path(cbf, rb, robots, obstacles):
                        Vx, Vy = cbf.compute_safe_velocity(rb, obstacles, robots)
                        rb.move(Vx, Vy, SIM_DT)

                # Safety-violation counting
                for i in range(len(robots)):
                    for j in range(i+1, len(robots)):
                        if robots[i].coords.distance_to(robots[j].coords) < 2*ROBOT_RADIUS:
                            sim["violations"] += 1
                    for obs in obstacles:
                        if isinstance(obs, SimPolyObstacle):
                            _, _, d = _poly_closest(robots[i].coords.x, robots[i].coords.y,
                                                    obs.border)
                            if d < ROBOT_RADIUS:
                                sim["violations"] += 1
                        elif robots[i].coords.distance_to(obs.coords) < ROBOT_RADIUS + obs.radius:
                            sim["violations"] += 1

                sim["t"] += SIM_DT; sim["steps"] += 1

        # Update robot artists
        min_sep = float("inf")
        for i, rb in enumerate(robots):
            if i >= len(artists["robot_bodies"]): break
            if rb.trail:
                xs, ys = zip(*rb.trail)
                artists["trails"][i].set_data(xs, ys)
            artists["robot_bodies"][i].center = (rb.coords.x, rb.coords.y)
            artists["safety_rings"][i].center = (rb.coords.x, rb.coords.y)
            for j in range(i+1, len(robots)):
                d = rb.coords.distance_to(robots[j].coords) - 2*ROBOT_RADIUS
                if d < min_sep: min_sep = d
            for obs in obstacles:
                if isinstance(obs, SimPolyObstacle):
                    _, _, dist = _poly_closest(rb.coords.x, rb.coords.y, obs.border)
                    d = dist - ROBOT_RADIUS
                else:
                    d = rb.coords.distance_to(obs.coords) - ROBOT_RADIUS - obs.radius
                if d < min_sep: min_sep = d

        arrived = sum(1 for rb in robots
                      if rb.target and rb.coords.distance_to(rb.target.coords) < DIST_THRESHOLD)
        status  = "PAUSED (space)" if not sim["running"] else "RUNNING (space=pause)"
        sep_str = f"{min_sep:.1f}" if min_sep < float("inf") else "—"

        lines = [
            f"Scenario : {sim['scenario']}",
            f"Status   : {status}", "",
            f"γ        : {cbf.gamma:.2f}",
            f"k_att    : {cbf.k_att:.2f}",
            f"margin   : {cbf.safety_margin:.2f}", "",
            f"Time     : {sim['t']:.1f} s",
            f"Steps    : {sim['steps']}", "",
            f"Robots   : {len(robots)}",
            f"Arrived  : {arrived}/{len(robots)}",
            f"Min sep  : {sep_str} u",
            f"Violations: {sim['violations']}", "",
        ] + [f"R{rb.tag}: dist={rb.coords.distance_to(rb.target.coords):.1f}"
             if rb.target else f"R{rb.tag}: no target"
             for rb in robots]

        metric_text.set_text("\n".join(lines))
        metric_text.set_color("#aaa")

        return (artists["trails"] + artists["robot_bodies"] +
                artists["safety_rings"] + [metric_text])

    anim = FuncAnimation(fig, animate, interval=40, blit=False, cache_frame_data=False)

    print("OnStage CBF Simulation")
    print("  Scenarios : cross | cluttered | onstage | circle | polygon")
    print("  Space     : pause / resume")
    print("  Sliders   : adjust CBF params live")
    plt.show()
