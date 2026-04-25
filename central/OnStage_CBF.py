import math
from OnStage_WifiComms import wifi_write
 
# ---------------------------------------------------------------------------
# Tunable constants — match these to your physical setup
# ---------------------------------------------------------------------------

ROBOT_RADIUS = 3.46 # 10*ft
DIST_THRESHOLD = 5      # arrival threshold
ENABLE_WIFI    = False   # keep in sync with ENABLE_WIFI in OnStage_Master.py
baseV          = 8    # maximum speed in 10*ft/s
 
# ---------------------------------------------------------------------------
# CBFController
# ---------------------------------------------------------------------------
 
class CBFController:
    """
    Reactive CBF safety filter for single-integrator robots (x_dot = u).
 
    Every call to compute_safe_velocity() does two things:
      1. Computes a nominal velocity directed straight at the target.
      2. Iteratively projects that velocity onto the CBF safety half-spaces
         for every obstacle and other robot until all constraints are satisfied.
 
    The barrier function for each circular obstacle / robot pair is:
        h(x) = ||x - c||^2 - d_safe^2
 
    The CBF safety condition enforced is:
        grad_h . u  >=  -gamma * h(x)
 
    When this is violated, the velocity is corrected by the minimum change
    (orthogonal projection) that restores the condition.
 
    Parameters
    ----------
    gamma         : CBF class-K gain.
                    Low  (~0.5) — gentle, wide detours.
                    High (~3.0) — sharp last-moment corrections.
    k_att         : Attraction gain toward the goal. Scales nominal speed and
                    produces a proportional slow-down near the target so the
                    robot does not overshoot DIST_THRESHOLD at full speed.
    safety_margin : Extra clearance (field units) on top of
                    ROBOT_RADIUS + obstacle.radius. Increase if robots graze.
    max_iter      : Projection iterations per control step.
                    20 is sufficient for the obstacle counts in this system.
    """
 
    def __init__(
        self,
        gamma: float         = 1.5,
        k_att: float         = 1.2,
        safety_margin: float = 2.0,
        max_iter: int        = 20,
    ):
        self.gamma         = gamma
        self.k_att         = k_att
        self.safety_margin = safety_margin
        self.max_iter      = max_iter
 
    # ------------------------------------------------------------------ #
    # Barrier function and gradient                                        #
    # ------------------------------------------------------------------ #
 
    @staticmethod
    def _h(rx, ry, cx, cy, d_safe):
        """h(x) = ||pos - center||^2 - d_safe^2    (positive = safe)"""
        return (rx - cx) ** 2 + (ry - cy) ** 2 - d_safe ** 2
 
    @staticmethod
    def _grad_h(rx, ry, cx, cy):
        """grad_h = 2 * (pos - center)"""
        return 2.0 * (rx - cx), 2.0 * (ry - cy)
 
    # ------------------------------------------------------------------ #
    # Helpers                                                              #
    # ------------------------------------------------------------------ #
 
    @staticmethod
    def _clip(vx, vy, max_speed):
        """Scale velocity vector down to max_speed if needed."""
        n = math.sqrt(vx * vx + vy * vy)
        if n > max_speed:
            f = max_speed / n
            return vx * f, vy * f
        return vx, vy
 
    # ------------------------------------------------------------------ #
    # Nominal control — straight line to target at up to baseV            #
    # ------------------------------------------------------------------ #
 
    def _nominal(self, robot):
        """
        Velocity vector pointing from robot directly to its target.
        Speed = min(k_att * distance, baseV) so the robot slows down
        proportionally as it approaches, preventing overshoot.
        """
        dx = robot.target.coords.x - robot.coords.x
        dy = robot.target.coords.y - robot.coords.y
        d  = math.sqrt(dx * dx + dy * dy)
        if d < 1e-6:
            return 0.0, 0.0
        speed = min(self.k_att * d, baseV)
        return dx / d * speed, dy / d * speed
 
    # ------------------------------------------------------------------ #
    # Build constraint list                                                #
    # ------------------------------------------------------------------ #
 
    def _constraints(self, robot, obstacles, all_robots):
        """
        Returns [(h_val, grad_x, grad_y), ...] for every safety constraint.
 
        Static obstacles : uses obstacle.coords (Point) and obstacle.radius
        Other robots     : uses robot.coords (Point) and ROBOT_RADIUS
        """
        cons = []
        rx, ry = robot.coords.x, robot.coords.y
 
        # Static circular obstacles
        for obs in obstacles:
            d_safe = ROBOT_RADIUS + obs.radius + self.safety_margin
            h      = self._h(rx, ry, obs.coords.x, obs.coords.y, d_safe)
            gx, gy = self._grad_h(rx, ry, obs.coords.x, obs.coords.y)
            cons.append((h, gx, gy))
 
        # Other robots treated as moving circular obstacles
        for other in all_robots:
            if other is robot:
                continue
            d_safe = 2.0 * ROBOT_RADIUS + self.safety_margin
            h      = self._h(rx, ry, other.coords.x, other.coords.y, d_safe)
            gx, gy = self._grad_h(rx, ry, other.coords.x, other.coords.y)
            cons.append((h, gx, gy))
 
        return cons
 
    # ------------------------------------------------------------------ #
    # Main: compute safe velocity                                          #
    # ------------------------------------------------------------------ #
 
    def compute_safe_velocity(self, robot, obstacles, all_robots):
        """
        Compute a CBF-safe (Vx, Vy) for robot to move toward its target.
 
        Parameters
        ----------
        robot      : robot instance from OnStage_Master
        obstacles  : list[obstacle] — global obstacles list
        all_robots : list[robot]    — global robots list
 
        Returns
        -------
        (Vx, Vy) : tuple[float, float] in field units/s
        """
        if robot.coords.distance_to(robot.target.coords) < DIST_THRESHOLD:
            return 0.0, 0.0
 
        # Start from the unconstrained nominal velocity
        Vx, Vy = self._nominal(robot)
        cons   = self._constraints(robot, obstacles, all_robots)
 
        # Iterative half-space projection until all CBF conditions are met
        for _ in range(self.max_iter):
            changed = False
            for h_val, gx, gy in cons:
                rhs = -self.gamma * h_val       # required: grad_h . u >= rhs
                lhs = gx * Vx + gy * Vy
                if lhs < rhs:
                    g2   = gx * gx + gy * gy + 1e-9
                    corr = (rhs - lhs) / g2
                    Vx  += corr * gx
                    Vy  += corr * gy
                    changed = True
            Vx, Vy = self._clip(Vx, Vy, baseV)
            if not changed:
                break   # all constraints satisfied — early exit
 
        return Vx, Vy
 
 
# ---------------------------------------------------------------------------
# cbf_follow_path — replaces pfield_path() + followPath()
# ---------------------------------------------------------------------------
 
def cbf_follow_path(cbf: CBFController, robot, all_robots, obstacles):
    """
    Single reactive CBF step. Call this every main loop iteration.
 
    This replaces the two-step process of:
        robot.path = pfield_path(robot, obstacles, field_width)
        followPath(robot)
 
    No separate path planning call is needed anywhere. The function:
      1. Returns True immediately if the robot has reached its target.
      2. Computes a CBF-safe velocity toward the target.
      3. Sends the velocity over WiFi.
      4. Sets robot.path = [[target.x, target.y]] so displayElements()
         in OnStage_Master continues to draw the goal marker correctly.
 
    Parameters
    ----------
    cbf        : CBFController — one shared instance for all robots
    robot      : robot to control
    all_robots : list[robot]    — global robots list (inter-robot avoidance)
    obstacles  : list[obstacle] — global obstacles list
 
    Returns
    -------
    True  — robot has arrived; caller should call cbf_stop_robot(robot)
    False — robot is still en route
    """
    if robot.coords.distance_to(robot.target.coords) < DIST_THRESHOLD:
        robot.path = []
        return True
 
    Vx, Vy = cbf.compute_safe_velocity(robot, obstacles, all_robots)
 
    # Update robot.path so displayElements() still shows the goal
    robot.path = [[robot.target.coords.x, robot.target.coords.y]]
 
    if ENABLE_WIFI:
        wifi_write(robot.sock, f"vx: {Vx/10:.3f}, vy: {Vy/10:.3f}, r: {robot.rotation:.0f}\n")
 
    return False
 
 
# ---------------------------------------------------------------------------
# cbf_stop_robot — replaces stopRobot()
# ---------------------------------------------------------------------------
 
def cbf_stop_robot(robot):
    """
    Send a zero-velocity command. Drop-in replacement for stopRobot(robot).
    Also clears robot.path so displayElements() shows no pending waypoints.
    """
    robot.path = []
    if ENABLE_WIFI:
        wifi_write(robot.sock, "vx: 0, vy: 0, r: 0\n")
        
### testing ###
if __name__ == "__main__":
    import math as _math
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Slider, Button
 
    # ------------------------------------------------------------------ #
    # Minimal stubs for the classes defined in OnStage_Master             #
    # (identical field names and logic — no import needed)                #
    # ------------------------------------------------------------------ #
 
    class Point:
        def __init__(self, x, y):
            self.x = x
            self.y = y
        def distance_to(self, other):
            return _math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)
        def __str__(self):
            return f"Point({self.x:.1f}, {self.y:.1f})"
 
    class SimRobot:
        """Mirrors the robot class from OnStage_Master exactly."""
        path  = []
        state = "None"
        haswater = False
        rotation = 0
        sock  = None
 
        def __init__(self, x, y, gx, gy, tag=0):
            self.tag    = tag
            self.IP     = ""
            self.port   = 0
            self.coords = Point(x, y)
            self.target = type("T", (), {"coords": Point(gx, gy)})()
            self.path   = []
            self.trail  = [(x, y)]   # simulation-only: position history
 
        def move(self, vx, vy, dt):
            """Integrate velocity, record trail."""
            self.coords.x += vx * dt
            self.coords.y += vy * dt
            self.trail.append((self.coords.x, self.coords.y))
            if len(self.trail) > 400:
                self.trail.pop(0)
 
    class SimObstacle:
        """Mirrors the obstacle class from OnStage_Master exactly."""
        def __init__(self, x, y, radius):
            self.coords = Point(x, y)
            self.radius = radius
            self.border = None   # not needed for simulation
 
    # ------------------------------------------------------------------ #
    # Simulation parameters                                                #
    # ------------------------------------------------------------------ #
 
    SIM_FIELD  = 80     # field_width from OnStage_Master
    SIM_DT     = 0.05   # integration timestep (seconds)
    STEPS_PER_FRAME = 3 # physics steps per animation frame
 
    # ------------------------------------------------------------------ #
    # Scenarios                                                            #
    # ------------------------------------------------------------------ #
 
    def make_scenario(name):
        """
        Returns (robots, obstacles) using SimRobot and SimObstacle,
        matching the field_width=80 coordinate system of OnStage_Master.
        """
        F = SIM_FIELD
 
        if name == "cross":
            # 4-way crossing — classic deadlock test
            r = [
                SimRobot(10, 40, 70, 40, tag=0),
                SimRobot(70, 40, 10, 40, tag=1),
                SimRobot(40, 10, 40, 70, tag=2),
                SimRobot(40, 70, 40, 10, tag=3),
            ]
            o = []
 
        elif name == "cluttered":
            # 2 robots navigate through dense static obstacles
            r = [
                SimRobot(10, 20, 70, 60, tag=0),
                SimRobot(10, 60, 70, 20, tag=1),
            ]
            o = [
                SimObstacle(30, 40, 5),
                SimObstacle(45, 25, 4),
                SimObstacle(45, 55, 4),
                SimObstacle(60, 40, 5),
            ]
 
        elif name == "onstage":
            # Mirrors the actual OnStage setup:
            # 2 robots, 3 obstacles, targets represent ice/plant positions
            r = [
                SimRobot(15, 15, 65, 65, tag=6),
                SimRobot(65, 15, 15, 65, tag=7),
            ]
            o = [
                SimObstacle(40, 40, 6),
                SimObstacle(25, 55, 4),
                SimObstacle(55, 25, 4),
            ]
 
        elif name == "circle":
            # 5 robots equally spaced on a ring, swap to opposite side
            import math as m
            N, R = 5, 28
            cx, cy = F / 2, F / 2
            r = []
            for i in range(N):
                a = 2 * m.pi * i / N
                r.append(SimRobot(
                    cx + R * m.cos(a),  cy + R * m.sin(a),
                    cx - R * m.cos(a),  cy - R * m.sin(a),
                    tag=i,
                ))
            o = []
 
        else:
            r, o = [], []
 
        return r, o
 
    # ------------------------------------------------------------------ #
    # Colour palette                                                       #
    # ------------------------------------------------------------------ #
 
    ROBOT_COLORS = ["#4fc3f7", "#81c784", "#ffb74d", "#f48fb1",
                    "#ce93d8", "#80cbc4", "#ff8a65", "#a5d6a7"]
 
    # ------------------------------------------------------------------ #
    # Build figure                                                         #
    # ------------------------------------------------------------------ #
 
    fig = plt.figure(figsize=(10, 8), facecolor="#1a1a2e")
    fig.canvas.manager.set_window_title("OnStage CBF Simulation")
 
    # Main axes — leave room at bottom for sliders / buttons
    ax = fig.add_axes([0.05, 0.30, 0.65, 0.65])
    ax.set_facecolor("#0d0d1a")
    ax.set_xlim(0, SIM_FIELD)
    ax.set_ylim(0, SIM_FIELD)
    ax.set_aspect("equal")
    ax.tick_params(colors="#555")
    for sp in ax.spines.values():
        sp.set_edgecolor("#333")
    ax.set_title("OnStage CBF Simulation", color="#ccc", fontsize=11, pad=8)
    ax.set_xlabel("x (field units)", color="#666", fontsize=9)
    ax.set_ylabel("y (field units)", color="#666", fontsize=9)
 
    # Grid
    for v in range(0, SIM_FIELD + 1, 10):
        ax.axvline(v, color="#ffffff08", linewidth=0.5)
        ax.axhline(v, color="#ffffff08", linewidth=0.5)
 
    # Metric panel (right of canvas)
    ax_info = fig.add_axes([0.73, 0.30, 0.25, 0.65])
    ax_info.axis("off")
    ax_info.set_facecolor("#12122a")
    metric_text = ax_info.text(
        0.05, 0.97, "", transform=ax_info.transAxes,
        color="#aaa", fontsize=8, va="top", fontfamily="monospace",
        linespacing=1.7,
    )
 
    # ------------------------------------------------------------------ #
    # Sliders                                                              #
    # ------------------------------------------------------------------ #
 
    slider_style = dict(facecolor="#1a1a2e", color="#4fc3f7")
 
    ax_gamma  = fig.add_axes([0.08, 0.20, 0.55, 0.025], facecolor="#12122a")
    ax_katt   = fig.add_axes([0.08, 0.15, 0.55, 0.025], facecolor="#12122a")
    ax_margin = fig.add_axes([0.08, 0.10, 0.55, 0.025], facecolor="#12122a")
 
    sl_gamma  = Slider(ax_gamma,  "γ  (CBF gain)",     0.2, 5.0, valinit=1.5,  color="#4fc3f7")
    sl_katt   = Slider(ax_katt,   "k_att (goal)",       0.3, 3.0, valinit=1.2,  color="#81c784")
    sl_margin = Slider(ax_margin, "Safety margin",       0.0, 8.0, valinit=2.0,  color="#ffb74d")
 
    for sl in (sl_gamma, sl_katt, sl_margin):
        sl.label.set_color("#aaa")
        sl.valtext.set_color("#fff")
 
    # ------------------------------------------------------------------ #
    # Scenario buttons                                                     #
    # ------------------------------------------------------------------ #
 
    btn_axes = {
        "cross":     fig.add_axes([0.08, 0.04, 0.10, 0.04]),
        "cluttered": fig.add_axes([0.20, 0.04, 0.10, 0.04]),
        "onstage":   fig.add_axes([0.32, 0.04, 0.10, 0.04]),
        "circle":    fig.add_axes([0.44, 0.04, 0.10, 0.04]),
        "reset":     fig.add_axes([0.60, 0.04, 0.10, 0.04]),
    }
    buttons = {}
    btn_colors = {"cross": "#263238", "cluttered": "#263238",
                  "onstage": "#263238", "circle": "#263238", "reset": "#37474f"}
    for name, bax in btn_axes.items():
        bax.set_facecolor(btn_colors[name])
        buttons[name] = Button(bax, name.capitalize(),
                               color=btn_colors[name], hovercolor="#37474f")
        buttons[name].label.set_color("#ccc")
        buttons[name].label.set_fontsize(8)
 
    # ------------------------------------------------------------------ #
    # Simulation state                                                     #
    # ------------------------------------------------------------------ #
 
    sim = {
        "robots":    [],
        "obstacles": [],
        "cbf":       CBFController(gamma=1.5, k_att=1.2, safety_margin=2.0),
        "running":   True,
        "t":         0.0,
        "steps":     0,
        "violations": 0,
        "scenario":  "cross",
    }
 
    # Matplotlib artist containers (rebuilt on scenario load)
    artists = {
        "trails":       [],
        "robot_bodies": [],
        "goal_markers": [],
        "safety_rings": [],
        "obs_patches":  [],
        "obs_rings":    [],
    }
 
    def clear_artists():
        for group in artists.values():
            for a in group:
                a.remove()
            group.clear()
 
    def load_scenario(name):
        sim["scenario"]  = name
        sim["t"]         = 0.0
        sim["steps"]     = 0
        sim["violations"] = 0
        sim["robots"], sim["obstacles"] = make_scenario(name)
 
        clear_artists()
 
        # Rebuild static obstacle artists
        for obs in sim["obstacles"]:
            patch = mpatches.Circle(
                (obs.coords.x, obs.coords.y), obs.radius,
                color="#e05555", alpha=0.55, zorder=3,
            )
            ring = mpatches.Circle(
                (obs.coords.x, obs.coords.y),
                obs.radius + sim["cbf"].safety_margin + ROBOT_RADIUS,
                fill=False, edgecolor="#e0555544", linewidth=0.8,
                linestyle="--", zorder=2,
            )
            ax.add_patch(patch)
            ax.add_patch(ring)
            artists["obs_patches"].append(patch)
            artists["obs_rings"].append(ring)
 
        # Rebuild per-robot artists
        for i, rb in enumerate(sim["robots"]):
            col = ROBOT_COLORS[i % len(ROBOT_COLORS)]
 
            trail_line, = ax.plot([], [], color=col, alpha=0.55,
                                  linewidth=1.2, zorder=4)
            body = mpatches.Circle(
                (rb.coords.x, rb.coords.y), ROBOT_RADIUS,
                color=col, alpha=0.9, zorder=6,
            )
            goal_mark, = ax.plot(
                rb.target.coords.x, rb.target.coords.y,
                marker="*", markersize=11, color=col,
                markeredgecolor="#fff", markeredgewidth=0.4, zorder=5,
            )
            safety = mpatches.Circle(
                (rb.coords.x, rb.coords.y), ROBOT_RADIUS + sim["cbf"].safety_margin,
                fill=False, edgecolor=col + "44", linewidth=0.7,
                linestyle="--", zorder=5,
            )
            ax.add_patch(body)
            ax.add_patch(safety)
 
            artists["trails"].append(trail_line)
            artists["robot_bodies"].append(body)
            artists["goal_markers"].append(goal_mark)
            artists["safety_rings"].append(safety)
 
    load_scenario("cross")
 
    # ------------------------------------------------------------------ #
    # Button callbacks                                                     #
    # ------------------------------------------------------------------ #
 
    def on_scenario(name):
        def cb(_event):
            load_scenario(name)
        return cb
 
    for name in ("cross", "cluttered", "onstage", "circle"):
        buttons[name].on_clicked(on_scenario(name))
 
    def on_reset(_event):
        load_scenario(sim["scenario"])
 
    buttons["reset"].on_clicked(on_reset)
 
    # ------------------------------------------------------------------ #
    # Slider callbacks — update CBF params live                           #
    # ------------------------------------------------------------------ #
 
    def update_cbf(_val):
        sim["cbf"].gamma         = sl_gamma.val
        sim["cbf"].k_att         = sl_katt.val
        sim["cbf"].safety_margin = sl_margin.val
        # Update safety ring radii on obstacles (visual only)
        for i, obs in enumerate(sim["obstacles"]):
            if i < len(artists["obs_rings"]):
                artists["obs_rings"][i].set_radius(
                    obs.radius + sim["cbf"].safety_margin + ROBOT_RADIUS
                )
        # Update safety ring radii on robots
        for i in range(len(sim["robots"])):
            if i < len(artists["safety_rings"]):
                artists["safety_rings"][i].set_radius(
                    ROBOT_RADIUS + sim["cbf"].safety_margin
                )
 
    sl_gamma.on_changed(update_cbf)
    sl_katt.on_changed(update_cbf)
    sl_margin.on_changed(update_cbf)
 
    # ------------------------------------------------------------------ #
    # Pause on spacebar                                                    #
    # ------------------------------------------------------------------ #
 
    def on_key(event):
        if event.key == " ":
            sim["running"] = not sim["running"]
 
    fig.canvas.mpl_connect("key_press_event", on_key)
 
    # ------------------------------------------------------------------ #
    # Animation update                                                     #
    # ------------------------------------------------------------------ #
 
    def animate(_frame):
        robots    = sim["robots"]
        obstacles = sim["obstacles"]
        cbf       = sim["cbf"]
 
        if sim["running"] and robots:
            for _ in range(STEPS_PER_FRAME):
                for rb in robots:
                    if rb.target is None:
                        continue
                    arrived = cbf_follow_path(cbf, rb, robots, obstacles)
                    if not arrived:
                        Vx, Vy = cbf.compute_safe_velocity(rb, obstacles, robots)
                        rb.move(Vx, Vy, SIM_DT)
 
                # Count safety violations
                for i in range(len(robots)):
                    for j in range(i + 1, len(robots)):
                        d = robots[i].coords.distance_to(robots[j].coords)
                        if d < 2 * ROBOT_RADIUS:
                            sim["violations"] += 1
                    for obs in obstacles:
                        d = robots[i].coords.distance_to(obs.coords)
                        if d < ROBOT_RADIUS + obs.radius:
                            sim["violations"] += 1
 
                sim["t"]     += SIM_DT
                sim["steps"] += 1
 
        # Update robot artists
        min_sep = float("inf")
        for i, rb in enumerate(robots):
            if i >= len(artists["robot_bodies"]):
                break
 
            # Trail
            if rb.trail:
                xs, ys = zip(*rb.trail)
                artists["trails"][i].set_data(xs, ys)
 
            # Body + safety ring
            artists["robot_bodies"][i].center = (rb.coords.x, rb.coords.y)
            artists["safety_rings"][i].center = (rb.coords.x, rb.coords.y)
 
            # Nearest-neighbour separation for metrics
            for j in range(i + 1, len(robots)):
                d = rb.coords.distance_to(robots[j].coords) - 2 * ROBOT_RADIUS
                if d < min_sep:
                    min_sep = d
            for obs in obstacles:
                d = rb.coords.distance_to(obs.coords) - ROBOT_RADIUS - obs.radius
                if d < min_sep:
                    min_sep = d
 
        # Metrics panel
        arrived = sum(
            1 for rb in robots
            if rb.target and rb.coords.distance_to(rb.target.coords) < DIST_THRESHOLD
        )
        sep_str = f"{min_sep:.1f}" if min_sep < float("inf") else "—"
        viol_col = "#ff6b6b" if sim["violations"] > 0 else "#81c784"
        status = "PAUSED (space)" if not sim["running"] else "RUNNING (space=pause)"
 
        lines = [
            f"Scenario : {sim['scenario']}",
            f"Status   : {status}",
            f"",
            f"γ        : {cbf.gamma:.2f}",
            f"k_att    : {cbf.k_att:.2f}",
            f"margin   : {cbf.safety_margin:.2f}",
            f"",
            f"Time     : {sim['t']:.1f} s",
            f"Steps    : {sim['steps']}",
            f"",
            f"Robots   : {len(robots)}",
            f"Arrived  : {arrived}/{len(robots)}",
            f"Min sep  : {sep_str} u",
            f"Violations: {sim['violations']}",
            f"",
        ]
        for j, rb in enumerate(robots):
            col = ROBOT_COLORS[j % len(ROBOT_COLORS)]
            d   = rb.coords.distance_to(rb.target.coords) if rb.target else 0
            lines.append(f"R{rb.tag}: dist={d:.1f}")
 
        metric_text.set_text("\n".join(lines))
        metric_text.set_color("#aaa")
 
        return (
            artists["trails"]
            + artists["robot_bodies"]
            + artists["safety_rings"]
            + [metric_text]
        )
 
    anim = FuncAnimation(
        fig, animate,
        interval=40,       # ~25 fps
        blit=False,        # blit=False so metric text redraws cleanly
        cache_frame_data=False,
    )
 
    print("OnStage CBF Simulation")
    print("  Scenarios : cross | cluttered | onstage | circle")
    print("  Space     : pause / resume")
    print("  Sliders   : adjust CBF params live")
    plt.show()
