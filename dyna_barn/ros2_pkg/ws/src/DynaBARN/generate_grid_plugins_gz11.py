#!/usr/bin/env python3
import os, math, random, argparse, subprocess
import numpy as np


# ---------- Gazebo plugin codegen ----------
def make_head(name, duration):
    return f"""
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

namespace gazebo {{
  class {name} : public ModelPlugin {{
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {{
      this->model = _parent;
      gazebo::common::PoseAnimationPtr anim(
        new gazebo::common::PoseAnimation("{name}", {duration:.2f}, true));
      gazebo::common::PoseKeyFrame *key;
"""


def make_waypoint(t, x, y):
    return f"""
      key = anim->CreateKeyFrame({t:.3f});
      key->Translation(ignition::math::Vector3d({x:.3f}, {y:.3f}, 0.0));
      key->Rotation(ignition::math::Quaterniond(0,0,0));
"""


def make_tail(name):
    return f"""
      this->model->SetAnimation(anim);
    }}
  private: physics::ModelPtr model;
  private: event::ConnectionPtr updateConnection;
  }};
  GZ_REGISTER_MODEL_PLUGIN({name})
}}
"""


def make_CMakeLists(names):
    lines = [
        "cmake_minimum_required(VERSION 3.10)",
        "project(animated_grid)",
        "find_package(gazebo REQUIRED)",
        "set(CMAKE_CXX_STANDARD 14)",
        "add_definitions(${GAZEBO_CXX_FLAGS})",
        "include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})",
        "link_directories(${GAZEBO_LIBRARY_DIRS})",
        "",
    ]
    for n in names:
        lines += [
            f"add_library({n} SHARED {n}.cc)",
            f"target_link_libraries({n} ${{GAZEBO_LIBRARIES}})",
            "",
        ]
    return "\n".join(lines)


def make_moving_model(plugin_so, r=0.25, h=0.50, m=1.0):
    ixx = iyy = (1.0 / 12.0) * m * (3.0 * (r * r) + h * h)
    izz = 0.5 * m * (r * r)
    name = plugin_so.split(".so")[0]
    return f"""
  <model name='{name}'>
    <static>1</static>
    <pose>0 0 0 0 0 0</pose>
    <link name='link'>
      <inertial><mass>{m}</mass>
        <inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name='collision'>
        <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
      </collision>
      <visual name='visual'>
        <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
        <material><script><name>Gazebo/Red</name><uri>file://media/materials/scripts/gazebo.material</uri></script></material>
      </visual>
    </link>
    <plugin name='{name}' filename='{plugin_so}'/>
  </model>
"""


# ---------- Path generators ----------
def row_path(y, x_min, x_max, passes):
    xs, ys = [], []
    for _ in range(passes):
        xs += [x_min, x_max, x_min]
        ys += [y, y, y]
    return _dedupe(xs, ys)


def col_path(x, y_min, y_max, passes):
    xs, ys = [], []
    for _ in range(passes):
        xs += [x, x, x]
        ys += [y_min, y_max, y_min]
    return _dedupe(xs, ys)


def random_path(x_min, x_max, y_min, y_max, points, bounce=False, seed=None):
    """
    Create a random closed path inside bounds.
    - If bounce=False: pick random waypoints, loop back to start.
    - If bounce=True: make a random walk that 'bounces' off walls.
    """
    rng = random.Random(seed)
    if points < 2:
        points = 2

    if not bounce:
        xs = [rng.uniform(x_min, x_max) for _ in range(points)]
        ys = [rng.uniform(y_min, y_max) for _ in range(points)]
        # close the loop
        xs.append(xs[0])
        ys.append(ys[0])
        return _dedupe(xs, ys)

    # bounce walk
    cx = rng.uniform(x_min, x_max)
    cy = rng.uniform(y_min, y_max)
    ang = rng.uniform(-math.pi, math.pi)
    step = max((x_max - x_min + y_max - y_min) / 30.0, 0.2)

    xs, ys = [cx], [cy]
    for _ in range(points - 1):
        cx += step * math.cos(ang)
        cy += step * math.sin(ang)
        bounced = False
        if cx < x_min:
            cx = x_min + (x_min - cx)
            ang = math.pi - ang
            bounced = True
        if cx > x_max:
            cx = x_max - (cx - x_max)
            ang = math.pi - ang
            bounced = True
        if cy < y_min:
            cy = y_min + (y_min - cy)
            ang = -ang
            bounced = True
        if cy > y_max:
            cy = y_max - (cy - y_max)
            ang = -ang
            bounced = True
        # inject a tiny angle jitter so paths don't become degenerate
        if not bounced:
            ang += rng.uniform(-0.35, 0.35)
        xs.append(cx)
        ys.append(cy)
    # close the loop
    xs.append(xs[0])
    ys.append(ys[0])
    return _dedupe(xs, ys)


def _dedupe(xs, ys):
    path = [(xs[0], ys[0])]
    for i in range(1, len(xs)):
        if (xs[i], ys[i]) != path[-1]:
            path.append((xs[i], ys[i]))
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    return xs, ys


def build_times(xs, ys, speed, dwell=0.25, speed_min=None, speed_max=None, rng=None):
    """Constant or segment-wise random speed. Returns strictly increasing times."""
    if rng is None:
        rng = random.Random()
    ts = [0.0]
    for i in range(len(xs) - 1):
        d = math.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i])
        if speed_min is not None and speed_max is not None and speed_max > 0:
            seg_speed = rng.uniform(speed_min, speed_max)
        else:
            seg_speed = speed
        seg_speed = max(1e-6, seg_speed)
        ts.append(ts[-1] + d / seg_speed)
    ts[-1] += dwell
    return ts


# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser()
    # layout
    ap.add_argument("--save_dir", default="./worlds")
    ap.add_argument("--plugins_dir", default="./plugins")
    ap.add_argument("--base_world", default="base_boilerplate.world")  # contains TOKEN
    # bounds & lanes
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--cols", type=int, default=6)
    ap.add_argument("--x_min", type=float, default=-9.0)
    ap.add_argument("--x_max", type=float, default=9.0)
    ap.add_argument("--y_min", type=float, default=-8.0)
    ap.add_argument("--y_max", type=float, default=8.0)
    # population & kinematics
    ap.add_argument("--count", type=int, default=40, help="number of moving obstacles")
    ap.add_argument("--passes", type=int, default=3, help="used for controlled mode")
    ap.add_argument(
        "--speed", type=float, default=1.0, help="constant speed if no range is given"
    )
    ap.add_argument(
        "--speed_min",
        type=float,
        default=None,
        help="random speed lower bound per segment",
    )
    ap.add_argument(
        "--speed_max",
        type=float,
        default=None,
        help="random speed upper bound per segment",
    )
    ap.add_argument("--dwell", type=float, default=0.25, help="pause at loop end (s)")
    ap.add_argument("--stagger_phase", action="store_true")
    ap.add_argument("--seed", type=int, default=11)
    # NEW: movement selection
    ap.add_argument(
        "--movement",
        choices=["random", "controlled"],
        default="controlled",
        help="random: random path in bounds; controlled: lanes",
    )
    ap.add_argument(
        "--axis",
        choices=["row", "column"],
        default="row",
        help="which lane axis to use when movement=controlled",
    )
    # NEW: random mode configurables
    ap.add_argument(
        "--rand_points",
        type=int,
        default=8,
        help="number of waypoints for random path (>=2)",
    )
    ap.add_argument(
        "--rand_bounce",
        action="store_true",
        help="use bouncing random-walk instead of random waypoints",
    )
    ap.add_argument(
        "--speed_mode",
        choices=["constant", "segment", "agent"],
        default="constant",
        help="constant: use --speed; segment: random per segment; agent: random per robot",
    )

    args = ap.parse_args()

    rng = random.Random(args.seed)
    np.random.seed(args.seed)

    os.makedirs(args.plugins_dir, exist_ok=True)
    os.makedirs(args.save_dir, exist_ok=True)
    build = os.path.join(args.plugins_dir, "build")
    os.makedirs(build, exist_ok=True)

    ys_rows = np.linspace(args.y_min, args.y_max, max(1, args.rows))
    xs_cols = np.linspace(args.x_min, args.x_max, max(1, args.cols))

    names = []
    for i in range(args.count):
        name = f"obs_{i}"

        # 1) Build a path
        if args.movement == "random":
            xs, yy = random_path(
                args.x_min,
                args.x_max,
                args.y_min,
                args.y_max,
                points=max(2, args.rand_points),
                bounce=args.rand_bounce,
                seed=args.seed + i,
            )
            # optional staggering = flip starting phase by reversing the loop
            if args.stagger_phase and (i % 2 == 1):
                xs = xs[::-1]
                yy = yy[::-1]

        else:  # controlled
            if args.axis == "column":
                col_x = float(xs_cols[i % len(xs_cols)])
                xs, yy = col_path(col_x, args.y_min, args.y_max, args.passes)
                if args.stagger_phase and (i % 2 == 1):
                    xs = xs[::-1]
                    yy = yy[::-1]
            else:  # row
                row_y = float(ys_rows[i % len(ys_rows)])
                xs, yy = row_path(row_y, args.x_min, args.x_max, args.passes)
                if args.stagger_phase and (i % 2 == 0):
                    xs = xs[::-1]
                    yy = yy[::-1]

        # 2) Build timing
        # ts = build_times(
        #     xs, yy,
        #     speed=args.speed,
        #     dwell=args.dwell,
        #     speed_min=args.speed_min,
        #     speed_max=args.speed_max,
        #     rng=rng
        # )
        if (
            args.speed_mode == "segment"
            and args.speed_min is not None
            and args.speed_max is not None
        ):
            ts = build_times(
                xs,
                yy,
                speed=args.speed,
                dwell=args.dwell,
                speed_min=args.speed_min,
                speed_max=args.speed_max,
                rng=rng,
            )
        elif (
            args.speed_mode == "agent"
            and args.speed_min is not None
            and args.speed_max is not None
        ):
            agent_speed = rng.uniform(args.speed_min, args.speed_max)
            ts = build_times(
                xs,
                yy,
                speed=agent_speed,
                dwell=args.dwell,
                speed_min=None,
                speed_max=None,
                rng=rng,
            )
        else:  # constant
            ts = build_times(
                xs,
                yy,
                speed=args.speed,
                dwell=args.dwell,
                speed_min=None,
                speed_max=None,
                rng=rng,
            )
        # 3) Emit plugin source
        cc = make_head(name, ts[-1])
        for t, x, y in zip(ts, xs, yy):
            cc += make_waypoint(t, x, y)
        cc += make_tail(name)
        with open(os.path.join(args.plugins_dir, f"{name}.cc"), "w") as f:
            f.write(cc)
        names.append(name)

    # CMakeLists and build
    with open(os.path.join(args.plugins_dir, "CMakeLists.txt"), "w") as f:
        f.write(make_CMakeLists(names))
    cwd = os.getcwd()
    os.chdir(build)
    subprocess.run(["cmake", ".."], check=True)
    subprocess.run(["make", "-j", str(os.cpu_count() or 2)], check=True)
    os.chdir(cwd)

    # Assemble world
    with open(args.base_world, "r") as f:
        s = f.read()
    if "TOKEN" not in s:
        raise RuntimeError("base_world must contain 'TOKEN'")
    p1, p2 = s.split("TOKEN", 1)

    so_list = sorted([f for f in os.listdir(build) if f.endswith(".so")])
    mid = "".join([make_moving_model(so) for so in so_list])

    out_world = os.path.join(args.save_dir, "world.world")
    with open(out_world, "w") as f:
        f.write(p1 + mid + p2)

    print("\nWrote", out_world)
    print(f"export GAZEBO_PLUGIN_PATH={build}:$GAZEBO_PLUGIN_PATH")
    print(f"gzserver --verbose {out_world}")


if __name__ == "__main__":
    main()
