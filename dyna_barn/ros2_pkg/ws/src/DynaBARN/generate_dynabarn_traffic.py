#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DynaBARN mover generator (Gazebo Classic 11 PoseAnimation)
- Builds C++ model plugins that animate obstacles
- Emits a .world that includes all built plugins

Movement modes:
  * controlled (lanes): --movement controlled --axis row|column
  * random:             --movement random [--rand_bounce]
  * traffic (lane-change): --movement traffic --axis row|column

Usage example:
  python3 generate_dynabarn_movers.py \
    --movement traffic --axis row \
    --rows 6 --cols 6 --x_min -9 --x_max 9 --y_min -8 --y_max 8 \
    --passes 4 --count 36 --speed 1.3 \
    --lane_change_prob 0.4 --traffic_samples 50 --stagger_phase
"""
import os, math, random, argparse, subprocess
import numpy as np

# ------------------------ Gazebo C++ codegen ------------------------

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
        new gazebo::common::PoseAnimation("{name}", {duration:.3f}, true));
      gazebo::common::PoseKeyFrame *key;
"""

def make_waypoint(t, x, y):
    return f"""
      key = anim->CreateKeyFrame({t:.4f});
      key->Translation(ignition::math::Vector3d({x:.4f}, {y:.4f}, 0.0));
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
        "project(animated_movers)",
        "find_package(gazebo REQUIRED)",
        "set(CMAKE_CXX_STANDARD 14)",
        "add_definitions(${GAZEBO_CXX_FLAGS})",
        "include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})",
        "link_directories(${GAZEBO_LIBRARY_DIRS})",
        "",
    ]
    for n in names:
        lines += [f"add_library({n} SHARED {n}.cc)",
                  f"target_link_libraries({n} ${{GAZEBO_LIBRARIES}})",
                  ""]
    return "\n".join(lines)

def make_moving_model(plugin_so, r=0.25, h=0.50, m=1.0):
    ixx = iyy = (1.0/12.0) * m * (3.0*(r*r) + h*h)
    izz = 0.5 * m * (r*r)
    name = plugin_so.split(".so")[0]
    return f"""
  <model name='{name}'>
    <static>1</static>
    <pose>0 0 0 0 0 0</pose>
    <link name='link'>
      <inertial><mass>{m}</mass>
        <inertia>
          <ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
      </collision>
      <visual name='visual'>
        <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
        <material><script>
          <name>Gazebo/Red</name>
          <uri>file://media/materials/scripts/gazebo.material</uri>
        </script></material>
      </visual>
    </link>
    <plugin name='{name}' filename='{plugin_so}'/>
  </model>
"""

# ------------------------ Path utilities ------------------------

def _dedupe(xs, ys):
    path = [(xs[0], ys[0])]
    for i in range(1, len(xs)):
        if (xs[i], ys[i]) != path[-1]:
            path.append((xs[i], ys[i]))
    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    return xs, ys

def _append_point(xs, ys, x, y):
    if not xs or (xs[-1], ys[-1]) != (x, y):
        xs.append(x); ys.append(y)

def _smoothstep01(u):
    return u*u*(3 - 2*u)

def _lane_centers(vmin, vmax, n):
    return [ (vmin + vmax)/2.0 ] if n <= 1 else list(np.linspace(vmin, vmax, n))

def row_path(y, x_min, x_max, passes):
    xs, ys = [], []
    for _ in range(passes):
        xs += [x_min, x_max, x_min]
        ys += [y,     y,     y]
    return _dedupe(xs, ys)

def col_path(x, y_min, y_max, passes):
    xs, ys = [], []
    for _ in range(passes):
        xs += [x, x, x]
        ys += [y_min, y_max, y_min]
    return _dedupe(xs, ys)

def random_path(x_min, x_max, y_min, y_max, points, bounce=False, seed=None):
    rng = random.Random(seed)
    points = max(2, points)

    if not bounce:
        xs = [rng.uniform(x_min, x_max) for _ in range(points)]
        ys = [rng.uniform(y_min, y_max) for _ in range(points)]
        xs.append(xs[0]); ys.append(ys[0])  # close loop
        return _dedupe(xs, ys)

    # bounce walk
    cx = rng.uniform(x_min, x_max)
    cy = rng.uniform(y_min, y_max)
    ang = rng.uniform(-math.pi, math.pi)
    step = max((x_max-x_min + y_max-y_min)/30.0, 0.2)
    xs, ys = [cx], [cy]
    for _ in range(points-1):
        cx += step*math.cos(ang)
        cy += step*math.sin(ang)
        bounced = False
        if cx < x_min: cx = x_min + (x_min - cx); ang = math.pi - ang; bounced = True
        if cx > x_max: cx = x_max - (cx - x_max); ang = math.pi - ang; bounced = True
        if cy < y_min: cy = y_min + (y_min - cy); ang = -ang;         bounced = True
        if cy > y_max: cy = y_max - (cy - y_max); ang = -ang;         bounced = True
        if not bounced:
            ang += rng.uniform(-0.35, 0.35)
        xs.append(cx); ys.append(cy)
    xs.append(xs[0]); ys.append(ys[0])
    return _dedupe(xs, ys)

def _lane_leg_row(y0, y1, x0, x1, samples=30):
    xs, ys = [], []
    for i in range(samples+1):
        u = i/samples
        x = x0 + u*(x1-x0)
        s = _smoothstep01(u)
        y = (1-s)*y0 + s*y1
        _append_point(xs, ys, x, y)
    return xs, ys

def _lane_leg_col(x0, x1, y0, y1, samples=30):
    xs, ys = [], []
    for i in range(samples+1):
        u = i/samples
        y = y0 + u*(y1-y0)
        s = _smoothstep01(u)
        x = (1-s)*x0 + s*x1
        _append_point(xs, ys, x, y)
    return xs, ys

def build_times(xs, ys, speed, dwell=0.25, speed_min=None, speed_max=None, rng=None):
    if rng is None: rng = random.Random()
    ts = [0.0]
    for i in range(len(xs)-1):
        d = math.hypot(xs[i+1]-xs[i], ys[i+1]-ys[i])
        seg_speed = speed
        if (speed_min is not None) and (speed_max is not None) and (speed_max > 0):
            seg_speed = rng.uniform(speed_min, speed_max)
        seg_speed = max(1e-6, seg_speed)
        ts.append(ts[-1] + d/seg_speed)
    ts[-1] += dwell
    return ts

# ------------------------ Main ------------------------

def main():
    ap = argparse.ArgumentParser()
    # files/dirs
    ap.add_argument('--save_dir', default='./worlds')
    ap.add_argument('--plugins_dir', default='./plugins')  # default to existing 'plugins'
    ap.add_argument('--base_world', default='base_boilerplate.world')  # contains TOKEN
    # bounds & lanes
    ap.add_argument('--rows', type=int, default=6)
    ap.add_argument('--cols', type=int, default=6)
    ap.add_argument('--x_min', type=float, default=-9.0)
    ap.add_argument('--x_max', type=float, default= 9.0)
    ap.add_argument('--y_min', type=float, default=-8.0)
    ap.add_argument('--y_max', type=float, default= 8.0)
    # movers & kinematics
    ap.add_argument('--count', type=int, default=36)
    ap.add_argument('--passes', type=int, default=4)
    ap.add_argument('--speed', type=float, default=1.2)
    ap.add_argument('--speed_min', type=float, default=None)
    ap.add_argument('--speed_max', type=float, default=None)
    ap.add_argument('--dwell', type=float, default=0.25)
    ap.add_argument('--stagger_phase', action='store_true')
    ap.add_argument('--seed', type=int, default=11)
    # movement modes
    ap.add_argument('--movement', choices=['controlled','random','traffic'], default='controlled')
    ap.add_argument('--axis', choices=['row','column'], default='row')  # used by controlled/traffic
    # random mode extras
    ap.add_argument('--rand_points', type=int, default=10)
    ap.add_argument('--rand_bounce', action='store_true')
    # traffic (lane-change) extras
    ap.add_argument('--lane_change_prob', type=float, default=0.35)
    ap.add_argument('--traffic_samples', type=int, default=40)
    args = ap.parse_args()

    rng = random.Random(args.seed)
    np.random.seed(args.seed)

    os.makedirs(args.plugins_dir, exist_ok=True)
    os.makedirs(args.save_dir, exist_ok=True)
    build = os.path.join(args.plugins_dir, 'build')
    os.makedirs(build, exist_ok=True)

    y_lanes = _lane_centers(args.y_min, args.y_max, max(1, args.rows))
    x_lanes = _lane_centers(args.x_min, args.x_max, max(1, args.cols))

    names = []
    for i in range(args.count):
        name = f"obs_{i}"

        # -------- path synthesis --------
        if args.movement == 'random':
            xs, ys = random_path(
                args.x_min, args.x_max, args.y_min, args.y_max,
                points=args.rand_points, bounce=args.rand_bounce,
                seed=args.seed + i
            )
            if args.stagger_phase and (i % 2 == 1):
                xs = xs[::-1]; ys = ys[::-1]

        elif args.movement == 'controlled':
            if args.axis == 'column':
                x0 = float(x_lanes[i % len(x_lanes)])
                xs, ys = col_path(x0, args.y_min, args.y_max, args.passes)
                if args.stagger_phase and (i % 2 == 1):
                    xs = xs[::-1]; ys = ys[::-1]
            else:
                y0 = float(y_lanes[i % len(y_lanes)])
                xs, ys = row_path(y0, args.x_min, args.x_max, args.passes)
                if args.stagger_phase and (i % 2 == 0):
                    xs = xs[::-1]; ys = ys[::-1]

        else:  # traffic with lane changes
            xs, ys = [], []
            if args.axis == 'column':
                # move along y; change X lanes occasionally
                lane = i % len(x_lanes)
                x = float(x_lanes[lane])
                yB, yT = args.y_min, args.y_max
                up = (i % 2 == 0)
                for _ in range(args.passes*2):  # two legs per pass
                    y0, y1 = (yB, yT) if up else (yT, yB)
                    change = (rng.random() < args.lane_change_prob) and (len(x_lanes) > 1)
                    if change:
                        candidates = []
                        if lane-1 >= 0: candidates.append(lane-1)
                        if lane+1 < len(x_lanes): candidates.append(lane+1)
                        tgt = rng.choice(candidates)
                        x_tgt = float(x_lanes[tgt])
                        # split leg into approach / change / finish
                        yA = y0 + 0.33*(y1-y0)
                        yC = y0 + 0.66*(y1-y0)
                        _xs, _ys = _lane_leg_col(x, x, y0, yA, args.traffic_samples//3);  [ _append_point(xs,ys,X,Y) for X,Y in zip(_xs,_ys) ]
                        _xs, _ys = _lane_leg_col(x, x_tgt, yA, yC, args.traffic_samples//2); [ _append_point(xs,ys,X,Y) for X,Y in zip(_xs,_ys) ]
                        _xs, _ys = _lane_leg_col(x_tgt, x_tgt, yC, y1, args.traffic_samples//3); [ _append_point(xs,ys,X,Y) for X,Y in zip(_xs,_ys) ]
                        lane = tgt; x = x_tgt
                    else:
                        _xs, _ys = _lane_leg_col(x, x, y0, y1, max(8, args.traffic_samples)); [ _append_point(xs,ys,X,Y) for X,Y in zip(_xs,_ys) ]
                    up = not up
            else:
                # move along x; change Y lanes occasionally
                lane = i % len(y_lanes)
                y = float(y_lanes[lane])
                xL, xR = args.x_min, args.x_max
                right = (i % 2 == 0)
                for _ in range(args.passes*2):
                    x0, x1 = (xL, xR) if right else (xR, xL)
                    change = (rng.random() < args.lane_change_prob) and (len(y_lanes) > 1)
                    if change:
                        candidates = []
                        if lane-1 >= 0: candidates.append(lane-1)
                        if lane+1 < len(y_lanes): candidates.append(lane+1)
                        tgt = rng.choice(candidates)
                        y_tgt = float(y_lanes[tgt])
                        xA = x0 + 0.33*(x1-x0)
                        xC = x0 + 0.66*(x1-x0)
                        _xs, _ys = _lane_leg_row(y, y, x0, xA, args.traffic_samples//3);  [ _append_point(xs,ys,X,Y) for X,Y in zip(_xs,_ys) ]
                        _xs, _ys = _lane_leg_row(y, y_tgt, xA, xC, args.traffic_samples//2); [ _append_point(xs,ys,X,Y) for X,Y in zip(_xs,_ys) ]
                        _xs, _ys = _lane_leg_row(y_tgt, y_tgt, xC, x1, args.traffic_samples//3); [ _append_point(xs,ys,X,Y) for X,Y in zip(_xs,_ys) ]
                        lane = tgt; y = y_tgt
                    else:
                        _xs, _ys = _lane_leg_row(y, y, x0, x1, max(8, args.traffic_samples)); [ _append_point(xs,ys,X,Y) for X,Y in zip(_xs,_ys) ]
                    right = not right

        # -------- timing --------
        ts = build_times(
            xs, ys,
            speed=args.speed,
            dwell=args.dwell,
            speed_min=args.speed_min,
            speed_max=args.speed_max,
            rng=rng
        )

        # -------- emit C++ plugin --------
        cc = make_head(name, ts[-1])
        for t, x, y in zip(ts, xs, ys):
            cc += make_waypoint(t, x, y)
        cc += make_tail(name)
        with open(os.path.join(args.plugins_dir, f"{name}.cc"), "w") as f:
            f.write(cc)
        names.append(name)

    # ------------------------ build & assemble world ------------------------
    with open(os.path.join(args.plugins_dir, "CMakeLists.txt"), "w") as f:
        f.write(make_CMakeLists(names))

    cwd = os.getcwd()
    build_dir = os.path.join(args.plugins_dir, "build")
    os.chdir(build_dir)
    subprocess.run(["cmake", ".."], check=True)
    subprocess.run(["make", "-j", str(os.cpu_count() or 2)], check=True)
    os.chdir(cwd)

    with open(args.base_world, "r") as f:
        s = f.read()
    if "TOKEN" not in s:
        raise RuntimeError("base_world must contain 'TOKEN'")
    p1, p2 = s.split("TOKEN", 1)

    sos = sorted([f for f in os.listdir(build_dir) if f.endswith(".so")])
    mid = "".join([make_moving_model(so) for so in sos])

    out_world = os.path.join(args.save_dir, "world_generated.world")
    with open(out_world, "w") as f:
        f.write(p1 + mid + p2)

    print("\nWrote", out_world)
    print(f"export GAZEBO_PLUGIN_PATH={build_dir}:$GAZEBO_PLUGIN_PATH")
    print(f"gzserver --verbose {out_world}")

if __name__ == "__main__":
    main()
