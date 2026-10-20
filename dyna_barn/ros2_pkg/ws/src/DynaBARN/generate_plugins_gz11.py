#!/usr/bin/env python3
import os
import random
import subprocess
import argparse
import numpy as np

from polynomial_fit import get_random_traj_points
from polynomial_fit import distance, calc_time


# =============================================================================
# DynaBARN world generator — quick reference
#
# This script builds animated obstacle plugins (Gazebo Classic 11) and emits
# .world files that include those plugins. Key CLI flags and what they do:
#
#   --save_dir <dir>       Where generated .world files are written (e.g. ./worlds).
#   --plugins_dir <dir>    Where plugin sources are generated/compiled (./plugins).
#                          Runtime must have:  export GAZEBO_PLUGIN_PATH=<plugins_dir>/build:$GAZEBO_PLUGIN_PATH
#   --seed <int>           RNG seed for reproducibility.
#
#   --min_object <int>     Minimum number of moving obstacles per world.
#   --max_object <int>     Maximum number of moving obstacles per world.
#                          ↑ Increase these to “populate” the environment (e.g., 12–20).
#
#   --n_worlds <int>       NOTE: In the current code this DOES NOT control how many
#                          .world files are written. The script always writes 20 worlds
#                          via:  for i in range(20). Change that loop or wire args.n_worlds in.
#
#   --min_speed <m/s>      Lower bound for the randomly chosen average speed of each obstacle.
#   --max_speed <m/s>      Upper bound for the randomly chosen average speed.
#                          These bounds feed into calc_time(...) to schedule keyframes.
#
#   --min_std <m/s>        Lower bound for speed jitter (std dev) along a path.
#   --max_std <m/s>        Upper bound for speed jitter. Set both to 0.0 for constant speed.
#
#   --min_order <int>      Minimum polynomial order for random trajectory generation.
#   --max_order <int>      Maximum polynomial order. Higher can be more curvy/complex.
#
#   --base_world <path>    Boilerplate SDF with a literal 'TOKEN' where generated models go.
#
# -----------------------------------------------------------------------------
# Make obstacles move AS FAST AS POSSIBLE (and densely):
#
#   1) Force the max speed and remove jitter:
#        --min_speed 3.0 --max_speed 3.0 --min_std 0.0 --max_std 0.0
#      (Pick 3.0 m/s or higher if your scene can handle it.)
#
#   2) Increase obstacle count per world:
#        --min_object 12 --max_object 20
#
#   3) (Optional) Global speed-up: add a "speed_scale" to compress keyframe times
#      if you modify the code accordingly:
#        times = [t / args.speed_scale for t in times]  # speed_scale > 1.0 → faster
#      Then run with e.g.:
#        --speed_scale 2.0
#
# -----------------------------------------------------------------------------
# Obstacle SIZE notes (so TB3 doesn’t look tiny):
#
# * TB3 Burger is ~0.14 m wide (wheel-to-wheel ~0.16 m). The default cylinders here
#   were radius=0.5 m, height=1.0 m (huge). In make_moving_model(), use smaller values:
#       radius ~ 0.10 m, height ~ 0.20 m
#   and update inertia accordingly (solid cylinder):
#       Ixx = Iyy = (1/12) * m * (3 r^2 + h^2),  Izz = 0.5 * m * r^2
#
# * Walls in boilerplate.world are ~20 m long / 1 m tall. If you want a tighter arena,
#   shrink those <box><size> values (e.g., 4 x 0.25 x 0.5) and adjust <pose> Z (half height).
#
# -----------------------------------------------------------------------------
# Example: dense & fast, small obstacles
#
#   python3 generate_plugins_gz11.py \
#     --save_dir ./worlds \
#     --plugins_dir ./plugins \
#     --seed 11 \
#     --min_object 12 \
#     --max_object 20 \
#     --n_worlds 100 \
#     --min_speed 3.0 \
#     --max_speed 3.0 \
#     --min_std 0.0 \
#     --max_std 0.0 \
#     --min_order 3 \
#     --max_order 6 \
#     --base_world ./boilerplate.world
#
# (If you added --speed_scale, tack on e.g. --speed_scale 2.0)
#
# -----------------------------------------------------------------------------
# Performance & physics:
# * Obstacles are <static> with PoseAnimation; they move and collide but aren’t
#   dynamically simulated (no forces/impulses). This is fine for moving hazards.
# * Very high counts/speeds can stress gzclient; gzserver physics still runs.
# * Ensure GAZEBO_PLUGIN_PATH includes ./plugins/build at runtime.
# =============================================================================



# ---------- Codegen helpers (Gazebo Classic 11) ----------

def make_head(name, time_interval):
    return "\n\
#include <gazebo/gazebo.hh>\n\
#include <gazebo/physics/physics.hh>\n\
#include <gazebo/common/common.hh>\n\
#include <ignition/math/Vector3.hh>\n\
#include <ignition/math/Quaternion.hh>\n\
#include <stdio.h>\n\
\n\
namespace gazebo\n\
{\n\
  class %s : public ModelPlugin\n\
  {\n\
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)\n\
    {\n\
      // Store the pointer to the model\n\
      this->model = _parent;\n\
\n\
      // Create the animation: name, duration (s), loop = true\n\
      gazebo::common::PoseAnimationPtr anim(\n\
          new gazebo::common::PoseAnimation(\"%s\", %.2f, true));\n\
\n\
      gazebo::common::PoseKeyFrame *key;\n\
" % (name, name, time_interval)


def make_tail(name):
    return "\n\
      // set the animation\n\
      this->model->SetAnimation(anim);\n\
    }\n\
\n\
    // Pointer to the model\n\
    private: physics::ModelPtr model;\n\
\n\
    // Pointer to the update event connection (unused here)\n\
    private: event::ConnectionPtr updateConnection;\n\
  };\n\
\n\
  // Register this plugin with the simulator\n\
  GZ_REGISTER_MODEL_PLUGIN(%s)\n\
}\n\
" % (name)


def make_waypoint(t, x, y):
    return "\n\
      key = anim->CreateKeyFrame(%.2f);\n\
      key->Translation(ignition::math::Vector3d(%.2f, %.2f, 0));\n\
      key->Rotation(ignition::math::Quaterniond(0, 0, 0));\n\
" % (t, x, y)


def make_CMakeLists(name_list):
    s = "\n\
cmake_minimum_required(VERSION 3.10)\n\
project(animated_box)\n\
\n\
find_package(gazebo REQUIRED)\n\
\n\
set(CMAKE_CXX_STANDARD 14)\n\
set(CMAKE_CXX_STANDARD_REQUIRED ON)\n\
\n\
# Gazebo compile flags and include/link dirs\n\
add_definitions(${GAZEBO_CXX_FLAGS})\n\
include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})\n\
link_directories(${GAZEBO_LIBRARY_DIRS})\n\
\n\
# Ensure we're really on Gazebo 11+\n\
if(DEFINED gazebo_VERSION)\n\
  if(gazebo_VERSION VERSION_LESS 11.0)\n\
    message(FATAL_ERROR \"This project requires Gazebo Classic 11 or newer. Found ${gazebo_VERSION}.\")\n\
  endif()\n\
endif()\n\
\n\
"
    for n in name_list:
        s += "\n\
add_library(%s SHARED %s.cc)\n\
target_link_libraries(%s ${GAZEBO_LIBRARIES})\n\
" % (n, n, n)
    return s


def make_moving_model(plugin_name):
    name = plugin_name.split('.so')[0]
    # to change the dimention of object we can use this 
    r = 0.25   # was 0.5000
    h = 0.50   # was 1.0

    # Recompute inertia for a solid cylinder (about its center):
    # Ixx = Iyy = (1/12) * m * (3r^2 + h^2), Izz = 0.5 * m * r^2
    m = 1.0
    ixx = iyy = (1.0/12.0) * m * (3.0*(r*r) + h*h)
    izz = 0.5 * m * (r*r)

    return f"""
  <model name='{name}'>
    <static>1</static>
    <pose frame=''>-0.225000 0.075000 0.000000 0.000000 0.000000 0.000000</pose>
    <link name='link'>
      <inertial>
        <mass>{m}</mass>
        <inertia>
          <ixx>{ixx}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy}</iyy><iyz>0</iyz>
          <izz>{izz}</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <cylinder>
            <radius>{r}</radius>
            <length>{h}</length>
          </cylinder>
        </geometry>
        <surface>
          <contact><ode/></contact>
          <bounce/>
          <friction><torsional><ode/></torsional><ode/></friction>
        </surface>
      </collision>
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>{r}</radius>
            <length>{h}</length>
          </cylinder>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Red</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
      <gravity>1</gravity>
    </link>
    <plugin name='{name}' filename='{plugin_name}'/>
  </model>
"""


# ---------- Trajectory synthesis ----------

def create_worlds(num_worlds, min_order, max_order, min_objects, max_objects,
                  min_speed, max_speed, min_std, max_std):
    """Return [(t_i, x_i, y_i), ...] for one obstacle animation."""
    # Pick one random trajectory (uniform among valid ones)
    n = random.randint(min_objects, max_objects)
    # Generate until we get a valid polynomial path once; last one wins (as in your original)
    for _ in range(n):
        while True:
            try:
                order = random.randint(min_order, max_order)
                x, y, points = get_random_traj_points(order=order)
            except Exception:
                continue
            else:
                break

    avg_speed = round(random.uniform(min_speed, max_speed), 1)
    std = round(random.uniform(min_std, max_std), 2)

    times = calc_time(
        x, y, points,
        avg_speed=avg_speed,
        min_speed=min_speed,
        max_speed=max_speed,
        std=std
    )
    return list(zip(times, x, y))


# ---------- Main ----------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--save_dir', type=str, default="/worlds")
    parser.add_argument('--seed', type=int, default=11)
    parser.add_argument('--min_object', type=int, default=2)
    parser.add_argument('--max_object', type=int, default=6)
    parser.add_argument('--n_worlds', type=int, default=100)
    parser.add_argument('--min_speed', type=float, required=True)
    parser.add_argument('--max_speed', type=float, required=True)
    parser.add_argument('--min_std', type=float, required=True)
    parser.add_argument('--max_std', type=float, required=True)
    parser.add_argument('--min_order', type=int, required=True)
    parser.add_argument('--max_order', type=int, required=True)
    parser.add_argument('--plugins_dir', type=str, default="plugins")
    parser.add_argument('--base_world', type=str, default="base_boilerplate.world")
    args = parser.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)

    # 1) Generate plugin source files
    os.makedirs(args.plugins_dir, exist_ok=True)
    build_dir = os.path.join(args.plugins_dir, "build")
    os.makedirs(build_dir, exist_ok=True)
    os.makedirs(args.save_dir, exist_ok=True)

    name_list = []
    for i in range(200):
        name = f"obs_{i}"
        waypoints = create_worlds(
            args.n_worlds,
            args.min_order, args.max_order,
            args.min_object, args.max_object,
            args.min_speed, args.max_speed,
            args.min_std, args.max_std
        )
        print("name", name)
        print("waypoints ({} pts)".format(len(waypoints)))
        name_list.append(name)

        # C++ file contents
        fs = make_head(name, waypoints[-1][0] - waypoints[0][0])
        for wp in waypoints:
            fs += make_waypoint(*wp)
        fs += make_tail(name)

        with open(os.path.join(args.plugins_dir, name + ".cc"), "w") as f:
            f.write(fs)

    # CMakeLists.txt
    cmake_fs = make_CMakeLists(name_list)
    with open(os.path.join(args.plugins_dir, "CMakeLists.txt"), "w") as f:
        f.write(cmake_fs)

    # 2) Build plugins
    wd = os.getcwd()
    os.chdir(build_dir)
    print("Configuring CMake in", build_dir)
    subprocess.run(["cmake", ".."], check=True)
    print("Building (parallel)...")
    jobs = str(os.cpu_count() or 2)
    subprocess.run(["make", f"-j{jobs}"], check=True)
    os.chdir(wd)

    # 3) Create .world files that load the compiled .so
    # Expect base world file with 'TOKEN' placeholder
    with open(args.base_world, "r") as f:
        ss = f.read()
    if "TOKEN" not in ss:
        raise RuntimeError("base_world file must contain 'TOKEN' placeholder")

    part1, part2 = ss.split("TOKEN", 1)

    # All built plugin .so’s
    plugins = sorted([f for f in os.listdir(build_dir) if f.endswith(".so")])
    print("Built plugins:", plugins[:5], "..." if len(plugins) > 5 else "")

    for i in range(20):
        mid = ""
        n_obs = np.random.randint(args.min_object, args.max_object)
        obs = random.sample(plugins, n_obs)
        for plugin_so in obs:
            mid += make_moving_model(plugin_so)

        out_world = os.path.join(args.save_dir, f"world_{i}.world")
        with open(out_world, "w") as f:
            f.write(part1 + mid + part2)
        print("Wrote", out_world)

    print("\nDone. Remember to set GAZEBO_PLUGIN_PATH to your build dir when running worlds:")
    print(f"  export GAZEBO_PLUGIN_PATH={build_dir}:$GAZEBO_PLUGIN_PATH")
    print("Then run, e.g.:")
    print(f"  gzserver --verbose {os.path.join(args.save_dir, 'world_0.world')}")

if __name__ == "__main__":
    main()


# python3 generate_plugins_gz11.py \
#   --save_dir ./worlds \
#   --plugins_dir ./plugins \
#   --seed 11 \
#   --min_object 2 \
#   --max_object 6 \
#   --n_worlds 100 \
#   --min_speed 0.5 \
#   --max_speed 1.5 \
#   --min_std 0.05 \
#   --max_std 0.20 \
#   --min_order 3 \
#   --max_order 6