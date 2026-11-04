#!/usr/bin/env python3
"""Convert the 60 original DynaBARN (Gazebo Classic) worlds to Gazebo Harmonic.

Reads which obstacles each original world uses (and their cylinder size) from the
.world files, takes their exact keyframes from data/dynabarn_classic_trajectories.json
(see tools/classic_extract), and writes one ObstacleMover-based .sdf per world.
Motion uses spline interpolation, matching Classic's PoseAnimation.

Example:
  convert_dynabarn_classic.py --worlds_dir DynaBARN/DynaBARN_worlds_60 \
    --trajectories data/dynabarn_classic_trajectories.json --save_dir worlds/dynabarn
"""
import argparse
import json
import os
import re
import xml.etree.ElementTree as ET

import world_sdf


def obstacles_in(world_file):
    """[(model name, plugin file, radius, length), ...] for the animated models of a Classic world."""
    # The Classic boilerplate has malformed wall poses (`<pose frame=>`), so only parse model blocks
    # that carry a plugin.
    text = open(world_file).read()
    found = []
    for block in re.findall(r"<model name='[^']*'>.*?</model>", text, re.S):
        model = ET.fromstring(block)
        plugin = model.find("plugin")
        if plugin is None:
            continue
        cyl = model.find("link/collision/geometry/cylinder")
        found.append((model.get("name"), plugin.get("filename"),
                      float(cyl.findtext("radius")), float(cyl.findtext("length"))))
    return found


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--worlds_dir", required=True, help="original DynaBARN_worlds_60 folder")
    ap.add_argument("--trajectories", required=True, help="dynabarn_classic_trajectories.json")
    ap.add_argument("--save_dir", default="./worlds/dynabarn")
    args = ap.parse_args()

    trajectories = json.load(open(args.trajectories))["trajectories"]
    os.makedirs(args.save_dir, exist_ok=True)

    worlds = sorted((f for f in os.listdir(args.worlds_dir) if f.endswith(".world")),
                    key=lambda f: int(re.search(r"\d+", f).group()))
    for fname in worlds:
        models = []
        for name, plugin, radius, length in obstacles_in(os.path.join(args.worlds_dir, fname)):
            traj = trajectories[plugin]
            assert abs(traj["waypoints"][-1][0] - traj["duration"]) < 1e-6, plugin
            models.append(world_sdf.obstacle(name, [tuple(w) for w in traj["waypoints"]],
                                             radius=radius, height=length))
        stem = os.path.splitext(fname)[0]
        out = os.path.join(args.save_dir, f"{stem}.sdf")
        with open(out, "w") as f:
            f.write(world_sdf.world(f"dynabarn_{stem}", models))
        print(f"Wrote {out} ({len(models)} obstacles)")


if __name__ == "__main__":
    main()
