# Extracting the original DynaBARN trajectories

The 60 original DynaBARN worlds reference 600 Gazebo 9 plugins (`all_cylinder_plugins/*.so`,
from the DynaBARN Box folder) whose waypoints exist only inside the compiled binaries.
This tool loads each plugin, runs its `Load()`, and intercepts the `PoseAnimation` calls to
log the exact keyframes. The result is committed as `data/dynabarn_classic_trajectories.json`,
so this only needs re-running to verify that file.

```bash
docker run --rm -v $PWD:/x -v /path/to/all_cylinder_plugins:/p:ro gazebo:libgazebo9-bionic bash -c '
  apt-get update && apt-get install -y g++ pkg-config libgazebo9-dev
  cd /x && g++ -std=c++17 -O1 -rdynamic main.cc hooks.cc -o extract $(pkg-config --cflags --libs gazebo) -ldl
  ./extract /p/*.so > keyframes.txt'
```

`keyframes.txt` lists, per plugin: `PLUGIN <path>`, `ANIM <name> <length> <loop>`, then
`KEY <t>` / `POS <x> <y> <z>` / `ROT <w> <x> <y> <z>` per keyframe, and `END`.

## Checking the converted worlds against Classic

`SAMPLE_DT=0.001 ./extract <plugins of one world>` also prints Gazebo 9's own `PoseAnimation`
interpolation (`SAMPLE <t> <x> <y>`). Record the same world in Harmonic, unthrottled (in `gz sim`,
`-z` is an update rate in Hz, not a speed multiplier):

```bash
gz topic -e -t /world/default/dynamic_pose/info > harmonic.txt &
gz sim -s -r -z 100000 --iterations 70000 worlds/dynabarn/world_45.sdf
python3 compare_motion.py classic.txt harmonic.txt
```

`data/classic_vs_harmonic_motion.csv` holds the result for all 60 worlds. A `err_max_m` of metres is
a single sample at a loop point, where the obstacle jumps back to its start one step apart in the
two simulators (the error equals the jump distance); it is not a path difference.
