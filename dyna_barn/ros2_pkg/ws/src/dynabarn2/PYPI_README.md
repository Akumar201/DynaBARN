# dynabarn2-worlds

World generators and difficulty checks for the DynaBARN II dynamic navigation benchmark. Needs only
Python and NumPy.

```bash
dynabarn2-generate --save_dir worlds --per_cell 1 --families crowd --levels easy
```

This writes Gazebo Harmonic `.sdf` worlds and a `manifest.json` with each world's difficulty. To run
the worlds you need the `dynabarn2` ROS 2 package, which provides the obstacle plugin, or its Docker
image. See the project README.
