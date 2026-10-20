# DynaBARN
Repository for [DynaBARN](https://cs.gmu.edu/~xxiao2/Research/DynaBARN/DynaBARN.html). This code can be used to generate various DynaBARN environments of obstacles with different motion profiles. The 60 premade DynaBARN environments are at https://tufts.box.com/s/0dsoen9yno1qrnpj0l75eni7kmjdcx7c
The Gazebo world files are under the folder titled 'DynaBARN_worlds_60'. The plugin files are under 'all_cylinder_plugins'.
To simulate the 60 premade DynaBARN worlds, export the location of the plugins to the plugin path variable using 
```
export GAZEBO_PLUGIN_PATH=/path/to/plugins/testplugin/all_cylinder_plugins
```
Then simulate the environment using 
```
gazebo world_{world number}.world
```
## Example Worlds

![combined_big](https://user-images.githubusercontent.com/46573631/186241202-04005054-4157-46e2-b602-7565d62e8ba4.gif)


<!-- Easy World:


https://user-images.githubusercontent.com/46573631/185812166-b76cddd9-d2ca-4fd8-9a64-c41cc706c32e.mp4


Medium World:


https://user-images.githubusercontent.com/46573631/185812169-6a711fa1-8616-484b-a1df-7e8b1ccbf1d3.mp4


Hard World:


https://user-images.githubusercontent.com/46573631/185812175-7e8c0e44-5bd3-4981-8738-a84f4accdd21.mp4

 -->


## Requirements
* Python 2
* NumPy
* Matplotlib
* SciPy
## Using this repository
After cloning this repository onto your computer, create two folders called "plugins" and "worlds" in the same directory. 
There are a few arguments for creating the trajectories and worlds:
* --save_dir and --plugin_dir are the directories for the world files and plugins, respectively
* --min_object and --max_object are the minimum and maximum number of obstacles in each world
* --n_worlds are the number of worlds to generate 
* --min_speed and --max_speed are the minimum and maximum speeds between each waypoint in the trajectories for each obstacle
* --min_std and --max_std are the minimum and maximum standard deviations in speed between each waypoint
* --difficulty is the difficulty to label each trajectory, but it is not necessary


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


use this command to run the example 
gazebo --verbose /ws/src/DynaBARN/worlds/world_0.world

should enter through 
docker exec -it ros2-humble-gz /entrypoint.sh bash



Controlled lanes

Left↔right rows:

python3 generate_grid_plugins_gz11.py \
  --movement controlled --axis row \
  --rows 6 --x_min -9 --x_max 9 --y_min -8 --y_max 8 \
  --passes 3 --speed 1.2 --count 36 --stagger_phase


Front↔back columns:

python3 generate_grid_plugins_gz11.py \
  --movement controlled --axis column \
  --cols 6 --x_min -9 --x_max 9 --y_min -8 --y_max 8 \
  --passes 3 --speed 1.2 --count 36 --stagger_phase

Random motion

Uniform random waypoints (closed loop), constant speed:

python3 generate_grid_plugins_gz11.py \
  --movement random --rand_bounce --rand_points 30 \
  --speed_min 0.6 --speed_max 1.8 \
  --count 40 --stagger_phase

Random “bounce-walk” (reflects at borders), random speed per segment:

python3 generate_grid_plugins_gz11.py \
  --movement random --rand_bounce --rand_points 30 \
  --speed_min 0.6 --speed_max 1.8 \
  --count 40 --stagger_phase


per-robot constant random:

python3 generate_grid_plugins_gz11.py \
  --movement controlled --axis column \
  --cols 6 --x_min -9 --x_max 9 --y_min -8 --y_max 8 \
  --passes 3 --count 36 --stagger_phase \
  --speed_min 0.6 --speed_max 1.8 --speed_mode agent


# TODO: Traffic generation is not complete

Row-based traffic (cars moving left→right, right→left, sometimes changing to a neighboring row):

python3 generate_dynabarn_traffic.py \
  --movement traffic --axis row \
  --rows 6 --x_min -9 --x_max 9 --y_min -8 --y_max 8 \
  --passes 4 --count 36 --speed 1.3 \
  --lane_change_prob 0.4 --traffic_samples 50 --stagger_phase

Column-based traffic (cars moving front→back, back→front, sometimes changing to a neighboring column):

python3 generate_dynabarn_traffic.py \
  --movement traffic --axis column \
  --cols 6 --x_min -9 --x_max 9 --y_min -8 --y_max 8 \
  --passes 4 --count 36 --speed 1.3 \
  --lane_change_prob 0.4 --traffic_samples 50 --stagger_phase
