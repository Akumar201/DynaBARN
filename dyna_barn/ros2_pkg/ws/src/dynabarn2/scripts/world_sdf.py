"""SDF builders for DynaBARN II worlds (Gazebo Harmonic).

Obstacles are moved by the DynaBarnObstacleMover system (see src/ObstacleMover.cc),
so a world is a single self-contained .sdf file: no per-obstacle plugins to compile.
"""

# BARN and Clearpath's bringup both address the world as "default".
WORLD_NAME = "default"
ROBOT_NAME = "robot"

WALL_COLOR = "0 0 1 1"
OBSTACLE_COLOR = "1 0 0 1"


def _material(rgba):
    return (f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>"
            f"<specular>0.1 0.1 0.1 1</specular></material>")


# Publishes on /robot/touched whenever this model's contact sensor touches the robot.
TOUCH_SENSOR = ('<sensor name="sensor_contact" type="contact">'
                '<contact><collision>collision</collision></contact></sensor>')
TOUCH_PLUGIN = f"""
      <plugin filename="gz-sim-touchplugin-system" name="gz::sim::systems::TouchPlugin">
        <target>{ROBOT_NAME}</target>
        <namespace>{ROBOT_NAME}</namespace>
        <time>0.001</time>
        <enabled>true</enabled>
      </plugin>"""


def wall(name, x, y, sx, sy, h=1.0):
    geom = f"<geometry><box><size>{sx} {sy} {h}</size></box></geometry>"
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {h / 2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">{geom}</collision>
        <visual name="visual">{geom}{_material(WALL_COLOR)}</visual>
        {TOUCH_SENSOR}
      </link>{TOUCH_PLUGIN}
    </model>"""


def default_walls():
    """Same three walls as the original DynaBARN boilerplate (open on the +x side)."""
    return (wall("back_wall", -10, 0, 0.5, 20)
            + wall("top_wall", 0, 10, 20, 0.5)
            + wall("bottom_wall", 0, -10, 20, 0.5))


def _shape(kind, radius, height):
    if kind == "box":  # square that fits inside the disc of this radius
        side = radius * 2 ** 0.5
        return f"<geometry><box><size>{side:.4f} {side:.4f} {height}</size></box></geometry>"
    return f"<geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>"


def static_obstacle(name, x, y, radius, height=1.0):
    """Fixed cylinder, for clutter between the moving obstacles."""
    geom = _shape("cylinder", radius, height)
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {height / 2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">{geom}</collision>
        <visual name="visual">{geom}{_material(WALL_COLOR)}</visual>
        {TOUCH_SENSOR}
      </link>{TOUCH_PLUGIN}
    </model>"""


def obstacle(name, waypoints, radius=0.5, height=1.0, mass=10.0,
             interpolation="spline", loop=True, shape="cylinder"):
    """Obstacle following waypoints [(t, x, y), ...]; t in seconds, starting at 0."""
    z = height / 2.0
    ixx = (1.0 / 12.0) * mass * (3.0 * radius ** 2 + height ** 2)
    izz = 0.5 * mass * radius ** 2
    geom = _shape(shape, radius, height)
    t0, x0, y0 = waypoints[0]
    wps = "\n".join(f"        <waypoint>{t:.4f} {x:.4f} {y:.4f} {z:.4f}</waypoint>"
                    for t, x, y in waypoints)
    return f"""
    <model name="{name}">
      <pose>{x0:.4f} {y0:.4f} {z:.4f} 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>{mass}</mass>
          <inertia><ixx>{ixx:.6f}</ixx><iyy>{ixx:.6f}</iyy><izz>{izz:.6f}</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="collision">{geom}</collision>
        <visual name="visual">{geom}{_material(OBSTACLE_COLOR)}</visual>
        {TOUCH_SENSOR}
      </link>{TOUCH_PLUGIN}
      <plugin filename="DynaBarnObstacleMover" name="dynabarn2::ObstacleMover">
        <loop>{str(loop).lower()}</loop>
        <interpolation>{interpolation}</interpolation>
{wps}
      </plugin>
    </model>"""


def world(models, walls=None, step_size=0.001):
    """Full world SDF: physics + core systems, ground, sun, walls and the given model XML."""
    if walls is None:
        walls = default_walls()
    ground = ("<geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>")
    return f"""<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="{WORLD_NAME}">
    <physics name="default_physics" type="ignored">
      <max_step_size>{step_size}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">{ground}</collision>
        <visual name="visual">{ground}{_material("0.8 0.8 0.8 1")}</visual>
      </link>
    </model>
{walls}
{"".join(models)}
  </world>
</sdf>
"""
