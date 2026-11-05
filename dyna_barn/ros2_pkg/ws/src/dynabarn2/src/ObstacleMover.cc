// ObstacleMover: moves a DynaBARN obstacle model along timed waypoints in
// Gazebo Harmonic (gz-sim 8).
//
// Replaces the per-obstacle Gazebo Classic ModelPlugins (PoseAnimation) with a
// single system configured from SDF. Interpolation matches Classic's
// PoseAnimation (gz::math::Spline with auto-calculated tangents) by default.
//
// The model stays a real (non-static) physics body: every step it is
// teleported to the trajectory pose and given the trajectory velocity, so
// contacts with the robot behave like a moving solid obstacle.
//
// Usage (inside a <model>):
//   <plugin filename="DynaBarnObstacleMover" name="dynabarn2::ObstacleMover">
//     <loop>true</loop>                  <!-- optional, default true -->
//     <duration>90.25</duration>         <!-- optional, default last waypoint time -->
//     <interpolation>spline</interpolation>  <!-- spline (default) | linear -->
//     <waypoint>0.0  -9 -8 0.5</waypoint>    <!-- time x y z -->
//     <waypoint>15.0  9 -8 0.5</waypoint>
//   </plugin>
//
// Reactive mode: set DYNABARN_REACTIVE=yield or block in Gazebo's environment and obstacles respond
// to the robot, moving away from it or towards it while near it, then easing back to their path.
// It uses no randomness, so the same robot motion always produces the same obstacle motion.
// DYNABARN_REACTIVE_SPEED (m/s) and DYNABARN_REACTIVE_RADIUS (m) override the defaults below.
//
// Synchronised start: set DYNABARN_SYNC_START=true and obstacles wait at their first waypoint until
// the robot first moves. Otherwise they start at sim time 0, and the robot's variable start-up delay
// puts them somewhere different in every trial.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Spline.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

namespace dynabarn2
{
constexpr double kInfluenceRadius = 2.5;  // m, default for how close the robot must be to respond
constexpr double kResponseSpeed = 1.0;    // m/s, default speed at zero distance from the robot
constexpr double kRecoveryTime = 2.0;     // s, time constant for easing back to the path
constexpr double kMaxOffset = 2.0;        // m, furthest an obstacle strays from its path
constexpr double kStartRadius = 0.1;      // m, the robot's move that starts the trajectory clock

struct Waypoint
{
  double time;
  gz::math::Vector3d position;
};

class ObstacleMover
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      gzerr << "ObstacleMover must be attached to a model.\n";
      return;
    }
    const std::string name = this->model.Name(_ecm);

    auto sdf = _sdf->Clone();
    if (sdf->HasElement("loop"))
      this->loop = sdf->Get<bool>("loop");
    if (sdf->HasElement("interpolation"))
      this->spline = sdf->Get<std::string>("interpolation") != "linear";

    for (auto elem = sdf->FindElement("waypoint"); elem;
         elem = elem->GetNextElement("waypoint"))
    {
      std::istringstream in(elem->Get<std::string>());
      Waypoint wp;
      if (!(in >> wp.time >> wp.position.X() >> wp.position.Y() >>
            wp.position.Z()))
      {
        gzerr << "[" << name << "] bad <waypoint> (expected 't x y z'): '"
              << elem->Get<std::string>() << "'\n";
        continue;
      }
      this->waypoints.push_back(wp);
    }

    if (this->waypoints.size() < 2)
    {
      gzerr << "[" << name << "] needs at least 2 waypoints; not moving.\n";
      this->waypoints.clear();
      return;
    }
    std::sort(this->waypoints.begin(), this->waypoints.end(),
              [](const Waypoint &a, const Waypoint &b)
              { return a.time < b.time; });

    this->duration = this->waypoints.back().time;
    if (sdf->HasElement("duration"))
      this->duration = std::max(this->duration, sdf->Get<double>("duration"));

    this->curve.AutoCalculate(true);
    for (const auto &wp : this->waypoints)
      this->curve.AddPoint(wp.position);

    if (const char *mode = std::getenv("DYNABARN_REACTIVE"))
    {
      const std::string value(mode);
      if (value == "yield")
        this->reactiveSign = 1.0;
      else if (value == "block")
        this->reactiveSign = -1.0;
      else if (!value.empty() && value != "off")
        gzerr << "Unknown DYNABARN_REACTIVE '" << value << "'; using fixed motion.\n";
    }

    if (const char *value = std::getenv("DYNABARN_SYNC_START"))
      this->syncStart = std::string(value) == "true";
    if (const char *value = std::getenv("DYNABARN_REACTIVE_SPEED"))
      this->responseSpeed = std::stod(value);
    if (const char *value = std::getenv("DYNABARN_REACTIVE_RADIUS"))
      this->influenceRadius = std::stod(value);

    // Keep the model's initial orientation; obstacles only translate.
    this->rotation = gz::sim::worldPose(_entity, _ecm).Rot();

    gzmsg << "[" << name << "] ObstacleMover: " << this->waypoints.size()
          << " waypoints, duration " << this->duration << "s, "
          << (this->spline ? "spline" : "linear")
          << (this->loop ? ", looping" : "") << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (this->waypoints.empty())
      return;

    double t = std::chrono::duration<double>(_info.simTime).count();
    const bool waiting = this->syncStart && !this->RobotHasMoved(t, _ecm);
    t = waiting ? 0.0 : t - this->startTime;
    if (this->loop && this->duration > 0.0)
      t = std::fmod(t, this->duration);

    gz::math::Vector3d pos, vel;
    this->Sample(t, pos, vel);
    if (this->reactiveSign != 0.0 && !waiting)
      this->React(_info, _ecm, pos, vel);

    this->model.SetWorldPoseCmd(_ecm, gz::math::Pose3d(pos, this->rotation));

    // Velocity commands are in the model (body) frame.
    const gz::math::Vector3d bodyVel = this->rotation.RotateVectorReverse(vel);
    const gz::math::Vector3d zero = gz::math::Vector3d::Zero;
    const auto entity = this->model.Entity();
    _ecm.SetComponentData<gz::sim::components::LinearVelocityCmd>(entity,
                                                                  bodyVel);
    _ecm.SetComponentData<gz::sim::components::AngularVelocityCmd>(entity,
                                                                   zero);
  }

private:
  gz::sim::Entity Robot(gz::sim::EntityComponentManager &_ecm)
  {
    if (this->robot == gz::sim::kNullEntity)
      this->robot = _ecm.EntityByComponents(gz::sim::components::Model(),
                                            gz::sim::components::Name("robot"));
    return this->robot;
  }

  // True once the robot has left its spawn point; records that moment as the trajectory's time zero.
  bool RobotHasMoved(double _now, gz::sim::EntityComponentManager &_ecm)
  {
    if (this->started)
      return true;
    if (this->Robot(_ecm) == gz::sim::kNullEntity)
      return false;
    gz::math::Vector3d position = gz::sim::worldPose(this->robot, _ecm).Pos();
    position.Z() = 0.0;
    if (!this->robotSeen)
    {
      this->robotSeen = true;
      this->robotSpawn = position;
    }
    if (position.Distance(this->robotSpawn) < kStartRadius)
      return false;
    this->started = true;
    this->startTime = _now;
    return true;
  }

  // Shift the path position by an offset that grows near the robot and decays away from it.
  void React(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm,
             gz::math::Vector3d &_pos, gz::math::Vector3d &_vel)
  {
    this->Robot(_ecm);
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (!_info.paused && dt > 0.0)
    {
      gz::math::Vector3d rate = -this->offset / kRecoveryTime;
      if (this->robot != gz::sim::kNullEntity)
      {
        gz::math::Vector3d away = _pos + this->offset -
                                  gz::sim::worldPose(this->robot, _ecm).Pos();
        away.Z() = 0.0;
        const double distance = away.Length();
        if (distance > 1e-3 && distance < this->influenceRadius)
          rate += away / distance * this->reactiveSign * this->responseSpeed *
                  (1.0 - distance / this->influenceRadius);
      }
      const gz::math::Vector3d next = this->offset + rate * dt;
      this->offsetRate = rate;
      this->offset = next;
      if (this->offset.Length() > kMaxOffset)
      {
        this->offset = this->offset.Normalized() * kMaxOffset;
        this->offsetRate = gz::math::Vector3d::Zero;
      }
    }
    _pos += this->offset;
    _vel += this->offsetRate;
  }

  // Position and velocity at time _t (already wrapped into [0, duration]).
  void Sample(double _t, gz::math::Vector3d &_pos,
              gz::math::Vector3d &_vel) const
  {
    _vel = gz::math::Vector3d::Zero;
    if (_t <= this->waypoints.front().time)
    {
      _pos = this->waypoints.front().position;
      return;
    }
    if (_t >= this->waypoints.back().time)
    {
      _pos = this->waypoints.back().position;
      return;
    }

    // First waypoint strictly after _t; segment is [i, i + 1].
    auto it = std::upper_bound(this->waypoints.begin(), this->waypoints.end(),
                               _t, [](double t, const Waypoint &wp)
                               { return t < wp.time; });
    const unsigned int i =
        static_cast<unsigned int>(it - this->waypoints.begin()) - 1;
    const Waypoint &a = this->waypoints[i];
    const Waypoint &b = this->waypoints[i + 1];
    const double dt = b.time - a.time;
    if (dt <= 0.0)
    {
      _pos = b.position;
      return;
    }
    const double s = (_t - a.time) / dt;

    if (this->spline)
    {
      _pos = this->curve.Interpolate(i, s);
      _vel = this->curve.InterpolateTangent(i, s) / dt;
    }
    else
    {
      _pos = a.position + (b.position - a.position) * s;
      _vel = (b.position - a.position) / dt;
    }
  }

  gz::sim::Model model{gz::sim::kNullEntity};
  std::vector<Waypoint> waypoints;
  gz::math::Spline curve;
  gz::math::Quaterniond rotation;
  double duration{0.0};
  bool loop{true};
  bool spline{true};
  double reactiveSign{0.0};  // 0 fixed motion, 1 yield to the robot, -1 block it
  bool syncStart{false};
  bool started{false};
  bool robotSeen{false};
  double startTime{0.0};
  gz::math::Vector3d robotSpawn{gz::math::Vector3d::Zero};
  double responseSpeed{kResponseSpeed};
  double influenceRadius{kInfluenceRadius};
  gz::sim::Entity robot{gz::sim::kNullEntity};
  gz::math::Vector3d offset{gz::math::Vector3d::Zero};
  gz::math::Vector3d offsetRate{gz::math::Vector3d::Zero};
};
}  // namespace dynabarn2

GZ_ADD_PLUGIN(dynabarn2::ObstacleMover, gz::sim::System,
              dynabarn2::ObstacleMover::ISystemConfigure,
              dynabarn2::ObstacleMover::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(dynabarn2::ObstacleMover, "dynabarn2::ObstacleMover")
