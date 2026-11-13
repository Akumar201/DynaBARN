// PredictionCritic: an MPPI critic that scores trajectories against moving obstacles in space-time.
//
// A costmap can only say where an obstacle is. This critic compares each trajectory's position at
// step k with where each tracked obstacle will be k steps from now, assuming constant velocity, so a
// gap that will have closed by the time the robot gets there costs more than one that has not.
//
// Optional reaction model (reaction_gain != 0): within reaction_radius each obstacle also moves along
// the line to the robot's position on that trajectory, at reaction_gain * (1 - d / reaction_radius)
// m/s (negative gain: towards the robot, as blocking obstacles do; positive: away, as yielding ones
// do), drifting back over reaction_recovery s and at most reaction_max_offset m off its path. This is
// the rule DynaBARN II's ObstacleMover uses, so with matching values it is an oracle of the reaction.
//
// Tracks come from obstacle_predictor on /predicted_tracks as [stamp, x, y, vx, vy, x, y, vx, vy, ...]
// in the costmap's frame.

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace mppi::critics
{

class PredictionCritic : public CriticFunction
{
public:
  void initialize() override
  {
    auto getParam = parameters_handler_->getParamGetter(name_);
    getParam(weight_, "cost_weight", 60.0f);
    getParam(collision_cost_, "collision_cost", 2000.0f);
    getParam(hit_radius_, "hit_radius", 0.75f);     // robot plus obstacle, centre to centre
    getParam(margin_, "margin", 0.5f);              // extra clearance that is preferred
    getParam(max_age_, "max_age", 0.5f);            // s, older tracks are ignored
    getParam(discount_, "discount", 0.98f);         // per step, as predictions weaken with time
    getParam(reaction_gain_, "reaction_gain", 0.0f);              // m/s, 0 disables the reaction model
    getParam(reaction_radius_, "reaction_radius", 2.5f);
    getParam(reaction_recovery_, "reaction_recovery", 2.0f);      // s
    getParam(reaction_max_offset_, "reaction_max_offset", 2.0f);  // m

    auto node = parent_.lock();
    clock_ = node->get_clock();
    subscription_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/predicted_tracks", rclcpp::QoS(5),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        tracks_ = msg->data;
      });
  }

  void score(CriticData & data) override
  {
    if (!enabled_) {
      return;
    }
    std::vector<double> tracks;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      tracks = tracks_;
    }
    if (tracks.size() < 5) {
      return;
    }
    const float age = static_cast<float>(clock_->now().seconds() - tracks[0]);
    if (age > max_age_) {
      return;
    }

    const auto & x = data.trajectories.x;
    const auto & y = data.trajectories.y;
    const size_t batch = x.shape(0);
    const size_t steps = x.shape(1);
    const float soft = hit_radius_ + margin_;
    constexpr size_t kStride = 2;

    const size_t n_tracks = (tracks.size() - 1) / 4;
    const bool react = reaction_gain_ != 0.0f;
    const float dt = data.model_dt;
    std::vector<float> ox(n_tracks), oy(n_tracks);

    for (size_t i = 0; i < batch; ++i) {
      float cost = 0.0f;
      bool hit = false;
      float weight = 1.0f;
      std::fill(ox.begin(), ox.end(), 0.0f);
      std::fill(oy.begin(), oy.end(), 0.0f);
      for (size_t k = 0; k < steps; ++k) {
        const float t = age + static_cast<float>(k) * dt;
        const bool scored = k % kStride == 0;
        for (size_t j = 0; j < n_tracks; ++j) {
          const size_t n = 1 + 4 * j;
          const float dx = x(i, k) - (static_cast<float>(tracks[n] + tracks[n + 2] * t) + ox[j]);
          const float dy = y(i, k) - (static_cast<float>(tracks[n + 1] + tracks[n + 3] * t) + oy[j]);
          const float distance = std::sqrt(dx * dx + dy * dy);
          if (scored && distance < soft) {
            hit = hit || distance < hit_radius_;
            cost += weight * (soft - distance) / margin_;
          }
          if (react) {
            // dx, dy point from the obstacle to the robot; the obstacle moves along -(dx, dy) * gain.
            float rx = -ox[j] / reaction_recovery_, ry = -oy[j] / reaction_recovery_;
            if (distance > 1e-3f && distance < reaction_radius_) {
              const float s = -reaction_gain_ * (1.0f - distance / reaction_radius_) / distance;
              rx += s * dx;
              ry += s * dy;
            }
            ox[j] += rx * dt;
            oy[j] += ry * dt;
            const float norm = std::sqrt(ox[j] * ox[j] + oy[j] * oy[j]);
            if (norm > reaction_max_offset_) {
              ox[j] *= reaction_max_offset_ / norm;
              oy[j] *= reaction_max_offset_ / norm;
            }
          }
        }
        if (scored) {
          weight *= discount_;
        }
      }
      data.costs[i] += weight_ * cost + (hit ? collision_cost_ : 0.0f);
    }
  }

private:
  float weight_{60.0f}, collision_cost_{2000.0f}, hit_radius_{0.75f}, margin_{0.5f};
  float max_age_{0.5f}, discount_{0.98f};
  float reaction_gain_{0.0f}, reaction_radius_{2.5f}, reaction_recovery_{2.0f}, reaction_max_offset_{2.0f};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
  std::mutex mutex_;
  std::vector<double> tracks_;
};

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::PredictionCritic, mppi::critics::CriticFunction)
