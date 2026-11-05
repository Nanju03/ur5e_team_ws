// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <utility>

namespace gazebo_settling {
enum class Result { waiting, settled, extended, simulation_timeout, wall_timeout };

// Time alone must never mark a pose as settled. Extra simulation time is
// available only while measured position error continues to converge.
class Budget {
public:
  Budget(double initial_seconds,double maximum_seconds,double wall_seconds)
    : initial_(initial_seconds),maximum_(maximum_seconds),wall_(wall_seconds),
      deadline_(initial_seconds),window_(std::min(5.,initial_seconds)) {
    if(!std::isfinite(initial_)||!std::isfinite(maximum_)||!std::isfinite(wall_)||
       initial_<=0.||maximum_<initial_||wall_<=0.)
      throw std::invalid_argument("Invalid Gazebo arm settling time limits");
  }
  Result observe(double simulation_seconds,double wall_seconds,double error) {
    if(!std::isfinite(simulation_seconds)||simulation_seconds<0.||
       !std::isfinite(wall_seconds)||wall_seconds<0.||!std::isfinite(error)||error<0.)
      throw std::invalid_argument("Invalid Gazebo arm settling feedback");
    if(!history_.empty()&&simulation_seconds<history_.back().first)
      throw std::runtime_error("Gazebo clock moved backwards during settling");
    if(error<=tolerance)return Result::settled;
    if(wall_seconds>=wall_)return Result::wall_timeout;
    if(history_.empty()||simulation_seconds>history_.back().first)
      history_.emplace_back(simulation_seconds,error);
    else history_.back().second=error;
    while(history_.size()>1&&history_[1].first<=simulation_seconds-window_)
      history_.pop_front();
    if(simulation_seconds<deadline_)return Result::waiting;
    if(simulation_seconds>=maximum_)return Result::simulation_timeout;
    const auto [began,previous_error]=history_.front();
    const double interval=simulation_seconds-began;
    const double improvement=std::max(.0001,.02*previous_error);
    if(interval<window_||interval>2.*window_||previous_error-error<improvement)
      return Result::simulation_timeout;
    deadline_=std::min(maximum_,simulation_seconds+window_);
    return Result::extended;
  }
  double deadline()const{return deadline_;}
  static constexpr double tolerance=.005;
private:
  double initial_,maximum_,wall_,deadline_,window_;
  std::deque<std::pair<double,double>> history_;
};
}  // namespace gazebo_settling
