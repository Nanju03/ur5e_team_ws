// Compliant AG95 transmission for Gazebo Classic. Only joint torques are
// applied: contacts, carried objects and release remain the physics engine's job.
#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

namespace ur5e_a3_grid
{
class GazeboMimicPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    master_ = model->GetJoint(sdf->Get<std::string>("master"));
    if (!master_) {
      gzerr << "AG95 coupling: master joint is missing.\n";
      return;
    }
    for (auto element = sdf->GetElement("follower"); element;
      element = element->GetNextElement("follower"))
    {
      Follower follower;
      follower.joint = model->GetJoint(element->Get<std::string>("name"));
      follower.multiplier = element->Get<double>("multiplier");
      follower.offset = element->Get<double>("offset");
      follower.kp = element->Get<double>("kp");
      follower.kd = element->Get<double>("kd");
      follower.max_effort = element->Get<double>("max_effort");
      if (!follower.joint || !std::isfinite(follower.multiplier) ||
        !std::isfinite(follower.offset) || !std::isfinite(follower.kp) ||
        !std::isfinite(follower.kd) || !std::isfinite(follower.max_effort) ||
        follower.kp <= 0 || follower.kd < 0 || follower.max_effort <= 0)
      {
        gzerr << "AG95 coupling: invalid follower joint or force parameters.\n";
        return;
      }
      followers_.push_back(follower);
    }
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboMimicPlugin::Update, this));
  }

private:
  struct Follower
  {
    gazebo::physics::JointPtr joint;
    double multiplier, offset, kp, kd, max_effort;
  };

  void Update()
  {
    const double position = master_->Position(0);
    const double velocity = master_->GetVelocity(0);
    double reaction = 0.0;
    for (const auto & follower : followers_) {
      const double error = follower.multiplier * position + follower.offset -
        follower.joint->Position(0);
      const double velocity_error = follower.multiplier * velocity -
        follower.joint->GetVelocity(0);
      const double torque = std::clamp(
        follower.kp * error + follower.kd * velocity_error,
        -follower.max_effort, follower.max_effort);
      follower.joint->SetForce(0, torque);
      reaction -= follower.multiplier * torque;
    }
    // Gazebo accumulates forces within each physics step. This load reaction
    // adds to the bounded effort-controller command, allowing finger contact
    // to stall the master instead of forcing the followers through an object.
    master_->SetForce(0, reaction);
  }

  gazebo::physics::JointPtr master_;
  std::vector<Follower> followers_;
  gazebo::event::ConnectionPtr update_;
};
GZ_REGISTER_MODEL_PLUGIN(GazeboMimicPlugin)
}  // namespace ur5e_a3_grid
