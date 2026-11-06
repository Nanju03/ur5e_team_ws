// SPDX-License-Identifier: BSD-3-Clause
// Use MoveIt's own parallel OMPL implementation with an explicit CPU budget.
// The surrounding PlanningPipeline retains its solution collision checks.
#include <algorithm>
#include <memory>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <pluginlib/class_list_macros.hpp>

namespace ur5e_a3_grid {
class ParallelOMPLPlanner : public planning_interface::PlannerManager {
public:
  bool initialize(const moveit::core::RobotModelConstPtr& model,
                  const rclcpp::Node::SharedPtr& node, const std::string& ns) override {
    int64_t workers=2;
    node->get_parameter_or("planner_workers",workers,int64_t(2));
    if(workers<1||workers>64)throw std::runtime_error("planner_workers must be 1..64");
    interface_=std::make_unique<ompl_interface::OMPLInterface>(model,node,ns);
    interface_->getPlanningContextManager().setMaximumPlanningThreads(workers);
    config_settings_=interface_->getPlannerConfigurations();
    RCLCPP_INFO(node->get_logger(),"STACK_PARALLEL_PLANNER workers=%ld backend=ompl_parallel first_solution=true simplification=true",workers);
    return true;
  }
  std::string getDescription()const override{return "Bounded parallel OMPL";}
  bool canServiceRequest(const planning_interface::MotionPlanRequest& req)const override {
    return req.trajectory_constraints.constraints.empty();
  }
  void getPlanningAlgorithms(std::vector<std::string>& names)const override {
    names.clear();for(const auto& item:config_settings_)names.push_back(item.first);
  }
  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& settings) override {
    interface_->setPlannerConfigurations(settings);
    config_settings_=interface_->getPlannerConfigurations();
  }
  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr& scene,
      const planning_interface::MotionPlanRequest& request,
      moveit_msgs::msg::MoveItErrorCodes& code)const override {
    auto context=interface_->getPlanningContext(scene,request,code);
    if(context) {
      // Stop competing candidates on the first exact solution. Keep normal
      // simplification/interpolation, constraints and dense downstream checks.
      context->setHybridize(false);
      context->simplifySolutions(true);
    }
    return context;
  }
private:
  std::unique_ptr<ompl_interface::OMPLInterface> interface_;
};
}
PLUGINLIB_EXPORT_CLASS(ur5e_a3_grid::ParallelOMPLPlanner,planning_interface::PlannerManager)
