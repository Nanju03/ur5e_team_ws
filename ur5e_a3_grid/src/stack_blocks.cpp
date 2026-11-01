// SPDX-License-Identifier: BSD-3-Clause
// RViz/GenericSystem block stacking. All scene changes are scoped to ur5e_stack/.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <fstream>
#include <filesystem>
#include <future>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <controller_manager_msgs/srv/list_hardware_components.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace {
using T = Eigen::Isometry3d;
using State = moveit::core::RobotState;
using MGI = moveit::planning_interface::MoveGroupInterface;
constexpr double PI = 3.14159265358979323846;
const std::string PREFIX = "ur5e_stack/";

std::string jsonString(const std::string& value) {
  std::ostringstream result;result<<'"';
  for(unsigned char character:value) {
    switch(character) {
      case '"':result<<"\\\"";break;
      case '\\':result<<"\\\\";break;
      case '\b':result<<"\\b";break;
      case '\f':result<<"\\f";break;
      case '\n':result<<"\\n";break;
      case '\r':result<<"\\r";break;
      case '\t':result<<"\\t";break;
      default:
        if(character<0x20)result<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<static_cast<int>(character)<<std::dec;
        else result<<static_cast<char>(character);
    }
  }
  result<<'"';return result.str();
}

template<class V> V parameter(const rclcpp::Node::SharedPtr& node, const std::string& name, V fallback) {
  if (!node->has_parameter(name)) node->declare_parameter<V>(name, fallback);
  return node->get_parameter(name).get_value<V>();
}
Eigen::Vector3d xyz(const rclcpp::Node::SharedPtr& node, const std::string& name, std::vector<double> fallback) {
  auto value = parameter(node, name, fallback);
  if (value.size()!=3 || !std::all_of(value.begin(),value.end(),[](double x){return std::isfinite(x);}))
    throw std::runtime_error(name+" needs three finite values");
  return {value[0],value[1],value[2]};
}
geometry_msgs::msg::Pose pose(const T& transform) {
  geometry_msgs::msg::Pose p;
  p.position.x=transform.translation().x(); p.position.y=transform.translation().y(); p.position.z=transform.translation().z();
  Eigen::Quaterniond q(transform.linear());
  p.orientation.x=q.x(); p.orientation.y=q.y(); p.orientation.z=q.z(); p.orientation.w=q.w();
  return p;
}
struct Lock {
  int fd=-1;
  Lock() {
    const char* domain=std::getenv("ROS_DOMAIN_ID");
    std::string file="/tmp/ur5e_stack_"+std::string(domain?domain:"0")+".lock";
    fd=open(file.c_str(),O_CREAT|O_RDWR,0600);
    if(fd<0 || flock(fd,LOCK_EX|LOCK_NB)!=0) throw std::runtime_error("Another stacking task is running in this ROS domain");
  }
  ~Lock(){if(fd>=0)close(fd);}
};
struct Step {
  std::string label;
  std::string phase;
  bool hand=false, event=false;
  moveit_msgs::msg::RobotTrajectory trajectory;
  moveit_msgs::msg::PlanningScene diff;
  moveit_msgs::msg::PlanningScene before_scene,after_scene;
  std::vector<double> start_joints,end_joints;
  double velocity_scaling=0.,acceleration_scaling=0.;
  int completed=0;
};
struct GripperBounds {
  const moveit::core::LinkModel* link=nullptr;
  std::array<Eigen::Vector3d,8> corners;
};
struct AlignmentStop {
  std::size_t after_segment=0;
  std::string corner,axis;
  int pocket_cell=0;
  double yaw_deg=0.;
  T target=T::Identity();
};

class StackTask {
public:
  explicit StackTask(rclcpp::Node::SharedPtr node):node_(std::move(node)),logger_(node_->get_logger()) {
    mode_=parameter<std::string>(node_,"stack_mode","scene");
    if(mode_!="scene" && mode_!="plan" && mode_!="run" && mode_!="reset" && mode_!="reach") throw std::runtime_error("Invalid stack_mode");
    execution_mode_=parameter<std::string>(node_,"execution_mode","step");
    if(execution_mode_!="step"&&execution_mode_!="continuous")throw std::runtime_error("execution_mode must be step or continuous");
    plan_id_="sim_"+std::to_string(getpid())+"_"+std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count());
    empty_hand_test_=parameter<bool>(node_,"empty_hand_test",false);
    if(empty_hand_test_&&mode_!="plan")throw std::runtime_error("empty_hand_test supports plan mode only; physical execution is not performed by this planner");
    empty_xy_scan_=parameter<bool>(node_,"empty_xy_scan",false);
    plate_alignment_=parameter<bool>(node_,"plate_alignment_test",false);
    if(plate_alignment_&&mode_!="plan"&&mode_!="run")
      throw std::runtime_error("plate_alignment_test requires plan or mock run mode");
    if(empty_xy_scan_&&!plate_alignment_&&(!empty_hand_test_||mode_!="plan"))
      throw std::runtime_error("empty_xy_scan requires empty_hand_test=true and stack_mode=plan");
    scan_clearance_=parameter<double>(node_,"empty_hand_clearance",.05);
    if(!std::isfinite(scan_clearance_)||scan_clearance_<=0)
      throw std::runtime_error("empty_hand_clearance must be finite and positive");
    display_file_=parameter<std::string>(node_,"display_trajectory_file","");
    if(!display_file_.empty()&&mode_!="plan")throw std::runtime_error("display_trajectory_file is only supported in plan mode");
    recipe_file_=parameter<std::string>(node_,"recipe_manifest_file","");
    if(!recipe_file_.empty()&&(mode_!="plan"||empty_hand_test_||plate_alignment_||empty_xy_scan_))
      throw std::runtime_error("recipe_manifest_file requires a full stacking plan; no execution is performed by this exporter");
    group_=parameter<std::string>(node_,"planning_group","ur5e_arm");
    hand_group_=parameter<std::string>(node_,"gripper_group","dh_ag95_gripper");
    hand_joint_=parameter<std::string>(node_,"gripper_joint","left_outer_knuckle_joint");
    frame_=parameter<std::string>(node_,"reference_frame","world");
    tip_=parameter<std::string>(node_,"ik_link","tool0");
    tcp_=parameter<std::string>(node_,"tcp_link","grasp_link");
    approach_=parameter<std::string>(node_,"approach_link","ag95_base_link");
    table_=parameter<std::string>(node_,"table_link","workbench");
    target_=parameter<int>(node_,"target_cell",0);
    columns_=parameter<int>(node_,"columns",4); rows_=parameter<int>(node_,"rows",6);
    width_=parameter<double>(node_,"size_x",.297); height_=parameter<double>(node_,"size_y",.420);
    table_z_=parameter<double>(node_,"table_surface_z",.8);
    plate_enabled_=parameter<bool>(node_,"plate_enabled",false);
    independent_target_=parameter<bool>(node_,"independent_stack_target",false);
    target_has_foundation_=parameter<bool>(node_,"target_has_foundation",!independent_target_);
    if(independent_target_&&target_has_foundation_)
      throw std::runtime_error("An independent stacking target cannot contain a foundation block");
    plate_bottom_z_=parameter<double>(node_,"plate_bottom_z",table_z_);
    source_surface_z_=parameter<double>(node_,"source_surface_z",plate_enabled_?plate_bottom_z_+.005:table_z_);
    stack_surface_z_=parameter<double>(node_,"stack_surface_z",plate_enabled_?plate_bottom_z_+.005:table_z_);
    obstacle_surface_z_=parameter<double>(node_,"obstacle_surface_z",plate_enabled_?plate_bottom_z_+.005:table_z_);
    plate_yaw_=parameter<double>(node_,"plate_yaw_deg",-90.)*PI/180.;
    auto plate_center=parameter<std::vector<double>>(node_,"plate_center_xy",{.4,0.});
    auto target_xy=parameter<std::vector<double>>(node_,"stack_target_xy",plate_center);
    if(plate_center.size()!=2||target_xy.size()!=2)throw std::runtime_error("Plate and stack centres need two coordinates");
    plate_center_=Eigen::Vector2d(plate_center[0],plate_center[1]);target_xy_=Eigen::Vector2d(target_xy[0],target_xy[1]);
    if(!plate_center_.allFinite()||!target_xy_.allFinite()||!std::isfinite(plate_bottom_z_)||
       !std::isfinite(source_surface_z_)||!std::isfinite(stack_surface_z_)||!std::isfinite(obstacle_surface_z_)||!std::isfinite(plate_yaw_))
      throw std::runtime_error("Plate geometry must be finite");
    plate_uri_=parameter<std::string>(node_,"plate_mesh_resource","package://ur5e_block_description/meshes/cube_plate.STL");
    alignment_clearance_=parameter<double>(node_,"plate_alignment_clearance",.05);
    if(!std::isfinite(alignment_clearance_)||alignment_clearance_<.03||alignment_clearance_>.15)
      throw std::runtime_error("Plate alignment clearance must be 30..150 mm");
    alignment_tip_clearance_=parameter<double>(node_,"plate_alignment_tip_clearance",.010);
    alignment_hand_position_=parameter<double>(node_,"plate_alignment_gripper_position",.65);
    if(!std::isfinite(alignment_tip_clearance_)||alignment_tip_clearance_<0.||alignment_tip_clearance_>.10||
       !std::isfinite(alignment_hand_position_)||alignment_hand_position_<0.||alignment_hand_position_>.93)
      throw std::runtime_error("Plate alignment tip clearance or gripper position is invalid");
    if(plate_alignment_&&!plate_enabled_)throw std::runtime_error("Plate alignment requires the plate profile");
    block_=xyz(node_,"block_size_xyz",{.025,.025,.025});
    obstacle_=xyz(node_,"obstacle_size_xyz",{.025,.025,.15});
    grip_height_=parameter<double>(node_,"grasp_height_above_table",.0125);
    clearance_=parameter<double>(node_,"place_clearance",.04);
    placement_mode_=parameter<std::string>(node_,"placement_mode","side");
    vertical_placement_layers_=parameter<int>(node_,"vertical_placement_layers",10);
    side_clearance_=parameter<double>(node_,"side_clearance",.06);
    vertical_fallback_=parameter<int>(node_,"vertical_fallback_layers",4);
    if(placement_mode_!="side"&&placement_mode_!="vertical")throw std::runtime_error("placement_mode must be side or vertical");
    if(vertical_placement_layers_<0)
      throw std::runtime_error("vertical_placement_layers must be non-negative");
    if(side_clearance_<=0||vertical_fallback_<0)throw std::runtime_error("Invalid side placement settings");
    if(placement_mode_=="side"&&(block_.array()-block_.x()).abs().maxCoeff()>1e-8)
      throw std::runtime_error("Side placement currently requires cubic blocks");
    vel_=parameter<double>(node_,"velocity_scaling",.25);
    acc_=parameter<double>(node_,"acceleration_scaling",.3);
    split_speeds_=parameter<bool>(node_,"split_stack_speeds",true);
    approach_vel_=parameter<double>(node_,"approach_velocity_scaling",.05);
    transfer_vel_=parameter<double>(node_,"transfer_velocity_scaling",.20);
    stack_acc_=parameter<double>(node_,"stack_acceleration_scaling",.05);
    approach_acc_=parameter<double>(node_,"approach_acceleration_scaling",stack_acc_);
    transfer_acc_=parameter<double>(node_,"transfer_acceleration_scaling",.20);
    staged_release_=parameter<bool>(node_,"staged_release_enabled",true);
    release_first_=parameter<double>(node_,"release_first_position",.70);
    release_clearance_=parameter<double>(node_,"release_clearance_position",.60);
    open_=parameter<double>(node_,"gripper_open_position",.4);
    close_=parameter<double>(node_,"gripper_grasp_position",.73);
    if(!std::isfinite(approach_vel_)||!std::isfinite(transfer_vel_)||!std::isfinite(stack_acc_)||
       !std::isfinite(approach_acc_)||!std::isfinite(transfer_acc_)||
       approach_vel_<=0.||approach_vel_>1.||transfer_vel_<=0.||transfer_vel_>1.||stack_acc_<=0.||stack_acc_>1.||
       approach_acc_<=0.||approach_acc_>1.||transfer_acc_<=0.||transfer_acc_>1.||
       !std::isfinite(release_first_)||!std::isfinite(release_clearance_)||
       release_clearance_<0.||release_first_>0.93||release_clearance_>=release_first_||
       (staged_release_&&!empty_hand_test_&&!plate_alignment_&&release_first_>=close_))
      throw std::runtime_error("Stacking speed or staged-release settings are invalid");
    step_=parameter<double>(node_,"cartesian_step",.004);
    for(auto id:parameter<std::vector<int64_t>>(node_,"obstacle_cells",{})) obstacles_.insert(static_cast<int>(id));
    for(auto id:parameter<std::vector<int64_t>>(node_,"empty_cells",{})) empty_.insert(static_cast<int>(id));
    if(columns_<1 || rows_<1 || columns_*rows_>100 || width_<=0 || height_<=0 ||
       (block_.array()<=0).any() || (obstacle_.array()<=0).any() || grip_height_<=0 || grip_height_>=block_.z() ||
       clearance_<=0 || step_<=0 || vel_<=0 || vel_>1 || acc_<=0 || acc_>1 || close_<=open_)
      throw std::runtime_error("Invalid grid, block, speed, or clearance settings");
    if(empty_hand_test_) {
      if(!std::isfinite(vel_)||!std::isfinite(acc_))
        throw std::runtime_error("Empty-hand planning requires finite velocity and acceleration scaling");
      vel_=std::min(vel_,.05);acc_=std::min(acc_,.05);
      RCLCPP_INFO(logger_,"EMPTY_HAND_PLAN velocity_scaling=%.3f acceleration_scaling=%.3f gripper_commands=disabled",vel_,acc_);
    }
    for(int id:obstacles_) if(id<1||id>columns_*rows_) throw std::runtime_error("Obstacle cell is outside the grid");
    for(int id:empty_) if(id<1||id>columns_*rows_||obstacles_.count(id))throw std::runtime_error("Empty cell is invalid or an obstacle");
    if(target_<0||target_>columns_*rows_||(!empty_xy_scan_&&!independent_target_&&(obstacles_.count(target_)||empty_.count(target_))))
      throw std::runtime_error("Target cell is invalid, empty, or an obstacle");
    if(empty_xy_scan_&&columns_*rows_!=24)
      throw std::runtime_error("The empty-hand XY scan requires exactly 24 cells");
    if(empty_xy_scan_&&!plate_alignment_&&obstacles_.empty())
      throw std::runtime_error("The empty-hand XY scan requires at least one selected obstacle");
    get_=node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    apply_=node_->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
    display_=node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",rclcpp::QoS(1).transient_local());
    markers_=node_->create_publisher<visualization_msgs::msg::MarkerArray>("/ur5e_a3_stack/markers",rclcpp::QoS(1).transient_local());
    feedback_subscription_=node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        if(message->name.size()!=message->position.size())return;
        const auto received=std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        for(std::size_t index=0;index<message->name.size();++index) {
          if(std::isfinite(message->position[index]))
            feedback_[message->name[index]]={message->position[index],received};
          else feedback_.erase(message->name[index]);
        }
      });
    stop_service_=node_->create_service<std_srvs::srv::Trigger>("/ur5e_a3_stack/stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> reply){
        stop(); reply->success=true; reply->message="Stacking stopped; current scene preserved";
      });
    cancel_timer_=node_->create_wall_timer(std::chrono::milliseconds(50),[this]{
      // Cover cancellation while an action goal is being accepted as well as
      // cancellation after execution has already started.
      if(cancelled_.load()&&executing_.load())stop();
    });
  }
  void stop() {
    cancelled_=true;
    if(auto* pipeline=pipeline_stop_.load())pipeline->terminate();
    if(executing_.load())try {if(auto* arm=arm_stop_.load())arm->stop(); if(auto* hand=hand_stop_.load())hand->stop();}catch(const std::exception&){}
  }
  bool run() {
    arm_=std::make_unique<MGI>(node_,group_,nullptr,rclcpp::Duration::from_seconds(15));
    arm_stop_=arm_.get();
    model_=arm_->getRobotModel(); arm_group_=model_->getJointModelGroup(group_);
    auto parameters=node_->create_client<rcl_interfaces::srv::GetParameters>("/robot_state_publisher/get_parameters");
    auto request=std::make_shared<rcl_interfaces::srv::GetParameters::Request>();request->names={"robot_description"};
    const auto description=callService(parameters,request,std::chrono::seconds(5));
    if(description->values.size()!=1 || description->values.front().type!=rcl_interfaces::msg::ParameterType::PARAMETER_STRING ||
       description->values.front().string_value.empty())
      throw std::runtime_error("Robot model publisher returned an invalid robot_description string; NOTHING executed");
    const auto& published_description=description->values.front().string_value;
    const auto expected_description=node_->get_parameter("robot_description").as_string();
    if(published_description!=expected_description)
      throw std::runtime_error("GUI and MoveIt robot models differ (publisher bytes="+std::to_string(published_description.size())+
        ", MoveIt bytes="+std::to_string(expected_description.size())+"). Restart MoveIt/RViz with the same riser option; NOTHING executed");
    gripper_group_=model_->getJointModelGroup(hand_group_);
    for(const auto& link:{frame_,tip_,tcp_,approach_,table_}) if(!model_->getLinkModel(link))throw std::runtime_error("Missing model link: "+link);
    if(!arm_group_ || !gripper_group_ || gripper_group_->getActiveJointModelNames()!=std::vector<std::string>{hand_joint_})
      throw std::runtime_error("Arm/gripper group does not match the configuration");
    auto measured=measuredState(10.);
    measured->update();
    std::ostringstream measured_joints;
    for(const auto& name:arm_group_->getActiveJointModelNames())measured_joints<<name<<'='<<std::setprecision(9)<<measured->getVariablePosition(name)<<' ';
    measured_joints<<hand_joint_<<'='<<measured->getVariablePosition(hand_joint_);
    const T measured_tcp=measured->getGlobalLinkTransform(frame_).inverse()*measured->getGlobalLinkTransform(tcp_);
    RCLCPP_INFO(logger_,"STACK_MEASURED_START tcp=%.9f,%.9f,%.9f joints=%s",measured_tcp.translation().x(),
      measured_tcp.translation().y(),measured_tcp.translation().z(),measured_joints.str().c_str());
    if(empty_hand_test_) {
      frozen_hand_position_=measured->getVariablePosition(hand_joint_);
      if(!std::isfinite(frozen_hand_position_)||!measured->satisfiesBounds(gripper_group_))
        throw std::runtime_error("Empty-hand planning requires a finite measured gripper position within its limits");
      RCLCPP_INFO(logger_,"EMPTY_HAND_PLAN frozen_joint=%s position=%.9f",hand_joint_.c_str(),frozen_hand_position_);
      if(plate_alignment_&&std::abs(frozen_hand_position_-alignment_hand_position_)>.015)
        throw std::runtime_error("Prepare the gripper at the reviewed plate alignment position 0.65 before planning the arm path");
    }
    scene_=readScene();
    initialiseGeometry(*measured);
    if(empty_xy_scan_||plate_alignment_)initialiseGripperBounds();
    auto initial=makeInitialScene();
    scene_->setPlanningSceneDiffMsg(initial);
    apply(initial);
    publishLabels();
    status("SCENE",0);
    if(mode_=="scene" || mode_=="reset")return true;
    if(plate_alignment_) {
      if(!empty_hand_test_) {
        if(mode_=="run")checkMockHardware();
        hand_=std::make_unique<MGI>(node_,hand_group_,nullptr,rclcpp::Duration::from_seconds(15));
        hand_stop_=hand_.get();
      }
      return planPlateAlignment(*measured,initial);
    }
    if(empty_xy_scan_)return planXYScan(*measured);
    if(!independent_target_&&target_==0)throw std::runtime_error("Select a tower target cell in the GUI");
    if(mode_=="run")checkMockHardware();
    hand_=std::make_unique<MGI>(node_,hand_group_,nullptr,rclcpp::Duration::from_seconds(15));
    hand_stop_=hand_.get();
    std::vector<int> sources;
    for(int id=1;id<=columns_*rows_;++id)if((independent_target_||id!=target_)&&!obstacles_.count(id)&&!empty_.count(id))sources.push_back(id);
    // Clear the tower's shadow first: blocks nearly aligned with the arm base
    // and tower are picked while the tower is low, starting with its neighbours.
    Eigen::Vector2d base_xy=(world_frame_.inverse()*measured->getGlobalLinkTransform("base_link")).translation().head<2>();
    Eigen::Vector2d depth=targetXY()-base_xy;if(depth.norm()<1e-8)depth=Eigen::Vector2d::UnitX();depth.normalize();
    std::sort(sources.begin(),sources.end(),[this,depth,base_xy](int a,int b){
      auto priority=[this](int id){
        auto adjacent=[this](int x,int y){return std::abs((x-1)/columns_-(y-1)/columns_)+std::abs((x-1)%columns_-(y-1)%columns_)==1;};
        bool near_target=independent_target_?(cells_[id-1]-targetXY()).norm()<.085:adjacent(id,target_);
        bool near_obstacle=std::any_of(obstacles_.begin(),obstacles_.end(),[&](int obstacle){return adjacent(id,obstacle);});
        if(near_target&&near_obstacle)return 0;
        if(near_obstacle)return 1; // Do constrained picks before the tower narrows their access.
        if(near_target)return 2;
        return 3;
      };
      int priority_a=priority(a),priority_b=priority(b);if(priority_a!=priority_b)return priority_a<priority_b;
      if(priority_a<=2) {
        double distance_a=(cells_[a-1]-targetXY()).squaredNorm();
        double distance_b=(cells_[b-1]-targetXY()).squaredNorm();
        if(distance_a!=distance_b)return distance_a<distance_b;
      }
      Eigen::Vector2d pa=cells_[a-1]-base_xy,pb=cells_[b-1]-base_xy;
      double angle_a=std::abs(std::atan2(depth.x()*pa.y()-depth.y()*pa.x(),depth.dot(pa)));
      double angle_b=std::abs(std::atan2(depth.x()*pb.y()-depth.y()*pb.x(),depth.dot(pb)));
      int bucket_a=static_cast<int>(angle_a/.01),bucket_b=static_cast<int>(angle_b/.01);
      if(bucket_a!=bucket_b)return bucket_a<bucket_b;
      return pa.squaredNorm()<pb.squaredNorm();
    });
    std::string order;for(int source:sources)order+=(order.empty()?"":",")+std::to_string(source);
    RCLCPP_INFO(logger_,"STACK_PICK_ORDER %s",order.c_str());
    total_=sources.size();
    checkTowerReach(*measured,static_cast<int>(total_+foundationLayers()));
    if(mode_=="reach")return true;
    scene_->setCurrentState(*measured);
    initial_state_=std::make_shared<State>(*measured);
    if(!recipe_file_.empty())scene_->getPlanningSceneMsg(recipe_initial_scene_);
    initialisePipeline();
    planHand(open_,"Initial gripper open");
    // Plan from measured feedback, including a collision-aware working-home preparation.
    planNeutral("Prepare vertical working home");
    for(std::size_t index=0;index<sources.size();++index) {
      checkStop();
      int source=sources[index]; std::string object=blockId(source);
      int layer=static_cast<int>(index)+1+foundationLayers();
      RCLCPP_INFO(logger_,"STACK_PLANNING source=%d target=%d layer=%d/%zu",source,target_,layer,total_+foundationLayers());
      contacts(object,true);
      auto grasp=planApproach(at(cells_[source-1],source_surface_z_+grip_height_),travel_z_,"Source "+std::to_string(source));
      T pick_pre=grasp.second;
      planHand(close_,"Close on block "+std::to_string(source));
      attach(object,source);
      planStraight(pick_pre,"Lift block "+std::to_string(source));
      tableContact(object,false);
      // The carried cube intentionally contacts only the supporting top cube.
      std::string support=index==0?(target_has_foundation_?blockId(target_):plateId()):blockId(sources[index-1]);
      auto placement=planPlacement(object,support,layer,"Place block "+std::to_string(source)+" layer "+std::to_string(layer));
      T place_pre=placement.second;
      if(staged_release_)planHand(release_first_,"Release block "+std::to_string(source)+" first opening");
      else planHand(open_,"Release block "+std::to_string(source));
      release(object,layer);
      steps_.back().completed=static_cast<int>(index)+1;
      if(staged_release_)planHand(release_clearance_,"Release block "+std::to_string(source)+" clearance opening before retreat");
      planStraight(place_pre,"Retreat from layer "+std::to_string(layer));
      contacts(object,false);
      // Reorient away from the tower before choosing the next vertical grasp.
      // This also avoids carrying a side-placement wrist branch into a deep pick.
      if(index+1<sources.size())planNeutral("Prepare next vertical pickup");
    }
    planNeutral("Return to clear working pose");
    RCLCPP_INFO(logger_,"STACK_PREFLIGHT_PASSED moves=%zu layers=%zu segments=%zu",total_,total_+foundationLayers(),steps_.size());
    showPlan();
    if(!recipe_file_.empty())exportRecipe();
    if(mode_=="plan"){status("PLAN_READY",0);return true;}
    apply(initial);
    execute();
    return true;
  }
private:
  moveit::core::RobotStatePtr measuredState(double timeout) {
    // Arm and gripper publish separate partial messages. MoveIt's aggregate
    // state timestamp can become current after only the gripper has arrived,
    // leaving an unreceived arm at zero. Require every measured joint here.
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(timeout);
    auto names=arm_group_->getActiveJointModelNames();names.push_back(hand_joint_);
    std::string missing;
    while(true) {
      checkStop();std::map<std::string,double> positions;missing.clear();
      const auto now=std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        for(const auto& name:names) {
          const auto found=feedback_.find(name);
          if(found==feedback_.end()||now-found->second.received>std::chrono::seconds(1))missing+=name+" ";
          else positions[name]=found->second.position;
        }
      }
      if(missing.empty()) {
        auto state=std::make_shared<State>(model_);state->setToDefaultValues();
        std::vector<double> arm_positions;for(const auto& name:arm_group_->getActiveJointModelNames())arm_positions.push_back(positions.at(name));
        state->setJointGroupActivePositions(arm_group_,arm_positions);
        state->setJointGroupActivePositions(gripper_group_,std::vector<double>{positions.at(hand_joint_)});
        state->update();return state;
      }
      if(now>=deadline)throw std::runtime_error("Missing or stale measured joints; NOTHING executed: "+missing);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  void initialisePipeline() {
    pipeline_=std::make_unique<planning_pipeline::PlanningPipeline>(model_,node_,"ompl",
        "ompl_interface/OMPLPlanner",std::vector<std::string>{});
    pipeline_stop_=pipeline_.get();
    // Match the planner's edge checks to the dense validation in save(). The
    // OMPL default can skip a thin block/obstacle between distant joint states.
    auto planner=pipeline_->getPlannerManager();
    auto configurations=planner->getPlannerConfigurations();
    for(auto& entry:configurations)if(entry.second.group==group_) {
      auto& value=entry.second.config["longest_valid_segment_fraction"];
      double resolution=value.empty()?.0002:std::min(.0002,std::stod(value));
      value=std::to_string(resolution);
    }
    planner->setPlannerConfigurations(configurations);
    pipeline_->displayComputedMotionPlans(false);
    pipeline_->checkSolutionPaths(true);
  }
  void checkStop()const {if(cancelled_||!rclcpp::ok())throw std::runtime_error("Stacking cancelled; current scene preserved");}
  template<class Service>
  typename Service::Response::SharedPtr callService(const std::shared_ptr<rclcpp::Client<Service>>& client,
      const typename Service::Request::SharedPtr& request, std::chrono::seconds response_timeout) {
    using Clock=std::chrono::steady_clock;
    const auto poll=std::chrono::milliseconds(50);
    const int attempts=mode_=="plan"?3:1;
    const std::string service=client->get_service_name();
    for(int attempt=1;attempt<=attempts;++attempt) {
      checkStop();
      const auto service_deadline=Clock::now()+std::chrono::seconds(5);
      while(!client->service_is_ready()) {
        checkStop();
        const auto remaining=service_deadline-Clock::now();
        if(remaining<=Clock::duration::zero())break;
        client->wait_for_service(std::min(Clock::duration(poll),remaining));
      }
      checkStop();
      std::string failure;
      if(!client->service_is_ready()) {
        failure="ROS service unavailable after waiting 5 seconds: "+service;
      } else {
        auto future=client->async_send_request(request);
        const auto response_deadline=Clock::now()+response_timeout;
        try {
          while(future.wait_for(std::chrono::seconds(0))!=std::future_status::ready) {
            checkStop();
            const auto remaining=response_deadline-Clock::now();
            if(remaining<=Clock::duration::zero())break;
            future.wait_for(std::min(Clock::duration(poll),remaining));
          }
          checkStop();
          if(future.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
            auto response=future.get();
            if(!response)throw std::runtime_error("ROS service returned an empty response: "+service);
            return response;
          }
        } catch(...) {
          client->remove_pending_request(future);
          throw;
        }
        client->remove_pending_request(future);
        failure="ROS service response timed out: "+service;
      }
      if(attempt==attempts)throw std::runtime_error(failure);
      RCLCPP_WARN(logger_,"STACK_SERVICE_RETRY service=%s attempt=%d/%d reason=%s",service.c_str(),attempt,attempts,failure.c_str());
      const auto retry_at=Clock::now()+std::chrono::milliseconds(250);
      while(true) {
        checkStop();
        const auto remaining=retry_at-Clock::now();
        if(remaining<=Clock::duration::zero())break;
        std::this_thread::sleep_for(std::min(Clock::duration(poll),remaining));
      }
    }
    throw std::runtime_error("ROS service retry limit reached: "+service);
  }
  moveit_msgs::msg::PlanningScene diff()const {
    moveit_msgs::msg::PlanningScene out;out.is_diff=true;out.robot_state.is_diff=true;return out;
  }
  planning_scene::PlanningScenePtr readScene() {
    auto request=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();request->components.components=1023;
    const auto response=callService(get_,request,std::chrono::seconds(10));
    auto scene=std::make_shared<planning_scene::PlanningScene>(model_);scene->setPlanningSceneMsg(response->scene);return scene;
  }
  void apply(const moveit_msgs::msg::PlanningScene& update) {
    checkStop();
    auto request=std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();request->scene=update;
    const auto response=callService(apply_,request,std::chrono::seconds(10));
    if(!response->success)throw std::runtime_error("MoveIt rejected a scene update");
  }
  void initialiseGeometry(const State& measured) {
    State home(measured);
    if(!home.setToDefaultValues(arm_group_,parameter<std::string>(node_,"home_pose","home")))throw std::runtime_error("Named home is missing");
    home.update();world_frame_=home.getGlobalLinkTransform(frame_);
    center_=world_frame_.inverse()*home.getGlobalLinkTransform(tcp_);
    tip_tcp_=home.getGlobalLinkTransform(tip_).inverse()*home.getGlobalLinkTransform(tcp_);
    tcp_approach_=home.getGlobalLinkTransform(tcp_).inverse()*home.getGlobalLinkTransform(approach_);
    if(!parameter<bool>(node_,"center_from_home",true))center_.translation()=xyz(node_,"center_xyz",{0,0,0});
    center_.translation()+=xyz(node_,"center_offset_xyz",{0,0,0});
    T approach=world_frame_.inverse()*home.getGlobalLinkTransform(approach_);
    Eigen::Vector3d down(0,0,-1),x=approach.linear().col(0);x-=down*down.dot(x);x.normalize();
    Eigen::Matrix3d rotation;rotation.col(0)=x;rotation.col(1)=down.cross(x);rotation.col(2)=down;
    center_.linear()=rotation*approach.linear().transpose()*center_.linear();
    travel_z_=center_.translation().z();
    rotation_=Eigen::AngleAxisd(plate_enabled_?plate_yaw_:parameter<double>(node_,"grid_yaw_deg",0)*PI/180.,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    if(!empty_xy_scan_&&travel_z_<table_z_+obstacle_.z()+.04)
      throw std::runtime_error("Home travel height must clear the obstacle by 40 mm");
    auto measured_cells=parameter<std::vector<double>>(node_,"cell_centers_xy",{});
    if(plate_enabled_) {
      if(measured_cells.empty()) {
        std::vector<Eigen::Vector2d> points;
        for(double y:{.0594,.1188,.1782,.2376})for(double x:{.060,.120,.180,.240,.300,.360}) {
          Eigen::Vector3d local(x-.210,y-.1485,0.);
          points.push_back(plate_center_+(rotation_*local).head<2>());
        }
        std::stable_sort(points.begin(),points.end(),[](const auto& a,const auto& b){
          if(std::abs(a.y()-b.y())>1e-7)return a.y()>b.y();return a.x()<b.x();
        });
        cells_=points;
      } else {
        if(measured_cells.size()!=static_cast<std::size_t>(columns_*rows_*2))
          throw std::runtime_error("Plate profile needs every pocket XY coordinate");
        for(std::size_t i=0;i<measured_cells.size();i+=2) {
          Eigen::Vector2d point(measured_cells[i],measured_cells[i+1]);
          if(!point.allFinite())throw std::runtime_error("Plate pocket coordinates must be finite");
          cells_.push_back(point);
        }
      }
      center_.translation().head<2>()=plate_center_;
    } else for(int r=0;r<rows_;++r)for(int c=0;c<columns_;++c) {
      Eigen::Vector3d local(((c+.5)/columns_-.5)*width_,(.5-(r+.5)/rows_)*height_,0);
      cells_.push_back((center_.translation()+rotation_*local).head<2>());
    }
    touch_=parameter<std::vector<std::string>>(node_,"touch_links",{"left_finger_pad","right_finger_pad","left_finger","right_finger"});
    for(auto& link:touch_)if(!model_->getLinkModel(link))throw std::runtime_error("Missing touch link: "+link);
  }
  void initialiseGripperBounds() {
    for(const auto* link:model_->getLinkModels()) {
      bool gripper=false;
      for(const auto* ancestor=link;ancestor;) {
        if(ancestor->getName()==approach_){gripper=true;break;}
        const auto* joint=ancestor->getParentJointModel();
        ancestor=joint?joint->getParentLinkModel():nullptr;
      }
      if(!gripper)continue;
      const auto& geometry=link->getShapes();const auto& origins=link->getCollisionOriginTransforms();
      if(geometry.size()!=origins.size())throw std::runtime_error("Gripper collision geometry/origins differ");
      for(std::size_t index=0;index<geometry.size();++index) {
        const auto* shape=geometry[index].get();
        Eigen::Vector3d low,high;
        if(shape->type==shapes::MESH) {
          const auto* mesh=static_cast<const shapes::Mesh*>(shape);
          if(!mesh->vertex_count||!mesh->vertices)throw std::runtime_error("Empty gripper collision mesh");
          low.setConstant(std::numeric_limits<double>::infinity());high=-low;
          for(unsigned int vertex=0;vertex<mesh->vertex_count;++vertex) {
            Eigen::Vector3d point(mesh->vertices[3*vertex],mesh->vertices[3*vertex+1],mesh->vertices[3*vertex+2]);
            if(!point.allFinite())throw std::runtime_error("Nonfinite gripper collision vertex");
            low=low.cwiseMin(point);high=high.cwiseMax(point);
          }
        } else {
          Eigen::Vector3d half;
          if(shape->type==shapes::BOX) {
            const auto* box=static_cast<const shapes::Box*>(shape);
            half=Eigen::Vector3d(box->size[0],box->size[1],box->size[2])*.5;
          } else if(shape->type==shapes::SPHERE) {
            half.setConstant(static_cast<const shapes::Sphere*>(shape)->radius);
          } else if(shape->type==shapes::CYLINDER) {
            const auto* cylinder=static_cast<const shapes::Cylinder*>(shape);
            half=Eigen::Vector3d(cylinder->radius,cylinder->radius,cylinder->length*.5);
          } else if(shape->type==shapes::CONE) {
            const auto* cone=static_cast<const shapes::Cone*>(shape);
            half=Eigen::Vector3d(cone->radius,cone->radius,cone->length*.5);
          } else throw std::runtime_error("Unsupported gripper collision shape for the XY scan");
          if(!half.allFinite()||(half.array()<0).any())throw std::runtime_error("Invalid gripper collision dimensions");
          low=-half;high=half;
        }
        // Mesh coordinates need not be centered at zero. Keep their actual
        // local AABB center before applying the collision-origin transform.
        const Eigen::Vector3d center=(low+high)*.5,half=(high-low)*.5;
        GripperBounds bounds;bounds.link=link;int corner=0;
        for(int x:{-1,1})for(int y:{-1,1})for(int z:{-1,1})
          bounds.corners[corner++]=origins[index]*(center+Eigen::Vector3d(x*half.x(),y*half.y(),z*half.z()));
        gripper_bounds_.push_back(std::move(bounds));
      }
    }
    if(gripper_bounds_.empty())throw std::runtime_error("XY scan needs gripper collision geometry");
  }
  double minimumGripperZ(const State& state)const {
    const T reference=state.getGlobalLinkTransform(frame_).inverse();
    double minimum=std::numeric_limits<double>::infinity();
    for(const auto& bounds:gripper_bounds_) {
      const T transform=reference*state.getGlobalLinkTransform(bounds.link);
      for(const auto& corner:bounds.corners)minimum=std::min(minimum,(transform*corner).z());
    }
    if(!std::isfinite(minimum))throw std::runtime_error("Cannot determine the gripper collision height");
    return minimum;
  }
  void planScanOrientation(const T& target,const std::string& label="XY scan turn gripper down at fixed TCP") {
    State current(scene_->getCurrentState());current.update();
    const T start=world_frame_.inverse()*current.getGlobalLinkTransform(tcp_);
    const Eigen::Quaterniond first(start.linear()),last(target.linear());
    const double angle=first.angularDistance(last);
    if(angle<1e-7)return;
    const int count=std::max(1,static_cast<int>(std::ceil(angle/.01)));
    robot_trajectory::RobotTrajectory path(model_,group_);path.addSuffixWayPoint(current,0.);
    for(int index=1;index<=count;++index) {
      checkStop();T waypoint=target;
      waypoint.linear()=first.slerp(static_cast<double>(index)/count,last).toRotationMatrix();
      // Solve at the TCP, keeping its XYZ fixed while the approach axis turns
      // down. Interpolating tool0 directly would move an offset TCP in an arc.
      if(!solve(waypoint,current))throw std::runtime_error("No clearance-preserving gripper-down rotation for XY scan");
      path.addSuffixWayPoint(current,0.);
    }
    save(path,label);
  }
  bool planXYScan(const State& measured) {
    scene_->setCurrentState(measured);initial_state_=std::make_shared<State>(measured);
    initialisePipeline();total_=cells_.size();
    const T current=world_frame_.inverse()*measured.getGlobalLinkTransform(tcp_);
    T down=current;down.linear()=center_.linear();
    const Eigen::Quaterniond first(current.linear()),last(down.linear());
    const double angle=first.angularDistance(last);
    const int orientation_samples=std::max(1,static_cast<int>(std::ceil(angle/.01)));
    std::vector<Eigen::Vector3d> local_corners;double radius=0.;
    const T tcp_inverse=measured.getGlobalLinkTransform(tcp_).inverse();
    for(const auto& bounds:gripper_bounds_) {
      const T transform=tcp_inverse*measured.getGlobalLinkTransform(bounds.link);
      for(const auto& corner:bounds.corners) {
        const Eigen::Vector3d local=transform*corner;
        local_corners.push_back(local);radius=std::max(radius,local.norm());
      }
    }
    double lowest_offset=std::numeric_limits<double>::infinity();
    for(int index=0;index<=orientation_samples;++index) {
      const Eigen::Matrix3d rotation=first.slerp(static_cast<double>(index)/orientation_samples,last).toRotationMatrix();
      for(const auto& corner:local_corners)lowest_offset=std::min(lowest_offset,(rotation*corner).z());
    }
    // The radius*angle_step bound covers unsampled orientations between these
    // samples. An extra 5 mm keeps numerical IK tolerance away from the floor.
    scan_floor_=obstacle_surface_z_+obstacle_.z()+scan_clearance_+.02;
    const double orientation_margin=radius*angle/orientation_samples;
    const double height=std::max({travel_z_,current.translation().z()+.005,
                                 scan_floor_-lowest_offset+orientation_margin+.005});
    T raised=current;raised.translation().z()=height;
    planStraight(raised,"XY scan initial vertical lift preserving current orientation");
    if(steps_.empty()||steps_.front().event||steps_.front().hand)
      throw std::runtime_error("XY scan did not start with an arm-only vertical lift");
    scan_floor_check_=true;
    scan_minimum_z_=minimumGripperZ(scene_->getCurrentState());
    if(!valid(scene_->getCurrentStateNonConst()))throw std::runtime_error("Initial XY scan lift did not clear the gripper floor");
    travel_z_=height;down.translation().z()=height;
    planScanOrientation(down);
    for(int row=0;row<rows_;++row)for(int index=0;index<columns_;++index) {
      const int column=row%2?columns_-1-index:index;
      const int cell=row*columns_+column+1;
      T target=down;target.translation()=Eigen::Vector3d(cells_[cell-1].x(),cells_[cell-1].y(),height);
      planStraight(target,"XY scan cell "+std::to_string(cell));
      RCLCPP_INFO(logger_,"STACK_SCAN_POINT cell=%d x=%.9f y=%.9f z=%.9f",cell,
                  target.translation().x(),target.translation().y(),target.translation().z());
    }
    // The observation ends at the last cell. There is no return/home motion,
    // hand trajectory, attachment event, or synthetic placement in this path.
    auto& final=steps_.back().trajectory.joint_trajectory.points.back();
    final.velocities.assign(final.positions.size(),0.);final.accelerations.assign(final.positions.size(),0.);
    RCLCPP_INFO(logger_,"STACK_SCAN_HEIGHT min_gripper_z=%.9f floor=%.9f tcp_z=%.9f",scan_minimum_z_,scan_floor_,height);
    RCLCPP_INFO(logger_,"STACK_PREFLIGHT_PASSED scan_cells=%zu segments=%zu",total_,steps_.size());
    showPlan();status("PLAN_READY",0);return true;
  }
  bool planPlateAlignment(const State& measured,const moveit_msgs::msg::PlanningScene& initial) {
    scene_->setCurrentState(measured);initial_state_=std::make_shared<State>(measured);
    initialisePipeline();total_=4;
    if(!empty_hand_test_)planHand(alignment_hand_position_,"Plate alignment gripper position 0.65");
    // Physical alignment prepares the hand in a separately reviewed action,
    // then freezes its measured shape. Simulation previews the hand step first.
    const State prepared(scene_->getCurrentState());
    const T current=world_frame_.inverse()*prepared.getGlobalLinkTransform(tcp_);
    T down=current;down.linear()=center_.linear();
    const Eigen::Quaterniond first(current.linear()),last(down.linear());
    const double angle=first.angularDistance(last);
    const int orientation_samples=std::max(1,static_cast<int>(std::ceil(angle/.01)));
    const T tcp_inverse=prepared.getGlobalLinkTransform(tcp_).inverse();
    std::vector<Eigen::Vector3d> corners;double radius=0.;
    for(const auto& bounds:gripper_bounds_)for(const auto& corner:bounds.corners) {
      const Eigen::Vector3d local=tcp_inverse*prepared.getGlobalLinkTransform(bounds.link)*corner;
      corners.push_back(local);radius=std::max(radius,local.norm());
    }
    double lowest=std::numeric_limits<double>::infinity(),down_lowest=lowest;
    for(int index=0;index<=orientation_samples;++index) {
      const Eigen::Matrix3d rotation=first.slerp(static_cast<double>(index)/orientation_samples,last).toRotationMatrix();
      for(const auto& corner:corners)lowest=std::min(lowest,(rotation*corner).z());
    }
    T sideways=down;
    sideways.linear()=Eigen::AngleAxisd(PI/2.,Eigen::Vector3d::UnitZ()).toRotationMatrix()*down.linear();
    // Both jaws are checked at the same whole-gripper floor. A world-Z turn
    // preserves every corner's height, but retain the explicit second shape.
    for(const auto& corner:corners) {
      down_lowest=std::min(down_lowest,(down.linear()*corner).z());
      down_lowest=std::min(down_lowest,(sideways.linear()*corner).z());
    }
    // The empty corner pockets locate the physical plate. Hold the complete
    // gripper 10mm above the rim; this is distinct from the higher travel pose.
    // Selected obstacles remain collision geometry and are never ignored.
    scan_floor_=plate_bottom_z_+.010+alignment_tip_clearance_;
    const double height=std::max({travel_z_,current.translation().z()+.005,
      scan_floor_-lowest+radius*angle/orientation_samples+alignment_clearance_+.005});
    // Keep the requested 10mm minimum despite finite IK accuracy. The 0.1mm
    // target reserve avoids placing every endpoint exactly on the strict floor.
    const double hover=scan_floor_-down_lowest+.0001;
    T raised=current;raised.translation().z()=height;
    planStraight(raised,"Plate alignment initial vertical lift preserving current orientation");
    scan_floor_check_=true;scan_minimum_z_=minimumGripperZ(scene_->getCurrentState());
    down.translation().z()=height;planScanOrientation(down);
    auto coordinates=parameter<std::vector<double>>(node_,"plate_alignment_points_xyz",{});
    auto ids=parameter<std::vector<int64_t>>(node_,"plate_alignment_cell_ids",
      {1,columns_,columns_*rows_,(rows_-1)*columns_+1});
    if(ids.size()!=4||std::set<int64_t>(ids.begin(),ids.end()).size()!=4)
      throw std::runtime_error("Plate alignment requires four distinct corner pocket IDs");
    std::vector<Eigen::Vector3d> points;
    if(!coordinates.empty()&&coordinates.size()!=12)
      throw std::runtime_error("Plate alignment requires exactly four XYZ pocket centres");
    for(std::size_t index=0;index<ids.size();++index) {
      const auto id=ids[index];
      if(id<1||id>static_cast<int64_t>(cells_.size()))
        throw std::runtime_error("Plate alignment pocket ID is outside the grid");
      Eigen::Vector3d point(cells_[id-1].x(),cells_[id-1].y(),plate_bottom_z_+.005);
      if(!coordinates.empty()) {
        const Eigen::Vector3d configured(coordinates[index*3],coordinates[index*3+1],coordinates[index*3+2]);
        if(!configured.allFinite()||(configured-point).norm()>1e-8)
          throw std::runtime_error("Plate alignment points must match their pocket centres and floor height");
      }
      points.push_back(point);
    }
    alignment_stops_.clear();
    for(std::size_t index=0;index<points.size();++index) {
      const std::string letter(1,static_cast<char>('A'+index));
      const std::string pocket="Plate pocket "+letter+" [cell "+std::to_string(ids[index])+"]";
      T high=down;high.translation()=Eigen::Vector3d(points[index].x(),points[index].y(),height);
      planStraight(high,pocket+" X safe-height approach");
      T target=down;target.translation()=Eigen::Vector3d(points[index].x(),points[index].y(),hover);
      planStraight(target,pocket+" X alignment hold");
      recordAlignmentStop(letter,"X",static_cast<int>(ids[index]),0.,target);
      RCLCPP_INFO(logger_,"STACK_SCAN_POINT cell=%zu x=%.9f y=%.9f z=%.9f corner=%s pocket_cell=%ld",index+1,
        target.translation().x(),target.translation().y(),target.translation().z(),letter.c_str(),static_cast<long>(ids[index]));
      // Each lowered X/Y pose is a manual plate-placement stop. Raise before
      // rotating; the TCP stays at this pocket's high XYZ during the 90deg turn.
      planStraight(high,pocket+" X vertical retract");
      T high_y=sideways;high_y.translation()=high.translation();
      planScanOrientation(high_y,pocket+" turn to Y at safe height");
      T target_y=sideways;target_y.translation()=target.translation();
      planStraight(target_y,pocket+" Y alignment hold");
      recordAlignmentStop(letter,"Y",static_cast<int>(ids[index]),90.,target_y);
      if(index+1<points.size()) {
        planStraight(high_y,pocket+" Y vertical retract");
        planScanOrientation(high,pocket+" turn to X at safe height");
      }
    }
    auto& final=steps_.back().trajectory.joint_trajectory.points.back();
    final.velocities.assign(final.positions.size(),0.);final.accelerations.assign(final.positions.size(),0.);
    RCLCPP_INFO(logger_,"STACK_SCAN_HEIGHT min_gripper_z=%.9f floor=%.9f tcp_z=%.9f",scan_minimum_z_,scan_floor_,hover);
    RCLCPP_INFO(logger_,"STACK_PREFLIGHT_PASSED scan_cells=4 alignment_stops=%zu segments=%zu",alignment_stops_.size(),steps_.size());
    showPlan();
    if(mode_=="plan"){status("PLAN_READY",0);return true;}
    apply(initial);execute();
    RCLCPP_INFO(logger_,"STACK_ALIGNMENT_HELD pocket_cell=%ld axis=Y gripper_position=%.9f",static_cast<long>(ids.back()),alignment_hand_position_);
    return true;
  }
  std::string alignmentStopData(const AlignmentStop& stop)const {
    const auto p=pose(stop.target);
    std::ostringstream out;out<<std::setprecision(17)
      <<"{\"after_segment\":"<<stop.after_segment<<",\"corner\":"<<jsonString(stop.corner)
      <<",\"axis\":"<<jsonString(stop.axis)<<",\"pocket_cell\":"<<stop.pocket_cell
      <<",\"yaw_deg\":"<<stop.yaw_deg<<",\"x\":"<<p.position.x
      <<",\"y\":"<<p.position.y<<",\"z\":"<<p.position.z
      <<",\"orientation_xyzw\":["<<p.orientation.x<<','<<p.orientation.y<<','
      <<p.orientation.z<<','<<p.orientation.w<<"]}";
    return out.str();
  }
  void recordAlignmentStop(const std::string& corner,const std::string& axis,int cell,double yaw,const T& target) {
    if(steps_.empty()||steps_.back().event||steps_.back().hand)
      throw std::runtime_error("Plate alignment hold must end with an arm trajectory");
    AlignmentStop stop;stop.after_segment=steps_.size();stop.corner=corner;stop.axis=axis;
    stop.pocket_cell=cell;stop.yaw_deg=yaw;stop.target=target;alignment_stops_.push_back(stop);
    const auto data=alignmentStopData(stop);
    RCLCPP_INFO(logger_,"STACK_ALIGNMENT_STOP %s",data.c_str());
  }
  void alignmentEvent(const std::string& marker,std::size_t index)const {
    if(index>=alignment_stops_.size())throw std::runtime_error("Invalid alignment stop event index");
    auto data=alignmentStopData(alignment_stops_[index]);data.pop_back();
    data+=",\"stop_index\":"+std::to_string(index+1)+",\"total\":"+
      std::to_string(alignment_stops_.size())+"}";
    RCLCPP_INFO(logger_,"%s %s",marker.c_str(),data.c_str());
  }
  int foundationLayers()const{return target_has_foundation_?1:0;}
  Eigen::Vector2d targetXY()const {
    if(independent_target_)return target_xy_;
    if(target_<1||target_>static_cast<int>(cells_.size()))throw std::runtime_error("No stack target is selected");
    return cells_[target_-1];
  }
  std::string plateId()const{return plate_enabled_?PREFIX+"plate":table_;}
  T plateOrigin()const {
    T location=T::Identity();location.linear()=Eigen::AngleAxisd(plate_yaw_,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    location.translation()=Eigen::Vector3d(plate_center_.x(),plate_center_.y(),plate_bottom_z_)-
      location.linear()*Eigen::Vector3d(.210,.1485,0.);
    return location;
  }
  moveit_msgs::msg::CollisionObject plateObject()const {
    std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(plate_uri_,Eigen::Vector3d::Constant(.001)));
    if(!mesh||!mesh->vertex_count)throw std::runtime_error("Cannot load the measured plate collision mesh: "+plate_uri_);
    shapes::ShapeMsg message;shapes::constructMsgFromShape(mesh.get(),message);
    moveit_msgs::msg::CollisionObject out;out.id=plateId();out.header.frame_id=frame_;out.operation=out.ADD;
    out.pose.orientation.w=1.;out.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(message));
    out.mesh_poses.push_back(pose(plateOrigin()));return out;
  }
  std::string blockId(int id)const{return PREFIX+"block_"+std::to_string(id);}
  T at(const Eigen::Vector2d& xy,double z)const {T out=center_;out.translation()=Eigen::Vector3d(xy.x(),xy.y(),z);return out;}
  moveit_msgs::msg::CollisionObject box(const std::string& id,const Eigen::Vector3d& size,const T& location)const {
    moveit_msgs::msg::CollisionObject out;out.id=id;out.header.frame_id=frame_;out.operation=out.ADD;
    out.pose.orientation.w=1.0;
    shape_msgs::msg::SolidPrimitive solid;solid.type=solid.BOX;solid.dimensions={size.x(),size.y(),size.z()};
    out.primitives.push_back(solid);out.primitive_poses.push_back(pose(location));return out;
  }
  moveit_msgs::msg::PlanningScene makeInitialScene() {
    auto cleanup=diff();
    const auto old=scene_->getWorld()->getObjectIds();
    for(auto& id:old)if(id.rfind(PREFIX,0)==0) {
      moveit_msgs::msg::CollisionObject remove;remove.id=id;remove.operation=remove.REMOVE;cleanup.world.collision_objects.push_back(remove);
    }
    std::vector<const moveit::core::AttachedBody*> attached;scene_->getCurrentState().getAttachedBodies(attached);
    for(const auto* body:attached)if(body->getName().rfind(PREFIX,0)==0) {
      moveit_msgs::msg::AttachedCollisionObject remove;remove.object.id=body->getName();remove.object.operation=remove.object.REMOVE;
      cleanup.robot_state.attached_collision_objects.push_back(remove);
      moveit_msgs::msg::CollisionObject world_remove;world_remove.id=body->getName();world_remove.operation=world_remove.REMOVE;
      cleanup.world.collision_objects.push_back(world_remove);
    }
    // REMOVE also deletes ACM entries. Clear old objects before adding the new
    // objects/matrix, including when a layout reuses the same cell IDs.
    scene_->setPlanningSceneDiffMsg(cleanup);apply(cleanup);
    auto update=diff();auto acm=scene_->getAllowedCollisionMatrix();
    acm.setEntry(table_,arm_group_->getUpdatedLinkModelNames(),false);
    if(empty_hand_test_) {
      // The SRDF also excludes several gripper/table pairs. Restore every
      // moving descendant's table check, retaining the fixed base mounting
      // contacts. A link moves if any ancestor joint has a variable.
      for(const auto* link:model_->getLinkModels()) {
        for(const auto* ancestor=link;ancestor;) {
          const auto* joint=ancestor->getParentJointModel();
          if(!joint)break;
          if(joint->getVariableCount()>0) {
            acm.setEntry(table_,link->getName(),false);break;
          }
          ancestor=joint->getParentLinkModel();
        }
      }
    }
    if(mode_!="reset"&&plate_enabled_) {
      update.world.collision_objects.push_back(plateObject());acm.setEntry(table_,plateId(),true);
      moveit_msgs::msg::ObjectColor color;color.id=plateId();color.color.a=1.;
      color.color.r=.65;color.color.g=.68;color.color.b=.72;update.object_colors.push_back(color);
    }
    if(mode_!="reset")for(int id=1;id<=columns_*rows_;++id) {
      if((empty_xy_scan_||plate_alignment_)&&!obstacles_.count(id))continue;
      if(empty_.count(id))continue;
      // Empty-target profiles omit this cube. The regular selected pocket has
      // a foundation block which remains in place while sources are stacked.
      if(!independent_target_&&!target_has_foundation_&&id==target_&&!obstacles_.count(id))continue;
      bool obstacle=obstacles_.count(id);Eigen::Vector3d size=obstacle?obstacle_:block_;
      T location=T::Identity();location.linear()=rotation_;location.translation()=Eigen::Vector3d(cells_[id-1].x(),cells_[id-1].y(),(obstacle?obstacle_surface_z_:source_surface_z_)+size.z()/2);
      std::string name=obstacle?PREFIX+"obstacle_"+std::to_string(id):blockId(id);
      update.world.collision_objects.push_back(box(name,size,location));acm.setEntry(table_,name,true);
      if(plate_enabled_)acm.setEntry(plateId(),name,true);
      moveit_msgs::msg::ObjectColor color;color.id=name;color.color.a=1;
      color.color.r=obstacle?.9f:(id==target_?1.f:.15f);color.color.g=obstacle?.12f:(id==target_?.65f:.55f);color.color.b=obstacle?.1f:(id==target_?.1f:.85f);
      update.object_colors.push_back(color);
    }
    acm.getMessage(update.allowed_collision_matrix);return update;
  }
  std::vector<double> recipeJoints(const State& state)const {
    std::vector<double> values;
    for(const auto& name:arm_group_->getActiveJointModelNames())values.push_back(state.getVariablePosition(name));
    values.push_back(state.getVariablePosition(hand_joint_));return values;
  }
  std::string recipePhase(const std::string& label,bool hand,bool event=false)const {
    if(event) {
      if(label.rfind("ATTACH ",0)==0)return "attach";
      if(label.rfind("PLACE ",0)==0)return "place";
      return "scene";
    }
    if(hand) {
      if(label.rfind("Close on block ",0)==0)return "grasp";
      if(label.find("first opening")!=std::string::npos)return "release_first";
      if(label.find("clearance opening before retreat")!=std::string::npos)return "release_clearance";
      if(label.rfind("Release block ",0)==0)return "release";
      return "open";
    }
    if(label.rfind("Lift block ",0)==0)return "lift";
    if(label.rfind("Retreat from layer ",0)==0)return "retreat";
    if(label.find(" descend")!=std::string::npos||label.find("horizontal insert")!=std::string::npos)return "approach";
    return "transfer";
  }
  void commit(const moveit_msgs::msg::PlanningScene& update,const std::string& label) {
    Step event;event.label=label;event.event=true;event.diff=update;event.phase=recipePhase(label,false,true);
    if(!recipe_file_.empty()) {
      scene_->getPlanningSceneMsg(event.before_scene);
      event.start_joints=recipeJoints(scene_->getCurrentState());
    }
    scene_->setPlanningSceneDiffMsg(update);
    if(!recipe_file_.empty()) {
      scene_->getPlanningSceneMsg(event.after_scene);
      event.end_joints=recipeJoints(scene_->getCurrentState());
    }
    steps_.push_back(std::move(event));
  }
  void contacts(const std::string& id,bool allowed) {
    auto update=diff();auto acm=scene_->getAllowedCollisionMatrix();acm.setEntry(id,touch_,allowed);acm.getMessage(update.allowed_collision_matrix);commit(update,"Grasp contact permissions");
  }
  void tableContact(const std::string& id,bool allowed) {
    auto update=diff();auto acm=scene_->getAllowedCollisionMatrix();acm.setEntry(id,table_,allowed);
    if(plate_enabled_)acm.setEntry(id,plateId(),allowed);
    acm.getMessage(update.allowed_collision_matrix);commit(update,"Carried block/support collision checking");
  }
  void attach(const std::string& id,int source) {
    T cube=T::Identity();cube.linear()=rotation_;
    cube.translation()=Eigen::Vector3d(cells_[source-1].x(),cells_[source-1].y(),source_surface_z_+block_.z()/2);
    State state(scene_->getCurrentState());state.update();
    carry_fixed_=(world_frame_.inverse()*state.getGlobalLinkTransform(tcp_)).inverse()*cube;
    auto update=diff();moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name=tcp_;attached.touch_links=touch_;attached.object.id=id;attached.object.operation=attached.object.ADD;
    // No geometry means MoveIt attaches the existing world object in its current pose.
    update.robot_state.attached_collision_objects.push_back(attached);commit(update,"ATTACH "+id);
  }
  void release(const std::string& id,int layer) {
    auto update=diff();moveit_msgs::msg::AttachedCollisionObject remove;remove.link_name=tcp_;remove.object.id=id;remove.object.operation=remove.object.REMOVE;
    update.robot_state.attached_collision_objects.push_back(remove);
    State state(scene_->getCurrentState());state.update();
    T actual=world_frame_.inverse()*state.getGlobalLinkTransform(tcp_)*carry_fixed_;
    if((actual.translation()-placed_cube_.translation()).norm()>.001 ||
       Eigen::AngleAxisd(actual.linear()*placed_cube_.linear().transpose()).angle()>.01)
      throw std::runtime_error("Placed block differs from its attached pose");
    update.world.collision_objects.push_back(box(id,block_,placed_cube_));
    moveit_msgs::msg::ObjectColor color;color.id=id;color.color.a=1;color.color.r=.2;color.color.g=.75;color.color.b=.4;update.object_colors.push_back(color);
    commit(update,"PLACE "+id+" layer="+std::to_string(layer));
  }
  bool valid(State& state)const {
    state.update();
    if(!state.satisfiesBounds(arm_group_)||scene_->isStateColliding(state,"",false))return false;
    return !(empty_xy_scan_||plate_alignment_)||!scan_floor_check_||minimumGripperZ(state)>=scan_floor_-1e-9;
  }
  bool solve(const T& target,State& state) {
    T goal=world_frame_*target*tip_tcp_.inverse();
    return state.setFromIK(arm_group_,goal,tip_,.12,[this](State* s,const moveit::core::JointModelGroup* group,const double* q){s->setJointGroupPositions(group,q);return valid(*s);});
  }
  T chooseYaw(const T& base,const std::string& label) {
    for(int turn=0;turn<4;++turn) {
      T target=base;target.linear()=Eigen::AngleAxisd(turn*PI/2.,Eigen::Vector3d::UnitZ()).toRotationMatrix()*base.linear();
      State seed(scene_->getCurrentState());
      if(solve(target,seed)) {RCLCPP_INFO(logger_,"STACK_YAW degrees=%d label=%s",turn*90,label.c_str());return target;}
    }
    throw std::runtime_error("No collision-free vertical grasp orientation: "+label);
  }
  std::pair<T,T> planApproach(const T& base,double pre_z,const std::string& label) {
    State original(scene_->getCurrentState());const auto count=steps_.size();
    std::vector<double> heights{pre_z};
    // Keep successful home-height approaches; if blocked, shorten the vertical
    // segment and let collision-aware free transit reach that lower pre-grasp.
    for(double clearance:{.06,.10,.16}) {
      double z=base.translation().z()+clearance;
      if(z<pre_z-1e-6)heights.push_back(z);
    }
    for(double approach_z:heights)for(int reverse=0;reverse<2;++reverse)for(int turn=0;turn<4;++turn) {
      checkStop();scene_->setCurrentState(original);steps_.resize(count);
      T low=base;low.linear()=Eigen::AngleAxisd(turn*PI/2.,Eigen::Vector3d::UnitZ()).toRotationMatrix()*base.linear();
      T pre=low;pre.translation().z()=approach_z;
      try {
        State grasp(original);if(!solve(low,grasp))continue;
        if(!reverse){planFree(pre,label+" approach");planStraight(low,label+" descend");}
        else {
          // Solve the approach backwards from a valid grasp, then ask OMPL for
          // that exact pre-grasp posture instead of changing IK branches.
          scene_->setCurrentState(grasp);robot_trajectory::RobotTrajectory up(model_,group_);
          if(!cartesian({pre},up))continue;
          State ready(up.getLastWayPoint());ready.update();scene_->setCurrentState(original);
          planJoint(ready,pre,label+" approach (reverse IK)");
          up.reverse();save(up,label+" descend");
        }
        RCLCPP_INFO(logger_,"STACK_YAW degrees=%d reverse_ik=%d pre_z=%.4f label=%s",turn*90,reverse,approach_z,label.c_str());
        return {low,pre};
      }catch(const std::exception& error){
        checkStop();
        RCLCPP_WARN(logger_,"STACK_APPROACH_RETRY yaw=%d reverse=%d pre_z=%.4f label=%s reason=%s",turn*90,reverse,approach_z,label.c_str(),error.what());
      }
    }
    scene_->setCurrentState(original);steps_.resize(count);
    throw std::runtime_error("No collision-free vertical approach: "+label);
  }
  void supportContact(const std::string& object,const std::string& support) {
    auto update=diff();auto acm=scene_->getAllowedCollisionMatrix();acm.setEntry(object,support,true);
    acm.getMessage(update.allowed_collision_matrix);commit(update,"Allow placement support contact");
  }
  std::vector<std::pair<T,T>> sideCandidates(int layer,const T& carry)const {
    std::vector<std::pair<T,T>> result;
    // A cube can turn through any of its 24 proper symmetries and still rest
    // flat. Derive TCP from the attached cube, never teleport it on release.
    std::array<int,3> axes{0,1,2};
    do {for(int sx:{-1,1})for(int sy:{-1,1})for(int sz:{-1,1}) {
      Eigen::Matrix3d symmetry=Eigen::Matrix3d::Zero();
      symmetry(axes[0],0)=sx;symmetry(axes[1],1)=sy;symmetry(axes[2],2)=sz;
      if(symmetry.determinant()<.5)continue;
      T cube=T::Identity();cube.linear()=rotation_*symmetry;
      cube.translation()=Eigen::Vector3d(targetXY().x(),targetXY().y(),stack_surface_z_+block_.z()*(layer-.5));
      T tcp=cube*carry.inverse();Eigen::Matrix3d approach=tcp.linear()*tcp_approach_.linear();
      if(std::abs(approach(2,2))>.01||std::abs(approach(2,0))>.01)continue;
      T pre=tcp;pre.translation()-=side_clearance_*approach.col(2);
      result.emplace_back(tcp,pre);
    }}while(std::next_permutation(axes.begin(),axes.end()));
    Eigen::Vector2d base=(world_frame_.inverse()*scene_->getCurrentState().getGlobalLinkTransform("base_link")).translation().head<2>();
    Eigen::Vector2d outward=targetXY()-base;
    if(outward.norm()<1e-8)outward=Eigen::Vector2d::UnitX();outward.normalize();
    std::stable_sort(result.begin(),result.end(),[&](const auto& a,const auto& b){
      return (a.first.linear()*tcp_approach_.linear()).col(2).head(2).dot(outward)>
             (b.first.linear()*tcp_approach_.linear()).col(2).head(2).dot(outward);
    });return result;
  }
  bool usesVerticalPlacement(int layer)const {
    // Layer numbers are physical tower layers, with or without an existing
    // foundation. Ten 25mm cubes reach 250mm above their support surface.
    return placement_mode_=="vertical"||layer<=vertical_placement_layers_;
  }
  std::pair<T,T> planPlacement(const std::string& object,const std::string& support,int layer,const std::string& label) {
    State original(scene_->getCurrentState());auto acm=scene_->getAllowedCollisionMatrix();const auto count=steps_.size();
    const bool vertical=usesVerticalPlacement(layer);
    if(!vertical)for(const auto& candidate:sideCandidates(layer,carry_fixed_)) {
      checkStop();scene_->setCurrentState(original);scene_->getAllowedCollisionMatrixNonConst()=acm;steps_.resize(count);
      try {
        State probe(original);if(!solve(candidate.second,probe))continue;
        planFree(candidate.second,label+" side approach");
        supportContact(object,support); // Only the final insertion may touch its support.
        planStraight(candidate.first,label+" horizontal insert");
        placed_cube_=candidate.first*carry_fixed_;
        RCLCPP_INFO(logger_,"STACK_PLACEMENT layer=%d mode=side approach_xyz=%.3f,%.3f,%.3f",layer,
          (candidate.first.linear()*tcp_approach_.linear())(0,2),
          (candidate.first.linear()*tcp_approach_.linear())(1,2),
          (candidate.first.linear()*tcp_approach_.linear())(2,2));
        return candidate;
      }catch(const std::exception& error){checkStop();RCLCPP_WARN(logger_,"STACK_SIDE_RETRY layer=%d reason=%s",layer,error.what());}
    }
    scene_->setCurrentState(original);scene_->getAllowedCollisionMatrixNonConst()=acm;steps_.resize(count);
    if(!vertical&&layer>vertical_fallback_)throw std::runtime_error("No collision-free side placement: "+label);
    supportContact(object,support);
    T place=at(targetXY(),stack_surface_z_+block_.z()*(layer-1)+grip_height_);
    auto result=planApproach(place,place.translation().z()+clearance_,label);
    placed_cube_=result.first*carry_fixed_;
    RCLCPP_INFO(logger_,"STACK_PLACEMENT layer=%d mode=vertical",layer);return result;
  }
  void checkTowerReach(const State& measured,int required) {
    State seed(measured);seed.setToDefaultValues(arm_group_,parameter<std::string>(node_,"home_pose","home"));
    if(!empty_hand_test_)seed.setJointGroupActivePositions(gripper_group_,std::vector<double>{open_});
    seed.update();
    T nominal_cube=T::Identity();nominal_cube.linear()=rotation_;
    nominal_cube.translation()=Eigen::Vector3d(center_.translation().x(),center_.translation().y(),source_surface_z_+block_.z()/2);
    T nominal_tcp=at(center_.translation().head<2>(),source_surface_z_+grip_height_);
    T carry=nominal_tcp.inverse()*nominal_cube;
    int maximum=foundationLayers();
    for(int layer=foundationLayers()+1;layer<=required;++layer) {
      checkStop();T target=at(targetXY(),stack_surface_z_+block_.z()*(layer-1)+grip_height_+clearance_);
      bool reached=false;
      const bool vertical=usesVerticalPlacement(layer);
      if(!vertical)for(const auto& candidate:sideCandidates(layer,carry)) {
        State probe(seed);if(solve(candidate.second,probe)&&solve(candidate.first,probe)){seed=probe;reached=true;break;}
      }
      if(!reached&&(vertical||layer<=vertical_fallback_))for(int turn=0;turn<4;++turn) {
        T candidate=target;candidate.linear()=Eigen::AngleAxisd(turn*PI/2.,Eigen::Vector3d::UnitZ()).toRotationMatrix()*target.linear();
        if(solve(candidate,seed)){reached=true;break;}
      }
      if(!reached) {
        State probe(seed);T goal=world_frame_*target*tip_tcp_.inverse();
        if(probe.setFromIK(arm_group_,goal,tip_,.5)) {
          probe.update();scene_->isStateColliding(probe,"",true);
          RCLCPP_WARN(logger_,"STACK_REACH_FAILURE layer=%d has geometric IK but no collision-free IK was found",layer);
        }
        break;
      }
      maximum=layer;
    }
    RCLCPP_INFO(logger_,"STACK_HEIGHT required=%d reachable=%d target=%d",required,maximum,target_);
    if(maximum<required)throw std::runtime_error("Tower needs "+std::to_string(required)+" layers, but target cell "+std::to_string(target_)+" can reach only "+std::to_string(maximum)+" with this board position. Move the A3 board toward the robot or select another target; NOTHING executed");
  }
  bool cartesian(const std::vector<T>& targets,robot_trajectory::RobotTrajectory& output) {
    checkStop();State start(scene_->getCurrentState());start.update();
    if(!valid(start))return false;
    const T start_tcp=world_frame_.inverse()*start.getGlobalLinkTransform(tcp_);
    std::vector<double> previous_ik;start.copyJointGroupPositions(arm_group_,previous_ik);
    double largest_jump=0.,jump_before=0.,jump_after=0.;std::string jump_joint;
    int rejected=0;
    EigenSTL::vector_Isometry3d goals;for(auto& target:targets)goals.push_back(world_frame_*target*tip_tcp_.inverse());
    std::vector<moveit::core::RobotStatePtr> states;
    double fraction=moveit::core::CartesianInterpolator::computeCartesianPath(&start,arm_group_,states,model_->getLinkModel(tip_),goals,true,
      moveit::core::MaxEEFStep(step_),moveit::core::JumpThreshold(5.),
      [this,&previous_ik,&largest_jump,&jump_before,&jump_after,&jump_joint,&rejected]
      (State* s,const moveit::core::JointModelGroup* group,const double* q) {
        s->setJointGroupPositions(group,q);
        if(!valid(*s)) {
          if(++rejected<=2) {
            collision_detection::CollisionRequest request;collision_detection::CollisionResult result;
            request.contacts=true;request.max_contacts=3;scene_->checkCollision(request,result,*s);
            std::string contacts;
            for(const auto& contact:result.contacts)contacts+=contact.first.first+"/"+contact.first.second+" ";
            RCLCPP_WARN(logger_,"STACK_CARTESIAN_REJECT bounds=%d collision=%d floor_check=%d contacts=%s",
              s->satisfiesBounds(arm_group_),result.collision,scan_floor_check_,contacts.c_str());
          }
          return false;
        }
        std::vector<double> candidate;s->copyJointGroupPositions(arm_group_,candidate);
        for(std::size_t i=0;i<candidate.size();++i)if(std::abs(candidate[i]-previous_ik[i])>largest_jump) {
          largest_jump=std::abs(candidate[i]-previous_ik[i]);jump_joint=arm_group_->getVariableNames()[i];
          jump_before=previous_ik[i];jump_after=candidate[i];
        }
        previous_ik=std::move(candidate);return true;
      });
    if(fraction<1.-1e-6||states.empty()||largest_jump>.5) {
      const auto& target=targets.back();
      RCLCPP_WARN(logger_,"STACK_CARTESIAN_INCOMPLETE fraction=%.9f states=%zu rejected=%d max_joint_delta=%.9f joint=%s from=%.9f to=%.9f start_tcp=%.9f,%.9f,%.9f target_tcp=%.9f,%.9f,%.9f",
        fraction,states.size(),rejected,largest_jump,jump_joint.c_str(),jump_before,jump_after,
        start_tcp.translation().x(),start_tcp.translation().y(),start_tcp.translation().z(),
        target.translation().x(),target.translation().y(),target.translation().z());
    }
    if(fraction<1.-1e-6||states.empty())return false;
    for(auto& s:states)output.addSuffixWayPoint(s,0.0);return true;
  }
  void save(robot_trajectory::RobotTrajectory& trajectory,const std::string& label,bool hand=false) {
    checkStop();
    if(empty_hand_test_) {
      if(hand)throw std::runtime_error("A gripper trajectory appeared in the empty-hand plan");
      for(std::size_t i=0;i<trajectory.getWayPointCount();++i) {
        double position=trajectory.getWayPoint(i).getVariablePosition(hand_joint_);
        if(!std::isfinite(position)||std::abs(position-frozen_hand_position_)>1e-9)
          throw std::runtime_error("Gripper geometry changed in empty-hand path: "+label);
      }
    }
    // Staging routes may start with an already-reached waypoint. Consecutive
    // duplicate states yield equal timestamps, rejected by JointTrajectoryController.
    robot_trajectory::RobotTrajectory unique(model_,hand?hand_group_:group_);
    for(std::size_t i=0;i<trajectory.getWayPointCount();++i)
      if(unique.empty()||unique.getLastWayPoint().distance(trajectory.getWayPoint(i),hand?gripper_group_:arm_group_)>1e-8)
        unique.addSuffixWayPoint(trajectory.getWayPoint(i),0.0);
    if(unique.empty())throw std::runtime_error("Empty trajectory: "+label);
    if(unique.getWayPointCount()==1)unique.addSuffixWayPoint(unique.getFirstWayPoint(),.1);
    trajectory.swap(unique);
    const std::string phase=recipePhase(label,hand);
    const bool split=split_speeds_&&!empty_hand_test_&&!plate_alignment_;
    // Only final grasp/place entry uses the lower velocity and acceleration.
    // Lifting, retreating, reorientation and transport use the faster profile.
    const bool near_contact=phase=="approach";
    const double velocity=hand?.4:(split?(near_contact?approach_vel_:transfer_vel_):vel_);
    const double acceleration=hand?.4:(split?(near_contact?approach_acc_:transfer_acc_):acc_);
    trajectory_processing::IterativeParabolicTimeParameterization time;
    if(!time.computeTimeStamps(trajectory,velocity,acceleration))throw std::runtime_error("Timing failed: "+label);
    for(std::size_t i=1;i<trajectory.getWayPointCount();++i)
      if(trajectory.getWayPointDurationFromPrevious(i)<.001)trajectory.setWayPointDurationFromPrevious(i,.001);
    if(trajectory.getDuration()<.1)trajectory.setWayPointDurationFromPrevious(trajectory.getWayPointCount()-1,.1);
    if(!recipe_file_.empty()||plate_alignment_) {
      // Every exported command is an independently reviewable stationary
      // boundary. The physical executor additionally samples its exact quintic
      // interpolation before approving any actuator command.
      const auto* joints=hand?gripper_group_:arm_group_;
      const std::vector<double> zero(joints->getVariableCount(),0.);
      for(auto index:{std::size_t(0),trajectory.getWayPointCount()-1}) {
        trajectory.getWayPointPtr(index)->setJointGroupVelocities(joints,zero);
        trajectory.getWayPointPtr(index)->setJointGroupAccelerations(joints,zero);
      }
    }
    // Check intermediate controller samples as well as geometric path vertices.
    State previous(trajectory.getFirstWayPoint());previous.update();
    double verified_minimum=std::numeric_limits<double>::infinity();
    for(std::size_t i=0;i<trajectory.getWayPointCount();++i) {
      trajectory.getWayPointPtr(i)->update();const State& next=trajectory.getWayPoint(i);
      if(!hand)for(const auto& name:arm_group_->getVariableNames())
        if(std::abs(previous.getVariablePosition(name)-next.getVariablePosition(name))>.5)
          throw std::runtime_error("Joint discontinuity in path: "+label);
      int count=std::max(1,static_cast<int>(std::ceil(previous.distance(next,hand?gripper_group_:arm_group_)/.01)));
      for(int k=1;k<=count;++k) {State interpolated(previous);previous.interpolate(next,static_cast<double>(k)/count,interpolated);
        if(!valid(interpolated))throw std::runtime_error("Collision or gripper-clearance violation in timed path: "+label);
        if((empty_xy_scan_||plate_alignment_)&&scan_floor_check_)verified_minimum=std::min(verified_minimum,minimumGripperZ(interpolated));}
      previous=next;
    }
    Step step;step.label=label;step.hand=hand;step.phase=phase;
    step.velocity_scaling=velocity;step.acceleration_scaling=acceleration;
    trajectory.getRobotTrajectoryMsg(step.trajectory);
    if(!recipe_file_.empty()) {
      scene_->getPlanningSceneMsg(step.before_scene);
      step.start_joints=recipeJoints(trajectory.getFirstWayPoint());
      step.end_joints=recipeJoints(trajectory.getLastWayPoint());
    }
    if((empty_xy_scan_||plate_alignment_)&&scan_floor_check_)scan_minimum_z_=std::min(scan_minimum_z_,verified_minimum);
    scene_->setCurrentState(trajectory.getLastWayPoint());
    if(!recipe_file_.empty())scene_->getPlanningSceneMsg(step.after_scene);
    steps_.push_back(std::move(step));
  }
  void planStraight(const T& target,const std::string& label) {
    robot_trajectory::RobotTrajectory path(model_,group_);
    if(!cartesian({target},path))throw std::runtime_error("Straight approach/retreat blocked: "+label);
    save(path,label);
  }
  void planFree(const T& target,const std::string& label) {
    State current(scene_->getCurrentState());current.update();T start=world_frame_.inverse()*current.getGlobalLinkTransform(tcp_);
    std::vector<std::vector<T>> routes{{target}};
    double high=std::max({start.translation().z(),target.translation().z(),travel_z_});
    T up=start,across=target;up.translation().z()=high;across.translation().z()=high;routes.push_back({up,across,target});
    // Side staging lets the arm approach a high tower without lifting a distant source beyond its reach.
    Eigen::Vector2d near=start.translation().head<2>();
    if(target.translation().z()>start.translation().z())near=target.translation().head<2>();
    double toward=std::atan2(-near.y(),-near.x());
    for(double radius:{.10,.14})for(int direction=0;direction<8;++direction) {
      double angle=toward+direction*PI/4.;Eigen::Vector2d side=near+radius*Eigen::Vector2d(std::cos(angle),std::sin(angle));
      routes.push_back({at(side,start.translation().z()),at(side,target.translation().z()),target});
    }
    for(std::size_t index=0;index<routes.size();++index) {
      robot_trajectory::RobotTrajectory path(model_,group_);
      if(cartesian(routes[index],path)) {
        try {
          save(path,label);
          RCLCPP_INFO(logger_,"STACK_ROUTE strategy=%zu label=%s",index,label.c_str());return;
        }catch(const std::exception& error){
          checkStop();scene_->setCurrentState(current);
          RCLCPP_WARN(logger_,"STACK_ROUTE_RETRY strategy=%zu label=%s reason=%s",index,label.c_str(),error.what());
        }
      }
    }
    // OMPL searches around world obstacles and the carried cube if Cartesian staging is insufficient.
    State goal(current);if(!solve(target,goal))throw std::runtime_error("Unreachable transfer target: "+label);
    planJoint(goal,target,label);
  }
  void planNeutral(const std::string& label) {
    State original(scene_->getCurrentState());original.update();const auto count=steps_.size();
    Eigen::Vector2d center=center_.translation().head<2>();
    Eigen::Vector2d away=center-targetXY();
    if(away.norm()<1e-8)away=-center;if(away.norm()<1e-8)away=-Eigen::Vector2d::UnitX();away.normalize();
    double angle=std::atan2(away.y(),away.x());std::vector<Eigen::Vector2d> candidates{center};
    for(double radius:{.08,.12,.16})for(int turn=0;turn<8;++turn) {
      double direction=angle+turn*PI/4.;candidates.push_back(center+radius*Eigen::Vector2d(std::cos(direction),std::sin(direction)));
    }
    for(const auto& xy:candidates) {
      checkStop();scene_->setCurrentState(original);steps_.resize(count);
      T target=at(xy,travel_z_);State probe(original);if(!solve(target,probe))continue;
      try {
        planFree(target,label);
        RCLCPP_INFO(logger_,"STACK_NEUTRAL xyz=%.4f,%.4f,%.4f label=%s",xy.x(),xy.y(),travel_z_,label.c_str());return;
      }catch(const std::exception& error){checkStop();RCLCPP_WARN(logger_,"STACK_NEUTRAL_RETRY reason=%s",error.what());}
    }
    scene_->setCurrentState(original);steps_.resize(count);
    throw std::runtime_error("No clear upright staging pose after placement: "+label);
  }
  void planJoint(const State& goal,const T& target,const std::string& label) {
    State current(scene_->getCurrentState());current.update();T start=world_frame_.inverse()*current.getGlobalLinkTransform(tcp_);
    planning_interface::MotionPlanRequest request;request.group_name=group_;
    request.allowed_planning_time=5.;request.num_planning_attempts=2;
    moveit::core::robotStateToRobotStateMsg(current,request.start_state);
    request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal,arm_group_,.0001));
    moveit_msgs::msg::OrientationConstraint orientation;orientation.header.frame_id=frame_;orientation.link_name=approach_;
    orientation.orientation=pose(target*tcp_approach_).orientation;
    orientation.absolute_x_axis_tolerance=.005;orientation.absolute_y_axis_tolerance=.005;
    orientation.absolute_z_axis_tolerance=PI;orientation.weight=1;
    const Eigen::Matrix3d start_approach=world_frame_.linear().transpose()*current.getGlobalLinkTransform(approach_).linear();
    const Eigen::Matrix3d goal_approach=target.linear()*tcp_approach_.linear();
    std::vector<const moveit::core::AttachedBody*> attached;
    current.getAttachedBodies(attached);
    const bool carrying=std::any_of(attached.begin(),attached.end(),[](const auto* body){return body->getName().rfind(PREFIX,0)==0;});
    // A free empty gripper may tilt around obstacles. Only a carried block
    // needs the upright transfer constraint; the final Cartesian grasp path
    // still has a fixed vertical orientation in either case.
    if(carrying&&start_approach.col(2).dot(-Eigen::Vector3d::UnitZ())>std::cos(.005)&&
       start_approach.col(2).dot(goal_approach.col(2))>std::cos(.005))
      request.path_constraints.orientation_constraints.push_back(orientation);
    planning_interface::MotionPlanResponse response;
    if(!pipeline_->generatePlan(scene_,request,response)||response.error_code_.val!=moveit_msgs::msg::MoveItErrorCodes::SUCCESS||!response.trajectory_)
      throw std::runtime_error("Collision-free transfer planning failed: "+label);
    // OMPL vertices can be far apart along a valid continuous joint-space edge.
    // Subdivide those edges before timing, retaining collision checks on every
    // sample. Cartesian IK paths keep their independent jump rejection in save().
    robot_trajectory::RobotTrajectory dense(model_,group_);
    dense.addSuffixWayPoint(response.trajectory_->getFirstWayPoint(),0.);
    for(std::size_t i=1;i<response.trajectory_->getWayPointCount();++i) {
      const State& a=response.trajectory_->getWayPoint(i-1);const State& b=response.trajectory_->getWayPoint(i);
      double largest=0.;for(const auto& name:arm_group_->getVariableNames())
        largest=std::max(largest,std::abs(a.getVariablePosition(name)-b.getVariablePosition(name)));
      int count=std::max(1,static_cast<int>(std::ceil(largest/.05)));
      for(int k=1;k<=count;++k){State sample(a);a.interpolate(b,static_cast<double>(k)/count,sample);sample.update();dense.addSuffixWayPoint(sample,0.);}
    }
    RCLCPP_INFO(logger_,"STACK_ROUTE strategy=OMPL label=%s",label.c_str());save(dense,label);
  }
  void planHand(double goal,const std::string& label) {
    // Empty-hand previews must use the actual, stationary gripper geometry.
    // Skipping only the eventual hand goal would otherwise leave the arm path
    // checked against an opening/closing gripper that will not move in reality.
    if(empty_hand_test_) {
      if(std::abs(scene_->getCurrentState().getVariablePosition(hand_joint_)-frozen_hand_position_)>1e-9)
        throw std::runtime_error("Frozen gripper position changed during empty-hand planning");
      return;
    }
    State state(scene_->getCurrentState());double start=state.getVariablePosition(hand_joint_);
    robot_trajectory::RobotTrajectory path(model_,hand_group_);int count=std::max(1,static_cast<int>(std::ceil(std::abs(goal-start)/.01)));
    for(int i=0;i<=count;++i) {state.setJointGroupActivePositions(gripper_group_,std::vector<double>{start+(goal-start)*i/count});state.update();
      if(!state.satisfiesBounds(gripper_group_)||!valid(state))throw std::runtime_error("Gripper collision/limit: "+label);
      path.addSuffixWayPoint(state,0.0);}
    save(path,label,true);
  }
  void showPlan() {
    moveit_msgs::msg::DisplayTrajectory display;display.model_id=model_->getName();moveit::core::robotStateToRobotStateMsg(*initial_state_,display.trajectory_start);
    if(empty_hand_test_) {
      const auto& arm_names=arm_group_->getActiveJointModelNames();
      const std::set<std::string> expected(arm_names.begin(),arm_names.end());
      if(arm_names.size()!=6||expected.size()!=6)
        throw std::runtime_error("Empty-hand preview requires exactly six active arm joints");
      if(std::abs(initial_state_->getVariablePosition(hand_joint_)-frozen_hand_position_)>1e-9)
        throw std::runtime_error("Empty-hand preview start does not match the frozen measured gripper");
      for(const auto& step:steps_) {
        if(step.hand)throw std::runtime_error("A gripper trajectory appeared in the empty-hand plan");
        if(step.event)continue; // Local virtual-block events are never execution commands.
        const auto& names=step.trajectory.joint_trajectory.joint_names;
        if(names.size()!=6||std::set<std::string>(names.begin(),names.end())!=expected||
           step.trajectory.joint_trajectory.points.empty()||
           !step.trajectory.multi_dof_joint_trajectory.joint_names.empty()||
           !step.trajectory.multi_dof_joint_trajectory.points.empty())
          throw std::runtime_error("Empty-hand preview contains a non-arm or empty trajectory: "+step.label);
        // Keep the exact per-step points and timing. The separate physical
        // executor validates and seals this artifact before it can be used.
        display.trajectory.push_back(step.trajectory);
        if(!display_file_.empty()) {
          const std::string segment="{\"index\":"+std::to_string(display.trajectory.size())+
            ",\"label\":"+jsonString(step.label)+"}";
          RCLCPP_INFO(logger_,"STACK_PATH_SEGMENT %s",segment.c_str());
        }
      }
      if(display.trajectory.empty())throw std::runtime_error("Empty-hand preview contains no arm path");
    } else {
      State current(*initial_state_);robot_trajectory::RobotTrajectory combined(model_);
      for(auto& step:steps_)if(!step.event) {
        robot_trajectory::RobotTrajectory part(model_,step.hand?hand_group_:group_);part.setRobotTrajectoryMsg(current,step.trajectory);
        for(std::size_t i=0;i<part.getWayPointCount();++i)combined.addSuffixWayPoint(part.getWayPoint(i),part.getWayPointDurationFromPrevious(i));
        current=part.getLastWayPoint();
      }
      display.trajectory.emplace_back();combined.getRobotTrajectoryMsg(display.trajectory.back());
    }
    if(!display_file_.empty()) {
      // Batch plans can contain tens of thousands of points. Persist the exact
      // preview before exiting, independently of DDS discovery/fragment delivery.
      rclcpp::SerializedMessage serialized;
      rclcpp::Serialization<moveit_msgs::msg::DisplayTrajectory> serializer;
      serializer.serialize_message(&display,&serialized);
      const auto& bytes=serialized.get_rcl_serialized_message();
      const std::string temporary=display_file_+".tmp";
      std::ofstream stream(temporary,std::ios::binary|std::ios::trunc);
      stream.write(reinterpret_cast<const char*>(bytes.buffer),static_cast<std::streamsize>(bytes.buffer_length));
      stream.close();
      if(!stream||std::rename(temporary.c_str(),display_file_.c_str())!=0)
        throw std::runtime_error("Cannot save planned trajectory: "+display_file_);
      RCLCPP_INFO(logger_,"STACK_PATH_SAVED bytes=%zu",bytes.buffer_length);
    }
    planned_display_=display;
    display_->publish(display);
  }
  template<class Message> void writeRecipeMessage(const std::filesystem::path& file,const Message& message)const {
    rclcpp::SerializedMessage serialized;rclcpp::Serialization<Message> serializer;
    serializer.serialize_message(&message,&serialized);
    const auto& bytes=serialized.get_rcl_serialized_message();
    const auto temporary=file.string()+".tmp";
    std::ofstream stream(temporary,std::ios::binary|std::ios::trunc);
    stream.write(reinterpret_cast<const char*>(bytes.buffer),static_cast<std::streamsize>(bytes.buffer_length));stream.close();
    if(!stream||std::rename(temporary.c_str(),file.c_str())!=0)
      throw std::runtime_error("Cannot persist full stacking recipe message: "+file.string());
  }
  void exportRecipe()const {
    if(recipe_file_.empty()||mode_!="plan"||empty_hand_test_||plate_alignment_||empty_xy_scan_||steps_.empty()||!initial_state_)
      throw std::runtime_error("A full non-executing stacking plan is required for recipe export");
    const std::filesystem::path manifest=std::filesystem::absolute(recipe_file_);
    const auto folder=manifest.parent_path();std::filesystem::create_directories(folder);
    const auto prefix=manifest.stem().string();
    if(prefix.empty())throw std::runtime_error("Recipe manifest needs a filename");
    const auto initial_scene_name=prefix+"_initial_scene.cdr";
    const auto initial_state_name=prefix+"_initial_state.cdr";
    writeRecipeMessage(folder/initial_scene_name,recipe_initial_scene_);
    moveit_msgs::msg::RobotState initial_state;moveit::core::robotStateToRobotStateMsg(*initial_state_,initial_state);
    writeRecipeMessage(folder/initial_state_name,initial_state);
    auto joint_names=arm_group_->getActiveJointModelNames();joint_names.push_back(hand_joint_);
    auto numbers=[](std::ostream& out,const std::vector<double>& values) {
      out<<'[';for(std::size_t i=0;i<values.size();++i){if(i)out<<',';out<<std::setprecision(17)<<values[i];}out<<']';
    };
    auto strings=[](std::ostream& out,const std::vector<std::string>& values) {
      out<<'[';for(std::size_t i=0;i<values.size();++i){if(i)out<<',';out<<jsonString(values[i]);}out<<']';
    };
    std::ostringstream out;out<<std::setprecision(17);
    out<<"{\"schema\":1,\"initial_scene_file\":"<<jsonString(initial_scene_name)
       <<",\"initial_state_file\":"<<jsonString(initial_state_name)<<",\"joint_names\":";
    strings(out,joint_names);out<<",\"initial_joints\":";numbers(out,recipeJoints(*initial_state_));
    out<<",\"target_cell\":"<<target_<<",\"foundation_layers\":"<<foundationLayers()
       <<",\"moves\":"<<total_<<",\"layers\":"<<total_+foundationLayers()
       <<",\"gripper_open_position\":"<<open_<<",\"gripper_grasp_position\":"<<close_
       <<",\"release_first_position\":"<<release_first_<<",\"release_clearance_position\":"<<release_clearance_
       <<",\"staged_release_enabled\":"<<(staged_release_?"true":"false")
       <<",\"approach_velocity_scaling\":"<<approach_vel_<<",\"transfer_velocity_scaling\":"<<transfer_vel_
       <<",\"stack_acceleration_scaling\":"<<stack_acc_
       <<",\"approach_acceleration_scaling\":"<<approach_acc_
       <<",\"transfer_acceleration_scaling\":"<<transfer_acc_<<",\"steps\":[";
    for(std::size_t index=0;index<steps_.size();++index) {
      const auto& step=steps_[index];
      if(step.start_joints.size()!=7||step.end_joints.size()!=7)
        throw std::runtime_error("Recipe step does not describe all six arm joints and the master finger");
      std::ostringstream id;id<<prefix<<'_'<<std::setw(4)<<std::setfill('0')<<index+1;
      const auto file=id.str()+".cdr",before=id.str()+"_before.cdr",after=id.str()+"_after.cdr";
      if(step.event)writeRecipeMessage(folder/file,step.diff);else writeRecipeMessage(folder/file,step.trajectory);
      writeRecipeMessage(folder/before,step.before_scene);writeRecipeMessage(folder/after,step.after_scene);
      if(index)out<<',';
      out<<"{\"index\":"<<index+1<<",\"kind\":"<<jsonString(step.event?"scene":step.hand?"hand":"arm")
         <<",\"phase\":"<<jsonString(step.phase)<<",\"label\":"<<jsonString(step.label)
         <<",\"file\":"<<jsonString(file)<<",\"scene_before_file\":"<<jsonString(before)
         <<",\"scene_after_file\":"<<jsonString(after)<<",\"joint_names\":";
      strings(out,joint_names);out<<",\"start_joints\":";numbers(out,step.start_joints);
      out<<",\"end_joints\":";numbers(out,step.end_joints);
      out<<",\"velocity_scaling\":"<<step.velocity_scaling<<",\"acceleration_scaling\":"<<step.acceleration_scaling
         <<",\"completed\":"<<step.completed;
      if(step.hand)out<<",\"hand_target\":"<<step.end_joints.back();
      if(!step.event) {
        const auto& duration=step.trajectory.joint_trajectory.points.back().time_from_start;
        out<<",\"duration\":"<<duration.sec+duration.nanosec*1e-9;
      }
      out<<'}';
    }
    out<<"]}";
    const auto temporary=manifest.string()+".tmp";
    std::ofstream stream(temporary,std::ios::binary|std::ios::trunc);stream<<out.str()<<'\n';stream.close();
    if(!stream||std::rename(temporary.c_str(),manifest.c_str())!=0)
      throw std::runtime_error("Cannot commit full stacking recipe manifest");
    RCLCPP_INFO(logger_,"STACK_RECIPE_SAVED file=%s steps=%zu grasp=%.3f release=%.3f,%.3f approach_velocity=%.3f transfer_velocity=%.3f approach_acceleration=%.3f transfer_acceleration=%.3f",
      manifest.c_str(),steps_.size(),close_,release_first_,release_clearance_,approach_vel_,transfer_vel_,approach_acc_,transfer_acc_);
  }
  void checkMockHardware() {
    auto client=node_->create_client<controller_manager_msgs::srv::ListHardwareComponents>("/controller_manager/list_hardware_components");
    if(!client->wait_for_service(std::chrono::seconds(5)))throw std::runtime_error("Simulation hardware status service is unavailable");
    auto future=client->async_send_request(std::make_shared<controller_manager_msgs::srv::ListHardwareComponents::Request>());
    if(future.wait_for(std::chrono::seconds(5))!=std::future_status::ready)throw std::runtime_error("Simulation hardware status timed out");
    std::set<std::string> covered;
    const auto response=future.get();
    for(const auto& hardware:response->component)if(hardware.class_type=="mock_components/GenericSystem"&&hardware.state.id==3)
      for(const auto& command:hardware.command_interfaces)if(command.is_claimed)covered.insert(command.name);
    auto names=arm_group_->getActiveJointModelNames();names.push_back(hand_joint_);
    for(const auto& name:names)if(!covered.count(name+"/position"))throw std::runtime_error("RViz stacking runs only on active mock_components/GenericSystem; physical hardware execution refused");
  }
  std::string approvePreview(const moveit_msgs::msg::DisplayTrajectory& preview,
      std::size_t index,std::size_t total,const std::string& label) {
    checkStop();
    double duration=0.;
    for(const auto& trajectory:preview.trajectory) {
      const auto& points=trajectory.joint_trajectory.points;
      if(points.empty())throw std::runtime_error("Cannot approve an empty planned motion");
      const auto& last=points.back().time_from_start;
      duration+=static_cast<double>(last.sec)+static_cast<double>(last.nanosec)*1e-9;
    }
    if(preview.trajectory.empty()||!std::isfinite(duration)||duration<=0.)
      throw std::runtime_error("Cannot approve an empty or invalid preview");
    const auto request_id=plan_id_+"_"+std::to_string(++approval_sequence_);
    const auto service_name="/ur5e_a3_stack/approve_next_"+request_id;
    struct Gate {std::atomic<bool> active{true},accepted{false};};
    auto gate=std::make_shared<Gate>();
    auto service=node_->create_service<std_srvs::srv::Trigger>(service_name,
      [this,gate](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> reply) {
        bool expected=false;
        reply->success=gate->active.load()&&!cancelled_.load()&&
          gate->accepted.compare_exchange_strong(expected,true);
        reply->message=reply->success?"Reviewed preview approved once":"Preview expired, stopped, or already approved";
      });
    std::ostringstream record;
    record<<"{\"plan_id\":"<<jsonString(plan_id_)<<",\"request_id\":"<<jsonString(request_id)
      <<",\"index\":"<<index<<",\"total\":"<<total<<",\"label\":"<<jsonString(label)
      <<",\"duration\":"<<std::setprecision(17)<<duration
      <<",\"approve_service\":"<<jsonString(service_name)<<"}";
    const auto data=record.str();
    display_->publish(preview);
    RCLCPP_INFO(logger_,"STACK_STEP_READY %s",data.c_str());
    auto published=std::chrono::steady_clock::now();
    try {
      while(!gate->accepted.load()) {
        checkStop();
        const auto now=std::chrono::steady_clock::now();
        if(now-published>=std::chrono::seconds(1)) {display_->publish(preview);published=now;}
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      checkStop();
    } catch(...) {gate->active=false;throw;}
    gate->active=false;service.reset();
    RCLCPP_INFO(logger_,"STACK_STEP_APPROVED %s",data.c_str());
    return data;
  }
  moveit_msgs::msg::DisplayTrajectory alignmentPreview(std::size_t first,std::size_t last,const State& start)const {
    if(first>=last||last>steps_.size())throw std::runtime_error("Invalid plate alignment preview range");
    moveit_msgs::msg::DisplayTrajectory display;display.model_id=model_->getName();
    moveit::core::robotStateToRobotStateMsg(start,display.trajectory_start);
    State current(start);robot_trajectory::RobotTrajectory combined(model_);
    for(std::size_t index=first;index<last;++index) {
      const auto& step=steps_[index];
      if(step.event)throw std::runtime_error("Plate alignment preview cannot contain scene events");
      robot_trajectory::RobotTrajectory part(model_,step.hand?hand_group_:group_);
      part.setRobotTrajectoryMsg(current,step.trajectory);
      for(std::size_t point=0;point<part.getWayPointCount();++point)
        combined.addSuffixWayPoint(part.getWayPoint(point),part.getWayPointDurationFromPrevious(point));
      current=part.getLastWayPoint();
    }
    display.trajectory.emplace_back();combined.getRobotTrajectoryMsg(display.trajectory.back());
    return display;
  }
  void execute() {
    int completed=0;status("RUNNING",0);
    std::size_t motion_count=0;
    for(const auto& step:steps_)if(!step.event)++motion_count;
    if(!motion_count)throw std::runtime_error("No planned motion to execute");
    if(plate_alignment_) {
      if(alignment_stops_.size()!=8)throw std::runtime_error("Plate alignment requires eight X/Y stationary stops");
      std::size_t previous=0;
      for(std::size_t index=0;index<alignment_stops_.size();++index) {
        const auto& stop=alignment_stops_[index];
        if(stop.after_segment<=previous||stop.after_segment>steps_.size()||
           stop.corner!=std::string(1,static_cast<char>('A'+index/2))||
           stop.axis!=(index%2?"Y":"X")||stop.yaw_deg!=(index%2?90.:0.)||
           steps_[stop.after_segment-1].event||steps_[stop.after_segment-1].hand)
          throw std::runtime_error("Plate alignment X/Y stop sequence is invalid");
        for(const auto& values:{steps_[stop.after_segment-1].trajectory.joint_trajectory.points.back().velocities,
                                steps_[stop.after_segment-1].trajectory.joint_trajectory.points.back().accelerations})
          if(values.size()!=arm_group_->getVariableCount()||
             std::any_of(values.begin(),values.end(),[](double value){return !std::isfinite(value)||std::abs(value)>1e-8;}))
            throw std::runtime_error("Plate alignment hold is not a stationary boundary");
        previous=stop.after_segment;
      }
      if(previous!=steps_.size())throw std::runtime_error("Plate alignment must end at the final Y hold");
    }
    auto alignment_label=[this](std::size_t index) {
      const auto& stop=alignment_stops_.at(index);
      std::string label="모서리 "+stop.corner+" · "+stop.axis+"방향 맞춤 위치까지 이동 · 도착 후 정지";
      if(index) {
        const auto& held=alignment_stops_[index-1];
        label="모서리 "+held.corner+" · "+held.axis+"방향 도착 · 판 조정 후 손을 빼고 OK · "+label;
      }
      return label;
    };
    std::string continuous_approval;
    std::size_t continuous_alignment_group=0;
    if(execution_mode_=="continuous") {
      continuous_approval=plate_alignment_?
        approvePreview(alignmentPreview(0,alignment_stops_.front().after_segment,*initial_state_),1,
          alignment_stops_.size(),alignment_label(0)):
        approvePreview(planned_display_,1,1,"Entire stacking path");
      checkMockHardware();
    }
    std::size_t motion_index=0;
    for(std::size_t step_index=0;step_index<steps_.size();++step_index) {
      const auto& step=steps_[step_index];
      std::size_t leaving_stop=alignment_stops_.size(),arrival_stop=alignment_stops_.size();
      if(plate_alignment_)for(std::size_t index=0;index<alignment_stops_.size();++index) {
        if(alignment_stops_[index].after_segment==step_index)leaving_stop=index;
        if(alignment_stops_[index].after_segment==step_index+1)arrival_stop=index;
      }
      checkStop();
      if(step.event){apply(step.diff);if(step.completed){completed=step.completed;status("PLACED",completed);}continue;}
      ++motion_index;
      std::string approval;
      if(execution_mode_=="step") {
        auto measured=measuredState(5.);
        moveit_msgs::msg::DisplayTrajectory preview;preview.model_id=model_->getName();
        moveit::core::robotStateToRobotStateMsg(*measured,preview.trajectory_start);
        preview.trajectory.push_back(step.trajectory);
        const auto label=leaving_stop<alignment_stops_.size()?
          "모서리 "+alignment_stops_[leaving_stop].corner+" · "+alignment_stops_[leaving_stop].axis+
            "방향 도착 · 판 조정 후 손을 빼고 OK로 계속 · "+step.label:step.label;
        approval=approvePreview(preview,motion_index,motion_count,label);
        checkMockHardware();
      }
      if(leaving_stop<alignment_stops_.size())alignmentEvent("STACK_ALIGNMENT_RESUMED",leaving_stop);
      auto measured=measuredState(5.);
      auto live=readScene();State full(live->getCurrentState());full.setVariablePositions(measured->getVariablePositions());full.update();
      robot_trajectory::RobotTrajectory path(model_,step.hand?hand_group_:group_);path.setRobotTrajectoryMsg(full,step.trajectory);
      if(full.distance(path.getFirstWayPoint(),step.hand?gripper_group_:arm_group_)>.02)throw std::runtime_error("Measured start changed; stacking stopped before "+step.label);
      if(!live->isPathValid(path,"",false))throw std::runtime_error("Live planning scene changed or path is blocked: "+step.label);
      MGI::Plan plan;moveit::core::robotStateToRobotStateMsg(full,plan.start_state_);plan.trajectory_=step.trajectory;
      RCLCPP_INFO(logger_,"STACK_EXECUTING %s",step.label.c_str());
      checkStop();
      if(execution_mode_=="step")RCLCPP_INFO(logger_,"STACK_STEP_EXECUTING %s",approval.c_str());
      else if(motion_index==1||leaving_stop<alignment_stops_.size())
        RCLCPP_INFO(logger_,"STACK_STEP_EXECUTING %s",continuous_approval.c_str());
      executing_=true;
      auto result=(step.hand?*hand_:*arm_).execute(plan);
      executing_=false;
      if(result!=moveit::core::MoveItErrorCode::SUCCESS)throw std::runtime_error("Execution failed: "+step.label+" code="+std::to_string(result.val));
      if(execution_mode_=="step")RCLCPP_INFO(logger_,"STACK_STEP_COMPLETED %s",approval.c_str());
      if(arrival_stop<alignment_stops_.size()) {
        // Every lowered X/Y hold retains its own approval boundary. Neither a
        // retract nor a rotation may run on the preceding arrival's approval.
        alignmentEvent("STACK_ALIGNMENT_PAUSED",arrival_stop);
        if(execution_mode_=="continuous") {
          RCLCPP_INFO(logger_,"STACK_STEP_COMPLETED %s",continuous_approval.c_str());
          continuous_alignment_group=arrival_stop+1;
          if(continuous_alignment_group<alignment_stops_.size()) {
            auto held=measuredState(5.);
            continuous_approval=approvePreview(alignmentPreview(alignment_stops_[arrival_stop].after_segment,
                alignment_stops_[continuous_alignment_group].after_segment,*held),
                continuous_alignment_group+1,alignment_stops_.size(),alignment_label(continuous_alignment_group));
            checkMockHardware();
          }
        }
      }
    }
    if(execution_mode_=="continuous"&&!plate_alignment_)RCLCPP_INFO(logger_,"STACK_STEP_COMPLETED %s",continuous_approval.c_str());
    status("DONE",completed);RCLCPP_INFO(logger_,"STACK_SUCCESS moved=%d layers=%d target=%d",completed,completed+foundationLayers(),target_);
  }
  void status(const std::string& phase,int moved)const {
    RCLCPP_INFO(logger_,"STACK_STATUS phase=%s moved=%d total=%zu target=%d layers=%d",phase.c_str(),moved,total_,target_,moved+foundationLayers());
  }
  void publishLabels() {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;clear.action=clear.DELETEALL;markers.markers.push_back(clear);
    if(mode_!="reset")for(int id=1;id<=columns_*rows_;++id) {
      visualization_msgs::msg::Marker marker;marker.header.frame_id=frame_;marker.header.stamp=node_->now();marker.ns="cell_ids";marker.id=id;marker.type=marker.TEXT_VIEW_FACING;marker.action=marker.ADD;
      marker.pose.orientation.w=1;marker.pose.position.x=cells_[id-1].x()-.023;marker.pose.position.y=cells_[id-1].y()+.023;marker.pose.position.z=table_z_+.012;
      marker.scale.z=.015;marker.color.r=marker.color.g=marker.color.b=marker.color.a=1;marker.text=std::to_string(id);markers.markers.push_back(marker);
    }
    markers_->publish(markers);
  }
  rclcpp::Node::SharedPtr node_;rclcpp::Logger logger_;
  std::string mode_,execution_mode_,plan_id_,display_file_,recipe_file_,placement_mode_,group_,hand_group_,hand_joint_,frame_,tip_,tcp_,approach_,table_;
  std::size_t approval_sequence_=0;
  std::vector<AlignmentStop> alignment_stops_;
  int target_,columns_,rows_,vertical_fallback_,vertical_placement_layers_;std::set<int> obstacles_,empty_;std::size_t total_=0;
  double width_,height_,table_z_,grip_height_,clearance_,side_clearance_,vel_,acc_,open_,close_,step_,travel_z_;
  bool split_speeds_=true,staged_release_=true;
  double approach_vel_=.05,transfer_vel_=.20,stack_acc_=.05,approach_acc_=.05,transfer_acc_=.20;
  double release_first_=.70,release_clearance_=.60;
  bool empty_hand_test_=false,empty_xy_scan_=false,scan_floor_check_=false;
  bool plate_enabled_=false,independent_target_=false,target_has_foundation_=false,plate_alignment_=false;
  double plate_bottom_z_=0.,source_surface_z_=0.,stack_surface_z_=0.,obstacle_surface_z_=0.,plate_yaw_=0.,alignment_clearance_=.05;
  double alignment_tip_clearance_=.010,alignment_hand_position_=.65;
  Eigen::Vector2d plate_center_=Eigen::Vector2d::Zero(),target_xy_=Eigen::Vector2d::Zero();
  std::string plate_uri_;
  double frozen_hand_position_=0.,scan_clearance_=.05,scan_floor_=0.,scan_minimum_z_=std::numeric_limits<double>::infinity();
  std::vector<GripperBounds> gripper_bounds_;
  Eigen::Vector3d block_,obstacle_;Eigen::Matrix3d rotation_;T world_frame_,center_,tip_tcp_,tcp_approach_,carry_fixed_,placed_cube_;
  std::vector<Eigen::Vector2d> cells_;std::vector<std::string> touch_;std::vector<Step> steps_;
  std::atomic<bool> cancelled_{false},executing_{false};std::unique_ptr<MGI> arm_,hand_;moveit::core::RobotModelConstPtr model_;
  std::atomic<MGI*> arm_stop_{nullptr},hand_stop_{nullptr};
  std::atomic<planning_pipeline::PlanningPipeline*> pipeline_stop_{nullptr};
  const moveit::core::JointModelGroup* arm_group_=nullptr;const moveit::core::JointModelGroup* gripper_group_=nullptr;
  planning_scene::PlanningScenePtr scene_;std::shared_ptr<State> initial_state_;std::unique_ptr<planning_pipeline::PlanningPipeline> pipeline_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_;rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_;rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  struct JointSample {double position;std::chrono::steady_clock::time_point received;};
  std::mutex feedback_mutex_;std::map<std::string,JointSample> feedback_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_subscription_;
  moveit_msgs::msg::DisplayTrajectory planned_display_;
  moveit_msgs::msg::PlanningScene recipe_initial_scene_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;
};
} // namespace
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);auto node=rclcpp::Node::make_shared("ur5e_a3_stack",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);std::thread spinner([&]{executor.spin();});
  int result=1;std::unique_ptr<StackTask> task;
  try{Lock lock;task=std::make_unique<StackTask>(node);result=task->run()?0:1;}
  catch(const std::exception& error){RCLCPP_ERROR(node->get_logger(),"STACK_ERROR %s",error.what());if(task)task->stop();}
  executor.cancel();spinner.join();task.reset();rclcpp::shutdown();return result;
}
