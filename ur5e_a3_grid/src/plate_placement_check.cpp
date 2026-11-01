// SPDX-License-Identifier: BSD-3-Clause
// Offline pose/clearance search. This program creates no motion action clients,
// hardware drivers, planning-scene services or execution publishers.
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <Eigen/SVD>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace {
using T=Eigen::Isometry3d;
using State=moveit::core::RobotState;
constexpr double PI=3.14159265358979323846;
template<class V> V param(const rclcpp::Node::SharedPtr& node,const std::string& name,V fallback) {
  if(!node->has_parameter(name))node->declare_parameter<V>(name,fallback);
  return node->get_parameter(name).get_value<V>();
}
geometry_msgs::msg::Pose pose(const T& t) {
  geometry_msgs::msg::Pose p;p.position.x=t.translation().x();p.position.y=t.translation().y();p.position.z=t.translation().z();
  Eigen::Quaterniond q(t.linear());p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w();return p;
}
moveit_msgs::msg::CollisionObject box(const std::string& id,const Eigen::Vector3d& size,const T& t) {
  moveit_msgs::msg::CollisionObject o;o.id=id;o.header.frame_id="world";o.operation=o.ADD;o.pose.orientation.w=1.;
  shape_msgs::msg::SolidPrimitive shape;shape.type=shape.BOX;shape.dimensions={size.x(),size.y(),size.z()};
  o.primitives.push_back(shape);o.primitive_poses.push_back(pose(t));return o;
}
struct Result {
  double distance=0.,yaw=0.,score=-1e9,sigma=std::numeric_limits<double>::infinity();
  double margin=std::numeric_limits<double>::infinity(),clearance=std::numeric_limits<double>::infinity(),travel=0.;
  int pockets=0,corners=0,layers=0,poses=0;
  bool valid()const{return pockets==24&&corners==4&&layers==24;}
};
class Search {
public:
  explicit Search(rclcpp::Node::SharedPtr node):node_(std::move(node)),loader_(node_,"robot_description",true) {
    model_=loader_.getModel();if(!model_)throw std::runtime_error("Calibrated robot model is required");
    arm_=model_->getJointModelGroup("ur5e_arm");hand_=model_->getJointModelGroup("dh_ag95_gripper");
    if(!arm_||!hand_)throw std::runtime_error("Arm/gripper groups are required");
    home_=std::make_unique<State>(model_);home_->setToDefaultValues();
    if(!home_->setToDefaultValues(arm_,"home"))throw std::runtime_error("home state missing");
    table_z_=param<double>(node_,"table_surface_z",.8);
    open_q_=param<double>(node_,"placement_check_open_q",.5);close_q_=param<double>(node_,"placement_check_grasp_q",.72);
    grasp_height_=param<double>(node_,"grasp_height_above_table",.018);
    if(!std::isfinite(grasp_height_)||grasp_height_<=0.||grasp_height_>=.025)throw std::runtime_error("Invalid grasp height");
    setHand(*home_,open_q_);home_->update();
    tip_tcp_=home_->getGlobalLinkTransform("tool0").inverse()*home_->getGlobalLinkTransform("grasp_link");
    tcp_approach_=home_->getGlobalLinkTransform("grasp_link").inverse()*home_->getGlobalLinkTransform("ag95_base_link");
    const T initial_tcp=home_->getGlobalLinkTransform("grasp_link");
    const T approach=home_->getGlobalLinkTransform("ag95_base_link");
    const Eigen::Vector3d down(0,0,-1);
    Eigen::Vector3d x=approach.linear().col(0);x-=down*down.dot(x);x.normalize();
    Eigen::Matrix3d rotation;rotation.col(0)=x;rotation.col(1)=down.cross(x);rotation.col(2)=down;
    down_=rotation*approach.linear().transpose()*initial_tcp.linear();
    uri_=param<std::string>(node_,"plate_mesh_resource","package://ur5e_block_description/meshes/cube_plate.STL");
    plate_mesh_.reset(shapes::createMeshFromResource(uri_,Eigen::Vector3d::Constant(.001)));
    if(!plate_mesh_||!plate_mesh_->vertex_count)throw std::runtime_error("Plate mesh is unavailable");
    shapes::constructMsgFromShape(plate_mesh_.get(),plate_shape_);
    touch_={"left_finger_pad","right_finger_pad","left_finger","right_finger"};
  }
  std::vector<Result> run() {
    std::vector<Result> results;
    auto distances=param<std::vector<double>>(node_,"placement_check_distances",{.325,.350,.375,.400,.425,.450,.475,.500,.525,.550});
    auto yaws=param<std::vector<double>>(node_,"placement_check_yaws",{-90.,0.});
    for(double yaw:yaws)for(double distance:distances) {
      if(!std::isfinite(yaw)||!std::isfinite(distance)||distance<.25||distance>.65)throw std::runtime_error("Invalid placement candidates");
      auto result=check(distance,yaw);results.push_back(result);
      std::cout<<"PLATE_CANDIDATE "<<json(result)<<std::endl;
    }
    return results;
  }
  std::string json(const Result& r)const {
    std::ostringstream o;o<<std::setprecision(10)<<"{\"distance_m\":"<<r.distance<<",\"yaw_deg\":"<<r.yaw
      <<",\"pocket_pairs\":"<<r.pockets<<",\"corner_poses\":"<<r.corners<<",\"stack_layers\":"<<r.layers
      <<",\"checked_poses\":"<<r.poses<<",\"all_required_poses_valid\":"<<(r.valid()?"true":"false")
      <<",\"score\":"<<r.score<<",\"minimum_scaled_jacobian_singular_value\":"<<finite(r.sigma)
      <<",\"minimum_joint_limit_margin_rad\":"<<finite(r.margin)<<",\"minimum_collision_distance_m\":"<<finite(r.clearance)
      <<",\"mean_joint_distance_to_home_rad\":"<<(r.poses?r.travel/r.poses:0.)<<"}";return o.str();
  }
private:
  static double finite(double value){return std::isfinite(value)?value:0.;}
  void setHand(State& state,double q)const{state.setJointGroupActivePositions(hand_,std::vector<double>{q});state.update();}
  bool valid(State& state)const {
    state.update();return state.satisfiesBounds(arm_)&&state.satisfiesBounds(hand_)&&!scene_->isStateColliding(state,"",false);
  }
  bool solve(const T& tcp,State& state)const {
    const T goal=tcp*tip_tcp_.inverse();
    if(!state.setFromIK(arm_,goal,"tool0",.035,[this](State* s,const auto* group,const double* q){
      s->setJointGroupPositions(group,q);return valid(*s);
    }))return false;
    // Equivalent revolute representations near +/-2pi must not make a
    // candidate's margin/home-distance score depend on KDL's wrap choice.
    // This normalizes offline pose scoring only, never an execution path.
    std::vector<double> normalized;
    for(const auto& name:arm_->getVariableNames()) {
      const auto& bounds=model_->getVariableBounds(name);
      const double original=state.getVariablePosition(name),reference=home_->getVariablePosition(name);
      double best=original;
      for(int turn=-3;turn<=3;++turn) {
        const double equivalent=original+turn*2.*PI;
        if(bounds.position_bounded_&&(equivalent<bounds.min_position_||equivalent>bounds.max_position_))continue;
        if(std::abs(equivalent-reference)<std::abs(best-reference))best=equivalent;
      }
      normalized.push_back(best);
    }
    state.setJointGroupActivePositions(arm_,normalized);state.update();
    return valid(state);
  }
  T at(const Eigen::Vector2d& xy,double z)const{T t=T::Identity();t.linear()=down_;t.translation()=Eigen::Vector3d(xy.x(),xy.y(),z);return t;}
  void observe(const State& state,Result& result)const {
    Eigen::MatrixXd jacobian;state.getJacobian(arm_,model_->getLinkModel("tool0"),tip_tcp_.translation(),jacobian);
    jacobian.topRows(3)/=.5;Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
    result.sigma=std::min(result.sigma,svd.singularValues().minCoeff());
    for(const auto& name:arm_->getVariableNames()) {
      const auto& bounds=model_->getVariableBounds(name);const double q=state.getVariablePosition(name);
      if(bounds.position_bounded_)result.margin=std::min({result.margin,q-bounds.min_position_,bounds.max_position_-q});
    }
    result.clearance=std::min(result.clearance,scene_->distanceToCollision(state));
    result.travel+=state.distance(*home_,arm_);++result.poses;
  }
  std::string sourceId(int i)const{return "placement_search/source_"+std::to_string(i);}
  void remove(const std::string& id) {
    moveit_msgs::msg::CollisionObject msg;msg.id=id;msg.operation=msg.REMOVE;scene_->processCollisionObjectMsg(msg);
  }
  Result check(double distance,double yaw_deg) {
    Result result;result.distance=distance;result.yaw=yaw_deg;
    scene_=std::make_shared<planning_scene::PlanningScene>(model_);scene_->setCurrentState(*home_);
    const Eigen::Vector2d centre(distance,0.);
    const Eigen::Matrix3d rotation=Eigen::AngleAxisd(yaw_deg*PI/180.,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T origin=T::Identity();origin.linear()=rotation;
    origin.translation()=Eigen::Vector3d(distance,0.,table_z_)-rotation*Eigen::Vector3d(.210,.1485,0.);
    moveit_msgs::msg::CollisionObject plate;plate.id="placement_search/plate";plate.header.frame_id="world";
    plate.operation=plate.ADD;plate.pose.orientation.w=1.;plate.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(plate_shape_));
    plate.mesh_poses.push_back(pose(origin));scene_->processCollisionObjectMsg(plate);
    auto& acm=scene_->getAllowedCollisionMatrixNonConst();acm.setEntry("placement_search/plate","workbench",true);
    std::vector<Eigen::Vector2d> cells;
    for(double y:{.0594,.1188,.1782,.2376})for(double x:{.060,.120,.180,.240,.300,.360}) {
      const Eigen::Vector3d world=origin*Eigen::Vector3d(x,y,0.);
      cells.push_back(world.head<2>());
      T cube=T::Identity();cube.linear()=rotation;cube.translation()=Eigen::Vector3d(world.x(),world.y(),table_z_+.005+.0125);
      scene_->processCollisionObjectMsg(box(sourceId(cells.size()),Eigen::Vector3d::Constant(.025),cube));
    }
    // Every pocket is assessed with all the neighbouring cubes still present.
    for(std::size_t i=0;i<cells.size();++i) {
      acm.setEntry(sourceId(i+1),touch_,true);
      bool accepted=false;double best=-1.;State best_pre(*home_),best_pick(*home_);
      for(int turn=0;turn<4;++turn) {
        T low=at(cells[i],table_z_+.005+grasp_height_);
        low.linear()=Eigen::AngleAxisd(turn*PI/2.,Eigen::Vector3d::UnitZ()).toRotationMatrix()*low.linear();
        T high=low;high.translation().z()+=.06;State probe(*home_);setHand(probe,open_q_);
        if(!solve(high,probe))continue;State pre(probe);if(!solve(low,probe))continue;
        Eigen::MatrixXd j;probe.getJacobian(arm_,model_->getLinkModel("tool0"),tip_tcp_.translation(),j);
        j.topRows(3)/=.5;const double quality=Eigen::JacobiSVD<Eigen::MatrixXd>(j).singularValues().minCoeff();
        if(quality>best){best=quality;best_pre=pre;best_pick=probe;accepted=true;}
      }
      if(accepted){++result.pockets;observe(best_pre,result);observe(best_pick,result);}
      acm.setEntry(sourceId(i+1),touch_,false);
    }
    // Corner alignment also checks the widest hand shape, after removing the
    // supply cubes; the user aligns an empty plate, not a populated fixture.
    for(std::size_t i=1;i<=cells.size();++i)remove(sourceId(i));
    const double half_x=(std::abs(rotation(0,0))*.420+std::abs(rotation(0,1))*.297)/2.;
    const double half_y=(std::abs(rotation(1,0))*.420+std::abs(rotation(1,1))*.297)/2.;
    for(const auto& offset:std::vector<Eigen::Vector2d>{{-half_x,half_y},{half_x,half_y},{half_x,-half_y},{-half_x,-half_y}}) {
      State probe(*home_);setHand(probe,0.);T corner=at(centre+offset,table_z_+.010+.05+.020+.0165);
      if(solve(corner,probe)){++result.corners;observe(probe,result);}
    }
    // Attach one carried cube with the actual pickup offset. Endpoint checks
    // include the complete lower tower and the carried cube, but are not a
    // claim that a collision-free transfer path has been planned.
    T nominal=at(centre,table_z_+.005+grasp_height_);
    T cube=T::Identity();cube.linear()=rotation;cube.translation()=Eigen::Vector3d(distance,0.,table_z_+.005+.0125);
    const T carry=nominal.inverse()*cube;
    State carrying(*home_);setHand(carrying,close_q_);scene_->setCurrentState(carrying);
    moveit_msgs::msg::AttachedCollisionObject carried;carried.link_name="grasp_link";carried.touch_links=touch_;
    carried.object.id="placement_search/carried";carried.object.header.frame_id="grasp_link";carried.object.operation=carried.object.ADD;
    carried.object.pose.orientation.w=1.;shape_msgs::msg::SolidPrimitive solid;solid.type=solid.BOX;solid.dimensions={.025,.025,.025};
    carried.object.primitives.push_back(solid);carried.object.primitive_poses.push_back(pose(carry));
    scene_->processAttachedCollisionObjectMsg(carried);
    for(int layer=1;layer<=24;++layer) {
      std::vector<std::pair<T,T>> candidates;
      if(layer<=10)for(int turn=0;turn<4;++turn) {
        T low=at(centre,table_z_+.010+(layer-1)*.025+grasp_height_);
        low.linear()=Eigen::AngleAxisd(turn*PI/2.,Eigen::Vector3d::UnitZ()).toRotationMatrix()*low.linear();
        T high=low;high.translation().z()+=.04;candidates.emplace_back(low,high);
      } else {
        std::array<int,3> axes{0,1,2};
        do{for(int sx:{-1,1})for(int sy:{-1,1})for(int sz:{-1,1}) {
          Eigen::Matrix3d symmetry=Eigen::Matrix3d::Zero();symmetry(axes[0],0)=sx;symmetry(axes[1],1)=sy;symmetry(axes[2],2)=sz;
          if(symmetry.determinant()<.5)continue;
          T desired=T::Identity();desired.linear()=rotation*symmetry;desired.translation()=Eigen::Vector3d(distance,0.,table_z_+.010+(layer-.5)*.025);
          T tcp=desired*carry.inverse();const Eigen::Matrix3d approach=tcp.linear()*tcp_approach_.linear();
          if(std::abs(approach(2,2))>.01||std::abs(approach(2,0))>.01)continue;
          T pre=tcp;pre.translation()-=.060*approach.col(2);candidates.emplace_back(tcp,pre);
        }}while(std::next_permutation(axes.begin(),axes.end()));
      }
      const std::string support=layer==1?"placement_search/plate":"placement_search/tower_"+std::to_string(layer-1);
      acm.setEntry("placement_search/carried",support,true);
      bool accepted=false;double best=-1.;State best_pre(scene_->getCurrentState()),best_low(best_pre);
      for(const auto& candidate:candidates) {
        State probe(scene_->getCurrentState());if(!solve(candidate.second,probe))continue;State pre(probe);
        if(!solve(candidate.first,probe))continue;
        Eigen::MatrixXd j;probe.getJacobian(arm_,model_->getLinkModel("tool0"),tip_tcp_.translation(),j);j.topRows(3)/=.5;
        const double quality=Eigen::JacobiSVD<Eigen::MatrixXd>(j).singularValues().minCoeff();
        if(quality>best){best=quality;best_pre=pre;best_low=probe;accepted=true;}
      }
      if(!accepted){acm.setEntry("placement_search/carried",support,false);break;}
      result.layers=layer;observe(best_pre,result);observe(best_low,result);
      acm.setEntry("placement_search/carried",support,false);
      T placed=T::Identity();placed.linear()=rotation;placed.translation()=Eigen::Vector3d(distance,0.,table_z_+.010+(layer-.5)*.025);
      scene_->processCollisionObjectMsg(box("placement_search/tower_"+std::to_string(layer),Eigen::Vector3d::Constant(.025),placed));
    }
    result.score=2.*finite(result.sigma)+.05*finite(result.margin)+.25*finite(result.clearance)-.005*(result.poses?result.travel/result.poses:0.);
    return result;
  }
  rclcpp::Node::SharedPtr node_;robot_model_loader::RobotModelLoader loader_;
  moveit::core::RobotModelPtr model_;const moveit::core::JointModelGroup *arm_=nullptr,*hand_=nullptr;
  std::unique_ptr<State> home_;planning_scene::PlanningScenePtr scene_;
  double table_z_=0.,open_q_=.5,close_q_=.72,grasp_height_=.018;
  Eigen::Matrix3d down_;T tip_tcp_,tcp_approach_;
  std::string uri_;std::vector<std::string> touch_;std::unique_ptr<shapes::Mesh> plate_mesh_;shapes::ShapeMsg plate_shape_;
};
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  int code=1;
  try {
    auto node=rclcpp::Node::make_shared("plate_placement_check",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    Search search(node);const auto results=search.run();
    const auto better=[](const Result& a,const Result& b){
      if(a.valid()!=b.valid())return !a.valid();
      if(a.layers!=b.layers)return a.layers<b.layers;
      if(a.pockets!=b.pockets)return a.pockets<b.pockets;
      if(a.corners!=b.corners)return a.corners<b.corners;return a.score<b.score;
    };
    const auto best=std::max_element(results.begin(),results.end(),better);
    if(best==results.end())throw std::runtime_error("No placement candidates");
    std::ostringstream out;out<<"{\"search_kind\":\"finite calibrated IK and collision endpoint candidate search\","
      <<"\"physical_motion_sent\":false,\"full_transfer_paths_planned\":false,"
      <<"\"joint_angle_normalization\":\"accepted IK poses use the equivalent revolute angles closest to named home within model bounds; actual planner paths unchanged\",\"open_q\":"<<param<double>(node,"placement_check_open_q",.5)
      <<",\"grasp_q\":"<<param<double>(node,"placement_check_grasp_q",.72)
      <<",\"score_definition\":\"2*min_scaled_sigma + .05*min_joint_margin + .25*min_collision_distance - .005*mean_joint_distance_to_home\",\"best\":"<<search.json(*best)<<",\"candidates\":[";
    for(std::size_t i=0;i<results.size();++i){if(i)out<<',';out<<search.json(results[i]);}out<<"]}";
    const auto path=param<std::string>(node,"placement_check_output","");if(!path.empty()){std::ofstream file(path);file<<out.str()<<'\n';if(!file)throw std::runtime_error("Cannot save placement check results");}
    std::cout<<"PLATE_PLACEMENT_RESULT "<<out.str()<<std::endl;code=best->valid()?0:2;
  } catch(const std::exception& error){std::cerr<<"PLATE_PLACEMENT_ERROR "<<error.what()<<std::endl;}
  rclcpp::shutdown();return code;
}
