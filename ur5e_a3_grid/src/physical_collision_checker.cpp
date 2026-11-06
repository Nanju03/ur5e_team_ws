// Offline, persistent checker for the exact samples prepared by physical_stack.
// There are no action clients, driver connections or planning-scene writes.
// stdin: little-endian u32 scene length, u32 state count, CDR, count*7 doubles.
// stdout: one bounded JSON reply per request. Diagnostics go to stderr.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "bounded_surface_contacts.hpp"

namespace {
constexpr std::uint32_t MAX_SCENE=100*1024*1024,MAX_STATES=8192;
const std::vector<std::string> JOINTS={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
  "wrist_1_joint","wrist_2_joint","wrist_3_joint","left_outer_knuckle_joint"};
void read(char* data,std::size_t size) {
  if(!std::cin.read(data,size))throw std::runtime_error("Truncated checker request");
}
std::uint32_t u32(const unsigned char* p) {
  return std::uint32_t(p[0])|(std::uint32_t(p[1])<<8)|(std::uint32_t(p[2])<<16)|(std::uint32_t(p[3])<<24);
}
std::string quoted(const std::string& value) {
  std::string out="\"";
  for(unsigned char c:value) {
    if(c=='"'||c=='\\')out+='\\';
    if(c<32)out+=' ';else out+=c;
  }
  return out+'"';
}
}
int main(int argc,char** argv) {
  try {
    const std::uint16_t endian=1;
    if(*reinterpret_cast<const unsigned char*>(&endian)!=1||sizeof(double)!=8)
      throw std::runtime_error("Checker needs little-endian IEEE754 doubles");
    rclcpp::init(argc,argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.start_parameter_services(false).start_parameter_event_publisher(false);
    auto node=std::make_shared<rclcpp::Node>("physical_collision_checker",options);
    robot_model_loader::RobotModelLoader loader(node,"robot_description",false);
    const auto model=loader.getModel();
    const bool bounded_contacts=node->has_parameter("bounded_surface_contacts")&&
      node->get_parameter("bounded_surface_contacts").as_bool();
    if(!model)throw std::runtime_error("Checker model unavailable");
    for(const auto& name:JOINTS)model->getVariableIndex(name);  // throws on a model mismatch
    std::cout<<"{\"ready\":true,\"protocol\":1}"<<std::endl;
    std::uint32_t cached_size=0;
    std::vector<std::uint8_t> cached;
    planning_scene::PlanningScenePtr scene;
    while(std::cin.peek()!=std::char_traits<char>::eof()) {
      std::array<unsigned char,8> header{};read(reinterpret_cast<char*>(header.data()),header.size());
      const auto size=u32(header.data()),count=u32(header.data()+4);
      if(!size||size>MAX_SCENE||!count||count>MAX_STATES)throw std::runtime_error("Checker request limits exceeded");
      std::vector<std::uint8_t> data(size);read(reinterpret_cast<char*>(data.data()),data.size());
      std::vector<double> values(std::size_t(count)*JOINTS.size());
      read(reinterpret_cast<char*>(values.data()),values.size()*sizeof(double));
      if(!scene||cached_size!=size||cached!=data) {
        rclcpp::SerializedMessage serialized(size);
        auto& raw=serialized.get_rcl_serialized_message();
        std::memcpy(raw.buffer,data.data(),size);raw.buffer_length=size;
        moveit_msgs::msg::PlanningScene message;
        rclcpp::Serialization<moveit_msgs::msg::PlanningScene>().deserialize_message(&serialized,&message);
        if(message.is_diff||message.robot_state.is_diff)throw std::runtime_error("Checker requires a full scene snapshot");
        if(!message.robot_model_name.empty()&&message.robot_model_name!=model->getName())
          throw std::runtime_error("Checker scene/model mismatch");
        scene=std::make_shared<planning_scene::PlanningScene>(model);
        if(!scene->setPlanningSceneMsg(message))throw std::runtime_error("Checker scene rejected");
        if(bounded_contacts)incremental_motion::boundSurfaceContacts(*scene,scene->getAllowedCollisionMatrixNonConst());
        cached=std::move(data);cached_size=size;
      }
      moveit::core::RobotState state(scene->getCurrentState());
      bool valid=true;std::uint32_t failed=0;
      for(std::uint32_t index=0;index<count;++index) {
        for(std::size_t joint=0;joint<JOINTS.size();++joint) {
          const double q=values[std::size_t(index)*JOINTS.size()+joint];
          if(!std::isfinite(q))throw std::runtime_error("Nonfinite checker state");
          state.setVariablePosition(JOINTS[joint],q);
        }
        state.update();  // setVariablePosition also updates the driven mimic joints
        // Full robot, world, attached bodies, ACM, padding and link scales.
        // Bounds are checked as well as the service's collision/feasibility rule.
        if(!state.satisfiesBounds()||!scene->isStateValid(state,"",false)) {
          valid=false;failed=index;break;
        }
      }
      if(valid)std::cout<<"{\"ok\":true,\"checked\":"<<count<<"}"<<std::endl;
      else {
        // Collect contacts only on failure. Successful batches retain the
        // exact same one-check-per-state fast path.
        collision_detection::CollisionRequest request;collision_detection::CollisionResult result;
        request.contacts=true;request.max_contacts=8;request.max_contacts_per_pair=1;
        scene->checkCollision(request,result,state);
        std::cout<<"{\"ok\":false,\"checked\":"<<failed<<",\"invalid_index\":"<<failed
          <<",\"bounds\":"<<(state.satisfiesBounds()?"true":"false")<<",\"contacts\":[";
        bool comma=false;for(const auto& entry:result.contacts) {
          if(comma)std::cout<<',';comma=true;
          std::cout<<'['<<quoted(entry.first.first)<<','<<quoted(entry.first.second)<<']';
        }
        std::cout<<"]}"<<std::endl;
      }
    }
    rclcpp::shutdown();return 0;
  }catch(const std::exception& error) {
    std::cout<<"{\"ok\":false,\"error\":"<<quoted(error.what())<<"}"<<std::endl;
    std::cerr<<"Physical collision checker: "<<error.what()<<std::endl;return 1;
  }
}
