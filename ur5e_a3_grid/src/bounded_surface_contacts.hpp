#pragma once

#include <cmath>
#include <moveit/planning_scene/planning_scene.h>

namespace incremental_motion {
// Tighten existing permissions for the carried cube. Exact surface contact is
// useful at pickup/setdown; penetration must never be hidden by a blanket ACM.
// Touch links retain their intentional gripper contact permissions.
constexpr double SURFACE_DEPTH = 1e-5;
inline void boundSurfaceContacts(const planning_scene::PlanningScene& scene,
                                collision_detection::AllowedCollisionMatrix& acm) {
  std::vector<const moveit::core::AttachedBody*> bodies;
  scene.getCurrentState().getAttachedBodies(bodies);
  std::vector<std::string> names;
  acm.getAllEntryNames(names);
  for(const auto* body:bodies) {
    if(body->getName().rfind("ur5e_stack/block_",0)!=0)continue;
    for(const auto& name:names) {
      if(name==body->getName()||body->getTouchLinks().count(name))continue;
      collision_detection::AllowedCollision::Type type;
      if(acm.getAllowedCollision(body->getName(),name,type)&&
         type==collision_detection::AllowedCollision::ALWAYS) {
        collision_detection::DecideContactFn contact=[](collision_detection::Contact& c) {
          return std::isfinite(c.depth)&&c.depth<=SURFACE_DEPTH;
        };
        acm.setEntry(body->getName(),name,contact);
      }
    }
  }
}
}
