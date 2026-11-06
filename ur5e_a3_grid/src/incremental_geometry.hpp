#pragma once
#include <Eigen/Core>

namespace incremental_geometry {
inline bool levelBox(const Eigen::Matrix3d& rotation,const Eigen::Vector3d& size) {
  if(!rotation.allFinite()||!size.allFinite()||size.minCoeff()<=0.)return false;
  // A side placement permutes a cube's local axes. Its occupied geometry is
  // still level when ANY face normal is vertical, including a negative axis.
  const bool cube=(size.array()-size.x()).abs().maxCoeff()<1e-7;
  return cube?rotation.row(2).cwiseAbs().maxCoeff()>=.999:rotation(2,2)>=.999;
}
}
