#include "rx150_hac_controller/gravity_comp.hpp"

#include <iostream>

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

GravityCompensation::GravityCompensation(
    const std::string & urdf_xml, const std::vector<std::string> & joint_names) {
  try {
    pinocchio::urdf::buildModelFromXML(urdf_xml, model_);
    data_ = pinocchio::Data(model_);

    joint_id_map_.resize(joint_names.size(), -1);
    valid_ = true;

    for (size_t i = 0; i < joint_names.size(); ++i) {
      if (model_.existJointName(joint_names[i])) {
        const pinocchio::JointIndex joint_id = model_.getJointId(joint_names[i]);
        joint_id_map_[i] = model_.joints[joint_id].idx_v();
      } else {
        std::cerr << "[GravityCompensation] Joint not found in URDF: "
                  << joint_names[i] << std::endl;
        valid_ = false;
      }
    }
  } catch (const std::exception & e) {
    std::cerr << "[GravityCompensation] Failed to parse URDF: " << e.what()
              << std::endl;
    valid_ = false;
  }
}

std::vector<double> GravityCompensation::compute(const std::vector<double> & q) const {
  if (!valid_) {
    return std::vector<double>(joint_id_map_.size(), 0.0);
  }

  Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model_.nq);
  for (size_t i = 0; i < joint_id_map_.size(); ++i) {
    if (joint_id_map_[i] >= 0 && joint_id_map_[i] < model_.nq) {
      q_full[joint_id_map_[i]] = q[i];
    }
  }

  const Eigen::VectorXd & gravity =
      pinocchio::computeGeneralizedGravity(model_, data_, q_full);

  std::vector<double> torques(joint_id_map_.size(), 0.0);
  for (size_t i = 0; i < joint_id_map_.size(); ++i) {
    if (joint_id_map_[i] >= 0 && joint_id_map_[i] < model_.nv) {
      torques[i] = gravity[joint_id_map_[i]];
    }
  }

  return torques;
}
