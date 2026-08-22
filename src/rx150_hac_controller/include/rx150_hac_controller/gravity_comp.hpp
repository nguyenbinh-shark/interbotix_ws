#pragma once

#include <string>
#include <vector>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

class GravityCompensation {
 public:
  GravityCompensation(
      const std::string & urdf_xml, const std::vector<std::string> & joint_names);

  std::vector<double> compute(const std::vector<double> & q) const;

  bool isValid() const { return valid_; }

 private:
  pinocchio::Model model_;
  mutable pinocchio::Data data_;
  std::vector<int> joint_id_map_;
  bool valid_ = false;
};
