#include <joint_limit_table/JointLimitTable.h>

namespace joint_limit_table{
  double JointLimitTable::getInterpolatedLimitAngle (const double target_joint_angle, const bool is_llimit_angle) const {
    double target_angle = target_joint_angle * 180.0 / M_PI; // [rad]=>[deg]
    int int_target_angle = static_cast<int>(std::floor(target_angle));
    int target_range[2] = {int_target_angle, 1+int_target_angle};
    double self_joint_range[2];
    for (size_t i = 0; i < 2; i++) {
      size_t idx = std::min(std::max(target_llimit_angle_, target_range[i]), target_ulimit_angle_) - target_llimit_angle_;
      self_joint_range[i] = (is_llimit_angle ? llimit_table_[idx] : ulimit_table_[idx]);
    }
    double tmp_ratio = target_angle - int_target_angle;
    return (self_joint_range[0] * (1-tmp_ratio) + self_joint_range[1] * tmp_ratio) * M_PI / 180.0; // [deg]=>[rad]
  };

  std::vector<std::shared_ptr<JointLimitTable> > readJointLimitTablesFromProperty (const cnoid::BodyPtr robot,
                                                                                   const std::string& prop_string){
    std::vector<std::shared_ptr<JointLimitTable> > joint_limit_tables;

    size_t limit_table_size = 6; // self_joint_name:target_joint_name:target_min_angle:target_max_angle:min:max

    std::stringstream ss(prop_string);
    std::vector<std::string> item(limit_table_size);

    while (std::getline(ss, item[0], ':')) {
      for(size_t i=1;i<limit_table_size;i++){
        std::getline(ss, item[i], ':');
      }

      const cnoid::LinkPtr self_joint = robot->link(item[0]);
      const cnoid::LinkPtr target_joint = robot->link(item[1]);

      int target_llimit_angle = std::stoi(item[2]);
      int target_ulimit_angle = std::stoi(item[3]);

      std::vector<double> llimit_table;
      std::vector<double> ulimit_table;
      std::stringstream ssl(item[4]);
      std::stringstream ssu(item[5]);
      std::string sl, su;
      while (std::getline(ssl, sl, ',') && std::getline(ssu, su, ',')) {
        llimit_table.push_back(std::stod(sl));
        ulimit_table.push_back(std::stod(su));
      }

      if ( llimit_table.size() != ulimit_table.size() ||
           llimit_table.size() != target_ulimit_angle - target_llimit_angle + 1  ||
           !target_joint ||
           ! self_joint) {
        std::cerr << "[readJointLimitTablesFromProperty] " << item[0] << ":" << item[1] << " failed" << std::endl;
        continue;
      }

      joint_limit_tables.push_back(std::make_shared<JointLimitTable>(self_joint,
                                                                     target_joint,
                                                                     target_llimit_angle,
                                                                     target_ulimit_angle,
                                                                     llimit_table,
                                                                     ulimit_table));
    }
    return joint_limit_tables;
  }
};
