#ifndef JOINT_LIMIT_TABLE_JOINTLIMITTABLE_H
#define JOINT_LIMIT_TABLE_JOINTLIMITTABLE_H

#include <cnoid/Body>
#include <memory>
#include <vector>
#include <iostream>

namespace joint_limit_table{
  // JointLimitTable for one joint
  //   self_joint   : a joint to obtain llimit and ulimit from this class.
  //   target_joint : self_joint's limit is difference for target_joint's joint angle.
  class JointLimitTable {
  public:
    JointLimitTable (const cnoid::LinkPtr& self_joint,
                     const cnoid::LinkPtr& target_joint,
                     const int target_llimit_angle,//deg
                     const int target_ulimit_angle,//deg
                     const std::vector<double>& llimit_table,//deg
                     const std::vector<double>& ulimit_table//deg
                     )
      : self_joint_(self_joint),
        target_joint_(target_joint),
        target_llimit_angle_(target_llimit_angle),
        target_ulimit_angle_(target_ulimit_angle),
        llimit_table_(llimit_table),
        ulimit_table_(ulimit_table)
    {
      if(llimit_table_.size() != target_ulimit_angle_ - target_llimit_angle_ + 1 ||
         ulimit_table_.size() != target_ulimit_angle_ - target_llimit_angle_ + 1){
        std::cerr << "\e[0;31m" << "[JointLimitTable] dimension mismatch for joint [" << self_joint_ << "]"  << "\e[0m" << std::endl;
      }
    };
    const cnoid::LinkPtr getSelfJoint () const { return self_joint_; };
    const cnoid::LinkPtr getTargetJoint () const { return target_joint_; };
    const std::vector<double>& lLimitTable() const { return llimit_table_;}
    std::vector<double>& lLimitTable() { return llimit_table_;}
    const std::vector<double>& uLimitTable() const { return ulimit_table_;}
    std::vector<double>& uLimitTable() { return ulimit_table_;}
    double getLlimit (double target_joint_angle) const // [rad]
    {
      return getInterpolatedLimitAngle(target_joint_angle, true); // [rad]
    };
    double getLlimit () const
    {
      return getLlimit(target_joint_->q());
    }
    double getUlimit (double target_joint_angle) const // [rad]
    {
      return getInterpolatedLimitAngle(target_joint_angle, false); // [rad]
    };
    double getUlimit () const
    {
      return getUlimit(target_joint_->q());
    }
  private:
    const cnoid::LinkPtr self_joint_;
    const cnoid::LinkPtr target_joint_;
    int target_llimit_angle_, target_ulimit_angle_; // llimit and ulimit angle [deg] for target_joint
    std::vector<double> llimit_table_, ulimit_table_; // Tables for self_joint's llimit and ulimit
    double getInterpolatedLimitAngle (const double target_joint_angle, const bool is_llimit_angle) const;
  };

  std::vector<std::shared_ptr<JointLimitTable> > readJointLimitTablesFromProperty (const cnoid::BodyPtr robot,
                                                                                   const std::string& prop_string);
};

#endif
