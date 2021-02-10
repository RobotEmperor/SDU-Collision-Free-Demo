/*
 * force_constraint.cpp
 *
 *  Created on: Feb 9, 2021
 *      Author: yik
 */
#include <ur10e_collision_free_demo/force_constraint.h>
#include <iostream>


using namespace util;
using namespace Eigen;


VectorXd ForceConstraint::operator()(const VectorXd& current_joints_pos) const
{
  static Eigen::VectorXd previous_joints_pos(6);
  //initial joint pose
  Eigen::VectorXd joint_init_pos(6);
  joint_init_pos(0) = 2.97326;
  joint_init_pos(1) = -1.6538;
  joint_init_pos(2) = -2.33488;
  joint_init_pos(3) = -2.28384;
  joint_init_pos(4) = -2.53001;
  joint_init_pos(5) = -3.13221;

  std::vector<std::string> joint_names;
  joint_names.push_back("a_shoulder_pan_joint");
  joint_names.push_back("a_shoulder_lift_joint");
  joint_names.push_back("a_elbow_joint");
  joint_names.push_back("a_wrist_1_joint");
  joint_names.push_back("a_wrist_2_joint");
  joint_names.push_back("a_wrist_3_joint");

  std::string link = "a_final_link_2";
  Eigen::Isometry3d current_pose = env_->getCurrentState()->link_transforms.at(link);
  Eigen::Isometry3d initial_pose_;

  auto kin = env_->getManipulatorManager()->getFwdKinematicSolver("ur10e_a");
  kin->calcFwdKin(initial_pose_, joint_init_pos);
  kin->calcFwdKin(current_pose, current_joints_pos);

  // constraint function

  double spring_constant_k_ = 5;
  double force_magnitude_ = 0;
  double desired_force_magnitude_ = 9;
  static Eigen::VectorXd violation_joints(6);
  violation_joints << 0,0,0,0,0,0;

  static std::vector<double> force_unit_vector_;

  force_magnitude_ = sqrt(spring_constant_k_*spring_constant_k_*(pow(2,initial_pose_.translation()(0) - current_pose.translation()(0))
      + pow(2,initial_pose_.translation()(1) - current_pose.translation()(1))
      + pow(2,initial_pose_.translation()(2) - current_pose.translation()(2))));

  force_unit_vector_.push_back(initial_pose_.translation()(0) - current_pose.translation()(0));
  force_unit_vector_.push_back(initial_pose_.translation()(1) - current_pose.translation()(1));
  force_unit_vector_.push_back(initial_pose_.translation()(2) - current_pose.translation()(2));

  //    for(long unsigned int num_inner_ = 0; num_inner_ < 3; num_inner_ ++)
  //    {
  //      current_object_force_torque_vector_[num_+1][num_inner_]=force_unit_vector_[num_inner_]*force_magnitude_;
  //    }
  //check
  std::cout << force_magnitude_ << std::endl;
  //printf("force_magnitude_ :: %f  ", force_magnitude_);
  force_unit_vector_.clear();

  if(force_magnitude_ - desired_force_magnitude_ >= -0.5 && force_magnitude_ - desired_force_magnitude_ <= 0.5)
  {
    violation_joints << 0,0,0,0,0,0;
    //previous_joints_pos = current_joints_pos;
    return violation_joints;
  }
  else
  {
    double temp_a = force_magnitude_ - desired_force_magnitude_ ;
    violation_joints << temp_a,temp_a,temp_a,temp_a,temp_a,temp_a;
    return violation_joints;
  }
}

void ForceConstraint::Plot(const tesseract_visualization::Visualization::Ptr& plotter, const VectorXd& dof_vals)
{
//  Isometry3d cur_pose;
//  manip_->calcFwdKin(cur_pose, dof_vals, kin_link_->link_name);
//
//  cur_pose = world_to_base_ * cur_pose * kin_link_->transform * tcp_;
//
//  Isometry3d target = pose_inv_.inverse();

//  tesseract_visualization::AxisMarker m1(cur_pose);
//  m1.setScale(Eigen::Vector3d::Constant(0.05));
//  plotter->plotMarker(m1);
//
//  tesseract_visualization::AxisMarker m2(target);
//  m2.setScale(Eigen::Vector3d::Constant(0.05));
//  plotter->plotMarker(m2);
//
//  tesseract_visualization::ArrowMarker m3(cur_pose.translation(), target.translation());
//  m3.material = std::make_shared<tesseract_scene_graph::Material>("cart_pose_error_material");
//  m3.material->color << 1, 0, 1, 1;
//  plotter->plotMarker(m3);
}


