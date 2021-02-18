/*
 * force_constraint.cpp
 *
 *  Created on: Feb 9, 2021
 *      Author: yik
 */
#include <ur10e_collision_free_demo/force_constraint.h>
#include <iostream>
//sdu_math
#include <rw/math.hpp>


using namespace util;
using namespace Eigen;
using namespace rw::math;


VectorXd ForceConstraint::operator()(const VectorXd& current_joints_pos) const
{

  if(!set_robot_type) // a
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


    //set frame to parts frame
    Transform3D<> tf_base_a_to_init_end;
    Transform3D<> tf_base_a_to_curr_end;
    Transform3D<> tf_base_a_to_world;
    Transform3D<> tf_world_to_init_ee;
    Transform3D<> tf_world_to_curr_ee;

    Eigen::Quaterniond q = (Eigen::Quaterniond)initial_pose_.linear();

    tf_world_to_init_ee = Transform3D<> (Vector3D<>(initial_pose_.translation()(0), initial_pose_.translation()(1), initial_pose_.translation()(2)), rw::math::Quaternion<>(q.x(),q.y(),q.z(),q.w()).toRotation3D());// xyz w

    q = (Eigen::Quaterniond)current_pose.linear();

    tf_world_to_curr_ee = Transform3D<> (Vector3D<>(current_pose.translation()(0), current_pose.translation()(1), current_pose.translation()(2)), rw::math::Quaternion<>(q.x(),q.y(),q.z(),q.w()).toRotation3D());// xyz w
    tf_base_a_to_world = Transform3D<> (Vector3D<>(0, 0, -0.05), RPY<>(-180*DEGREE2RADIAN,0,0).toRotation3D());// xyz w

    tf_base_a_to_init_end = tf_base_a_to_world * tf_world_to_init_ee;
    tf_base_a_to_curr_end = tf_base_a_to_world * tf_world_to_curr_ee;


    rw::math::Transform3D<> tf_a_parts;
    rw::math::Transform3D<> tf_b_parts;
    rw::math::Transform3D<> tf_a_b;
    rw::math::Transform3D<> tf_world_to_a_base_link_;
    rw::math::Transform3D<> tf_world_to_b_base_link_;
    rw::math::Transform3D<> tf_big_to_small_;
    rw::math::Transform3D<> tf_a_to_small_;

    rw::math::Transform3D<> tf_small_to_init_ee;
    rw::math::Transform3D<> tf_small_to_final_ee;

    tf_world_to_a_base_link_ = Transform3D<> (Vector3D<>(0, 0, 0.05), RPY<> (180*DEGREE2RADIAN,0,0).toRotation3D()); // RPY

    tf_a_parts =  Transform3D<> (Vector3D<>(-0.739757210413974, 0.07993900394775277, 0.2449995438351456), EAA<>(-0.7217029684216122, -1.7591780460014375, 1.7685571865188172).toRotation3D());
    tf_b_parts =  Transform3D<> (Vector3D<>(-0.6654834316385497, -0.0844570012960042, 0.2551901723057327), EAA<>(-1.484318068681165, 0.6191402790204206, -0.6254296057933952 ).toRotation3D());

    tf_a_b = tf_a_parts * inverse(tf_b_parts);

    tf_big_to_small_ = Transform3D<> (Vector3D<>(-0.130, 0.007, 0), RPY<> (0,0,0).toRotation3D()); // RPY
    tf_a_to_small_ = tf_a_parts * tf_big_to_small_;

    tf_small_to_init_ee = inverse(tf_a_to_small_)*tf_base_a_to_init_end;
    tf_small_to_final_ee = inverse(tf_a_to_small_)*tf_base_a_to_curr_end;
    // tf_base_a_to_end

    //std::cout << "tf_base_a_to_init_end ::  "<< tf_base_a_to_init_end << std::endl;
    //std::cout << "tf_base_a_to_curr_end ::  "<< tf_base_a_to_curr_end << std::endl;

    // constraint function

    double spring_constant_k_ = 120;
    double force_magnitude_ = 0;
    double desired_force_magnitude_ = 10;
    static Eigen::VectorXd violation(1);
    violation << 0;

    static std::vector<double> force_vector_;

    force_vector_.push_back(spring_constant_k_*(tf_small_to_init_ee.P()[0] -  tf_small_to_final_ee.P()[0]));
    force_vector_.push_back(spring_constant_k_*(tf_small_to_init_ee.P()[1] -  tf_small_to_final_ee.P()[1]));
    force_vector_.push_back(spring_constant_k_*(tf_small_to_init_ee.P()[2] -  tf_small_to_final_ee.P()[2]));

    force_magnitude_ = sqrt(pow(force_vector_[0],2) + pow(force_vector_[1],2) + pow(force_vector_[2],2));

    force_vector_.clear();

    std::cout << "force_magnitude_a_ ::  "<<force_magnitude_ << std::endl;

    if(force_magnitude_ - desired_force_magnitude_ <= 0.2 &&  force_magnitude_ - desired_force_magnitude_ >= -0.2)
    {
      violation << 0;
      return violation;
    }
    else
    {
      double temp_a = desired_force_magnitude_ - force_magnitude_;
      violation << temp_a;
      std::cout << violation << std::endl;
      return violation;
    }
  }
  else
  {
    static Eigen::VectorXd previous_joints_pos(6);
    //initial joint pose
    Eigen::VectorXd joint_pos_b(6);
    joint_pos_b(0) =  3.87083;
    joint_pos_b(1) =  -2.28049;
    joint_pos_b(2) =  -1.3507;
    joint_pos_b(3) =  -2.64271;
    joint_pos_b(4) =  1.08987;
    joint_pos_b(5) =  3.12731;


    std::vector<std::string> joint_names_b;
    joint_names_b.push_back("b_shoulder_pan_joint");
    joint_names_b.push_back("b_shoulder_lift_joint");
    joint_names_b.push_back("b_elbow_joint");
    joint_names_b.push_back("b_wrist_1_joint");
    joint_names_b.push_back("b_wrist_2_joint");
    joint_names_b.push_back("b_wrist_3_joint");

    std::string link_b = "b_final_link_2";
    Eigen::Isometry3d current_pose_b = env_->getCurrentState()->link_transforms.at(link_b);
    Eigen::Isometry3d initial_pose_b;

    auto kin = env_->getManipulatorManager()->getFwdKinematicSolver("ur10e_b");
    kin->calcFwdKin(initial_pose_b, joint_pos_b);
    kin->calcFwdKin(current_pose_b, current_joints_pos);


    //set frame to parts frame
    Transform3D<> tf_base_b_to_init_end;
    Transform3D<> tf_base_b_to_curr_end;
    Transform3D<> tf_base_a_to_world;
    Transform3D<> tf_world_to_init_ee;
    Transform3D<> tf_world_to_curr_ee;
    Transform3D<> tf_a_parts;
    Transform3D<> tf_b_parts;
    Transform3D<> tf_a_b;
    Transform3D<> tf_world_to_a_base_link_;
    Transform3D<> tf_world_to_b_base_link_;
    Transform3D<> tf_big_to_small_;
    Transform3D<> tf_a_to_small_;

    Transform3D<> tf_big_to_init_ee;
    Transform3D<> tf_big_to_final_ee;


    tf_world_to_a_base_link_ = Transform3D<> (Vector3D<>(0, 0, 0.05), RPY<> (180*DEGREE2RADIAN,0,0).toRotation3D()); // RPY

    Eigen::Quaterniond q = (Eigen::Quaterniond)initial_pose_b.linear();

    tf_world_to_init_ee = Transform3D<> (Vector3D<>(initial_pose_b.translation()(0), initial_pose_b.translation()(1), initial_pose_b.translation()(2)), rw::math::Quaternion<>(q.x(),q.y(),q.z(),q.w()).toRotation3D());// xyz w

    q = (Eigen::Quaterniond)current_pose_b.linear();

    tf_world_to_curr_ee = Transform3D<> (Vector3D<>(current_pose_b.translation()(0), current_pose_b.translation()(1), current_pose_b.translation()(2)), rw::math::Quaternion<>(q.x(),q.y(),q.z(),q.w()).toRotation3D());// xyz w
    tf_base_a_to_world = Transform3D<> (Vector3D<>(0, 0, -0.05), RPY<>(-180*DEGREE2RADIAN,0,0).toRotation3D());// xyz w

    tf_base_b_to_init_end = inverse(tf_world_to_a_base_link_*tf_a_b) * tf_world_to_init_ee;
    tf_base_b_to_curr_end = inverse(tf_world_to_a_base_link_*tf_a_b) * tf_world_to_curr_ee;




    tf_a_parts =  Transform3D<> (Vector3D<>(-0.739757210413974, 0.07993900394775277, 0.2449995438351456), EAA<>(-0.7217029684216122, -1.7591780460014375, 1.7685571865188172).toRotation3D());
    tf_b_parts =  Transform3D<> (Vector3D<>(-0.6654834316385497, -0.0844570012960042, 0.2551901723057327), EAA<>(-1.484318068681165, 0.6191402790204206, -0.6254296057933952 ).toRotation3D());

    tf_a_b = tf_a_parts * inverse(tf_b_parts);

    tf_big_to_small_ = Transform3D<> (Vector3D<>(-0.130, 0.007, 0), RPY<> (0,0,0).toRotation3D()); // RPY
    tf_a_to_small_ = tf_a_parts * tf_big_to_small_;

    tf_big_to_init_ee = inverse(tf_b_parts)*inverse(tf_world_to_a_base_link_*tf_a_b)*tf_base_b_to_init_end;
    tf_big_to_final_ee = inverse(tf_b_parts)*inverse(tf_world_to_a_base_link_*tf_a_b)*tf_base_b_to_curr_end;
    // tf_base_a_to_end

    //std::cout << "tf_base_a_to_init_end ::  "<< tf_base_a_to_init_end << std::endl;
    //std::cout << "tf_base_a_to_curr_end ::  "<< tf_base_a_to_curr_end << std::endl;

    // constraint function

    double spring_constant_k_ = 120;
    double force_magnitude_ = 0;
    double desired_force_magnitude_ = 10;
    static Eigen::VectorXd violation(1);
    violation << 0;

    static std::vector<double> force_vector_;

    force_vector_.push_back(spring_constant_k_*(tf_big_to_init_ee.P()[0] -  tf_big_to_final_ee.P()[0]));
    force_vector_.push_back(spring_constant_k_*(tf_big_to_init_ee.P()[1] -  tf_big_to_final_ee.P()[1]));
    force_vector_.push_back(spring_constant_k_*(tf_big_to_init_ee.P()[2] -  tf_big_to_final_ee.P()[2]));

    force_magnitude_ = sqrt(pow(force_vector_[0],2) + pow(force_vector_[1],2) + pow(force_vector_[2],2));

    force_vector_.clear();

    std::cout << "force_magnitude_b_ ::  "<<force_magnitude_ << std::endl;

    if(force_magnitude_ - desired_force_magnitude_ <= 0.3 &&  force_magnitude_ - desired_force_magnitude_ >= -0.3)
    {
      violation << 0;
      return violation;
    }
    else
    {
      double temp_a = desired_force_magnitude_ - force_magnitude_;
      violation << temp_a;
      std::cout << violation << std::endl;
      return violation;
    }

  }

}

void ForceConstraint::set_robot_(bool robot_)// 0 : a 1: b
{
  set_robot_type = robot_;
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


