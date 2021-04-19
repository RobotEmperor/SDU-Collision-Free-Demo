/*
 * force_constraint.cpp
 *
 *  Created on: Feb 9, 2021
 *      Author: yik
 */
#include <ur10e_collision_free_demo/force_constraint.h>
#include <iostream>

using namespace rw::math;
using namespace util;
using namespace Eigen;

VectorXd ForceConstraint::operator()(const VectorXd& current_joints_pos) const
{
  tesseract_kinematics::ForwardKinematics::Ptr kin_a;
  tesseract_kinematics::ForwardKinematics::Ptr kin_b;

  //set frame to parts frame
  Transform3D<> tf_base_a_to_init_ee;
  Transform3D<> tf_base_a_to_curr_ee;

  Transform3D<> tf_base_b_to_init_ee;
  Transform3D<> tf_base_b_to_curr_ee;

  Transform3D<> tf_a_big_pulley;
  Transform3D<> tf_b_big_pulley;
  Transform3D<> tf_a_b;

  Transform3D<> tf_big_to_a_curr_ee;
  Transform3D<> tf_big_to_b_curr_ee;

  Transform3D<> tf_big_to_rubber_belt;

  Eigen::VectorXd joint_init_pos_a;
  Eigen::VectorXd joint_init_pos_b;

  std::vector<std::string> joint_names_a;
  std::vector<std::string> joint_names_b;

  std::string final_ee_link_a;
  std::string final_ee_link_b;

  Eigen::Isometry3d initial_pose_a;
  Eigen::Isometry3d initial_pose_b;

  Eigen::Isometry3d current_pose_a;
  Eigen::Isometry3d current_pose_b;

  Eigen::Quaterniond pose_quaternion_q;

  //constraint
  double curr_force_magnitude = 0;
  double desired_force_magnitude = 10;
  double belt_offset = 0.12;
  double force_offset = 0;

  //ref frame
  tf_a_big_pulley=  Transform3D<> (Vector3D<>(-0.739757210413974, 0.07993900394775277, 0.2449995438351456), EAA<>(-0.7217029684216122, -1.7591780460014375, 1.7685571865188172).toRotation3D());
  tf_b_big_pulley =  Transform3D<> (Vector3D<>(-0.6654834316385497, -0.0844570012960042, 0.2551901723057327), EAA<>(-1.484318068681165, 0.6191402790204206, -0.6254296057933952 ).toRotation3D());

  tf_a_b = tf_a_big_pulley * inverse(tf_b_big_pulley);

  tf_big_to_rubber_belt = Transform3D<> (Vector3D<>(-0.13-0.0175,0,0), EAA<>(0,0,0).toRotation3D());

  //std::cout << "tf_a_b :: " << tf_a_b << std::endl;

  joint_init_pos_a.resize(6);
  joint_init_pos_b.resize(6);

  joint_init_pos_a(0) = 2.97691;
  joint_init_pos_a(1) = -1.65511;
  joint_init_pos_a(2) = -2.33346;
  joint_init_pos_a(3) = -2.28544;
  joint_init_pos_a(4) = -2.52771;
  joint_init_pos_a(5) = -3.13402;

  joint_names_a.push_back("a_shoulder_pan_joint");
  joint_names_a.push_back("a_shoulder_lift_joint");
  joint_names_a.push_back("a_elbow_joint");
  joint_names_a.push_back("a_wrist_1_joint");
  joint_names_a.push_back("a_wrist_2_joint");
  joint_names_a.push_back("a_wrist_3_joint");

  joint_init_pos_b(0) =  3.86019;
  joint_init_pos_b(1) =  -2.31443;
  joint_init_pos_b(2) =  -1.28901;
  joint_init_pos_b(3) =  -2.67455;
  joint_init_pos_b(4) =  1.07796;
  joint_init_pos_b(5) =  3.13539;

  joint_names_b.push_back("b_shoulder_pan_joint");
  joint_names_b.push_back("b_shoulder_lift_joint");
  joint_names_b.push_back("b_elbow_joint");
  joint_names_b.push_back("b_wrist_1_joint");
  joint_names_b.push_back("b_wrist_2_joint");
  joint_names_b.push_back("b_wrist_3_joint");

  final_ee_link_a = "a_final_link_1";
  final_ee_link_b = "b_final_link_1";

  kin_a = env_->getManipulatorManager()->getFwdKinematicSolver("ur10e_a");
  kin_a->calcFwdKin(initial_pose_a, joint_init_pos_a);

  kin_b = env_->getManipulatorManager()->getFwdKinematicSolver("ur10e_b");
  kin_b->calcFwdKin(initial_pose_b, joint_init_pos_b);

  belt_offset = 0.12;
  static Eigen::VectorXd violation(1);

  current_pose_a = env_->getCurrentState()->link_transforms.at(final_ee_link_a);
  current_pose_b = env_->getCurrentState()->link_transforms.at(final_ee_link_b);

  if(!set_robot_type) // a
  {

    //    initial_position_a_ = Transform3D<> (Vector3D<>(-0.13, 0, -0.04), RPY<> (0,0,0).toRotation3D()); // RPY
    //    initial_position_a_ = tf_a_big_pulley * initial_position_a_ ;
    //
    //    initial_position_b_ = Transform3D<> (Vector3D<>(0.02, 0, -0.04), RPY<> (0,-25*DEGREE2RADIAN,0).toRotation3D()); // RPY
    //    initial_position_b_ = tf_b_big_pulley * initial_position_b_ ;

    //    std::cout << "A :: ---------------------------- " << std::endl;
    //
    //    std::cout << (initial_position_a_.P())[0] << std::endl;
    //    std::cout << (initial_position_a_.P())[1] << std::endl;
    //    std::cout << (initial_position_a_.P())[2] << std::endl;
    //
    //    std::cout << EAA<> (initial_position_a_.R())[0] << std::endl;
    //    std::cout << EAA<> (initial_position_a_.R())[1] << std::endl;
    //    std::cout << EAA<> (initial_position_a_.R())[2] << std::endl;
    //
    //    std::cout << "B :: ---------------------------- " << std::endl;
    //
    //    std::cout << (initial_position_b_.P())[0] << std::endl;
    //    std::cout << (initial_position_b_.P())[1] << std::endl;
    //    std::cout << (initial_position_b_.P())[2] << std::endl;
    //
    //    std::cout << EAA<> (initial_position_b_.R())[0] << std::endl;
    //    std::cout << EAA<> (initial_position_b_.R())[1] << std::endl;
    //    std::cout << EAA<> (initial_position_b_.R())[2] << std::endl;

    kin_a->calcFwdKin(current_pose_a, current_joints_pos);
    kin_b->calcFwdKin(current_pose_b, joint_init_pos_b);

    pose_quaternion_q = (Eigen::Quaterniond)current_pose_a.linear();
    tf_base_a_to_curr_ee = Transform3D<> (Vector3D<>(current_pose_a.translation()(0), current_pose_a.translation()(1), current_pose_a.translation()(2)), rw::math::Quaternion<>(pose_quaternion_q.x(),pose_quaternion_q.y(),pose_quaternion_q.z(),pose_quaternion_q.w()).toRotation3D());// xyz w

    pose_quaternion_q = (Eigen::Quaterniond)initial_pose_b.linear();
    tf_base_b_to_init_ee = Transform3D<> (Vector3D<>(initial_pose_b.translation()(0), initial_pose_b.translation()(1), initial_pose_b.translation()(2)), rw::math::Quaternion<>(pose_quaternion_q.x(),pose_quaternion_q.y(),pose_quaternion_q.z(),pose_quaternion_q.w()).toRotation3D());// xyz w

    tf_big_to_a_curr_ee = inverse(tf_a_big_pulley) * tf_base_a_to_curr_ee;
    tf_big_to_b_curr_ee = inverse(tf_b_big_pulley) * tf_base_b_to_init_ee;

    //std::cout << "tf_base_a_to_curr_ee ::  "<< tf_base_a_to_curr_ee.P()[0] << "  " << tf_base_a_to_curr_ee.P()[1] << "  " << tf_base_a_to_curr_ee.P()[2]<< std::endl;
    //std::cout << "tf_base_b_to_curr_ee ::  "<< tf_base_b_to_curr_ee.P()[0] << "  " << tf_base_b_to_curr_ee.P()[1] << "  " << tf_base_b_to_curr_ee.P()[2]<< std::endl;

    //std::cout << "tf_big_to_a_curr_ee ::  "<< tf_big_to_a_curr_ee << std::endl;
    //std::cout << "tf_big_to_b_curr_ee ::  "<< tf_big_to_b_curr_ee << std::endl;

    violation << 0;



    //HC model
    curr_force_magnitude = 1.047208039306791 *100000000*pow((abs(tf_big_to_b_curr_ee.P()[0] - tf_big_to_a_curr_ee.P()[0]) - belt_offset), 5.9659451);

    //std::cout << "All force ::  "<< curr_force_magnitude << "  " << (abs(tf_big_to_b_curr_ee.P()[0] - tf_big_to_a_curr_ee.P()[0])) << std::endl;

    double temp_a = desired_force_magnitude - curr_force_magnitude;
    violation << temp_a; //
    std::cout <<"violation ::  " << violation << std::endl;
    return violation;
  }
  else
  {

    kin_b->calcFwdKin(current_pose_b, current_joints_pos);

    pose_quaternion_q = (Eigen::Quaterniond)current_pose_b.linear();
    tf_base_b_to_curr_ee = Transform3D<> (Vector3D<>(current_pose_b.translation()(0), current_pose_b.translation()(1), current_pose_b.translation()(2)), rw::math::Quaternion<>(pose_quaternion_q.x(),pose_quaternion_q.y(),pose_quaternion_q.z(),pose_quaternion_q.w()).toRotation3D());// xyz w

    tf_big_to_b_curr_ee = inverse(tf_b_big_pulley) * tf_base_b_to_curr_ee;

    force_offset = 1.047208039306791 *100000000*pow((abs(tf_big_to_b_curr_ee.P()[0] - tf_big_to_rubber_belt.P()[0]) - belt_offset), 5.9659451);

    //HC model
    curr_force_magnitude = 1.047208039306791 *100000000*pow((abs(tf_big_to_b_curr_ee.P()[0] - tf_big_to_rubber_belt.P()[0]) - belt_offset), 5.9659451);

    std::cout << "All force ::  "<< curr_force_magnitude << "  " << (abs(tf_big_to_b_curr_ee.P()[0] - tf_big_to_rubber_belt.P()[0])) << std::endl;

    double temp_a = 12 - curr_force_magnitude;
    violation << temp_a;
    std::cout <<"violation ::  " << violation << std::endl;
    return violation;
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


