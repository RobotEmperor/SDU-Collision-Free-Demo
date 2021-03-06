/*
 * force_constraint.h
 *
 *  Created on: Feb 9, 2021
 *      Author: yik
 */

#ifndef SDU_COLLISION_FREE_DEMO_UR10E_COLLISION_FREE_DEMO_INCLUDE_UR10E_COLLISION_FREE_DEMO_FORCE_CONSTRAINT_H_
#define SDU_COLLISION_FREE_DEMO_UR10E_COLLISION_FREE_DEMO_INCLUDE_UR10E_COLLISION_FREE_DEMO_FORCE_CONSTRAINT_H_

#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>

#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/common.hpp>
#include <trajopt_sco/modeling.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <rw/math.hpp>

#define DEGREE2RADIAN M_PI/180.0
#define RADIAN2DEGREE  180.0/M_PI

using Eigen::MatrixX2d;

using namespace rw::math;
using namespace util;
using namespace Eigen;

struct ForceConstraint : public trajopt::TrajOptVectorOfVector
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    tesseract_environment::Environment::Ptr env_;
    ForceConstraint(tesseract_environment::Environment::Ptr env_2):env_(env_2){}
    void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const Eigen::VectorXd& dof_vals) override;
    void init_var();

    Eigen::VectorXd operator()(const Eigen::VectorXd& current_joints_pos) const override;

    void set_robot_(bool robot_); // 0 : a 1: b
    void set_data_(Eigen::VectorXd joint_a ,Eigen::VectorXd joint_b, Transform3D<> tf_a_ref, Transform3D<> tf_b_ref);

    bool set_robot_type = false;

    Transform3D<> tf_a_big_pulley;
    Transform3D<> tf_b_big_pulley;

    Eigen::VectorXd temp_joint_init_pos_a;
    Eigen::VectorXd temp_joint_init_pos_b;
};




#endif /* SDU_COLLISION_FREE_DEMO_UR10E_COLLISION_FREE_DEMO_INCLUDE_UR10E_COLLISION_FREE_DEMO_FORCE_CONSTRAINT_H_ */
