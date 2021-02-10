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

using Eigen::MatrixX2d;

struct ForceConstraint : public trajopt::TrajOptVectorOfVector
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    tesseract_environment::Environment::Ptr env_;

    ForceConstraint(tesseract_environment::Environment::Ptr env_2):env_(env_2){}

    void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const Eigen::VectorXd& dof_vals) override;

    Eigen::VectorXd operator()(const Eigen::VectorXd& current_joints_pos) const override;
};




#endif /* SDU_COLLISION_FREE_DEMO_UR10E_COLLISION_FREE_DEMO_INCLUDE_UR10E_COLLISION_FREE_DEMO_FORCE_CONSTRAINT_H_ */
