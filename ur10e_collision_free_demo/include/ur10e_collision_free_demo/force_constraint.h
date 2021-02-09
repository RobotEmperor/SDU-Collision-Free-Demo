/*
 * force_constraint.h
 *
 *  Created on: Feb 9, 2021
 *      Author: yik
 */

#ifndef SDU_COLLISION_FREE_DEMO_UR10E_COLLISION_FREE_DEMO_INCLUDE_UR10E_COLLISION_FREE_DEMO_FORCE_CONSTRAINT_H_
#define SDU_COLLISION_FREE_DEMO_UR10E_COLLISION_FREE_DEMO_INCLUDE_UR10E_COLLISION_FREE_DEMO_FORCE_CONSTRAINT_H_

#include <Eigen/Core>
#include <trajopt/common.hpp>
#include "sco/modeling_utils.hpp"

using Eigen::MatrixX2d;

struct ZMPConstraint : public sco::IneqConstraint, public trajopt::Plotter {
    Eigen::VectorXd m_joint_rads;
    sco::VarVector m_vars;
    MatrixX2d m_ab, m_pts;
    Eigen::VectorXd m_c;
    ZMPConstraint(Eigen::VectorXd joint_rads, const MatrixX2d& hullpts, const sco::VarVector& vars);
    sco::DblVec value(const sco::DblVec&);
    sco::ConvexConstraints::Ptr convex(const sco::DblVec&, sco::Model* model);

};


struct StaticTorqueCost : public sco::CostFromErrFunc {
    Eigen::VectorXd m_joint_rads;
    sco::VarVector m_vars;
    StaticTorqueCost(Eigen::VectorXd joint_rads, const sco::VarVector& vars, double coeff);
};


struct FootHeightConstraint : public sco::ConstraintFromErrFunc {
    FootHeightConstraint(Eigen::VectorXd joint_rads, std::string link, double height, const sco::VarVector& vars);
};



#endif /* SDU_COLLISION_FREE_DEMO_UR10E_COLLISION_FREE_DEMO_INCLUDE_UR10E_COLLISION_FREE_DEMO_FORCE_CONSTRAINT_H_ */
