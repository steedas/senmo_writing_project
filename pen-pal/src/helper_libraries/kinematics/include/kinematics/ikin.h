/** ============================================================================
 *  @file    ikin.h
 *  @author  Vinay Chawda (vinay.chawda@disneyresearch.com)
 *  @date    Dec 31, 2015 
 *  @version 1.2 
 *  
 *  @brief Header file containing declaration of IKin() function and additional 
 *  helper functions
 *
 *  @section DESCRIPTION
 * 
 *  The calling syntax is:   qdot = IKin(xd, xd_dot, q, Kp)
 *
 *  @section CHANGES
 *  Aug 10, 2018 - Matthew Pan (matt.pan@disneyresearch.com)
 *	The IKin function has been revised to accept generic Eigen matrices to 
 *  allow for other robot configurations other than the KUKA LBR to be able to 
 *  use the function. 
 * 
 * ===========================================================================*/
#ifndef KINEMATICS__IKIN_H_
#define KINEMATICS__IKIN_H_

#include "kin.h"
#include "eigen_support.h"
#include "safety.h"
#include "fkin.h"
#include <cmath>

using namespace std;
using namespace Eigen;
//#define H_TOOL 0.228448 ///< 0.152 (link A6's length) + 0.050448m (ATI sensor + mount length) + .026m (Electromagnet + mount length)
//#define H_TOOL 0.322 	// 0.152 (link A6's length) + tool length. 
//#define H_TOOL 0.152 	// 0.152 (link A6's length)
//#define H_TOOL 0.162 	// 0.152 (link A6's length) + Calibration object length
//#define H_TOOL 0.2024 // 0.152 (link A6's length) + tool length. (ATI sensor + mount length = 0.050448m)

#define NULL_SPACE_CF_GAIN 5000

VectorXd IKin(	const Matrix4d& t_base, 
				const Ref<const MatrixX5d>& dh_params, 
				const Matrix4d& t_tool, 
				const Matrix4d& t_dest, 
				const Vector6d& xdot,
				const Ref<const VectorXd>& q_min,
				const Ref<const VectorXd>& q_max,
				const Ref<const VectorXd>& q, 
				const Matrix6d& k_p);
				
int IKin (	const Matrix4d& t_base, 
			const Ref<const MatrixX5d>& dhParams, 
			const Matrix4d& t_tool, 
			const Matrix4d& TDes, 
			const Vector6d& xdot, 
			const Ref<const VectorXd>& q_start, 
			int branch, 
			int nsteps, 
			const Ref<const MatrixXd>& Winv, 
			const Matrix6d& eps,
			const Ref<const VectorXd>& q_min,
			const Ref<const VectorXd>& q_max, 
			const Ref<const VectorXd>& qdotIkinMax,
			Ref<VectorXd> q, 
			Ref<VectorXd> qdot);
			
int IKin (	const Matrix4d& t_base, 
			const Ref<const MatrixX5d>& dhParams, 
			const Matrix4d& t_tool, 
			Matrix4d& t_dest,
			double h_tool,
			const Vector6d& xdot, 
			const Ref<const VectorXd>& q_start, 
			int branch, 
			int nsteps, 
			const Ref<const MatrixXd>& W, 
			const Ref<const MatrixXd>& Winv, 
			const Matrix6d& eps,
			const Ref<const VectorXd>& q_min,
			const Ref<const VectorXd>& q_max,
			const Ref<const VectorXd>& qdotIkinMax,
			double minWorkspaceRadius,
			double maxWorkspaceRadius,
			double wristCenterSafetySphereRadius,
			const Ref<const VectorXd>& protectedSpace,
			const Vector3d& protectedSpaceLen,
			Ref<VectorXd> q, 
			Ref<VectorXd> qdot);
			
int LBRAnalyticIKin(	const Matrix4d& t_base, 
						const Ref<const MatrixX5d>& dhParams, 
						const Matrix4d& t_tool, 
						const Matrix4d& t_dest, 
						const Ref<const VectorXd>& q_min,
						const Ref<const VectorXd>& q_max,
						Ref<VectorXd> q);
				
int lbr_humanlike_ikin(	const Matrix4d& T_base, 
						const Ref<const MatrixX5d>& dhParams, 
						const Matrix4d& T_tool, 
						const Matrix4d& T_des,
						const double ringRadius,
						const int ringCheckPoints,
						const int elbowMode,
						const Ref<const VectorXd>& q_min,
						const Ref<const VectorXd>& q_max,
						Ref<Vector3d> P_onRing,
						Ref<VectorXd> q);

int isInsideProtectedSpace(	const Matrix4d& T_protected,
							double xdim, 
							double ydim, 
							double zdim, 
							double HTool,
							double wristCenterSafetySphereRadius,
							Matrix4d& T);
							


#endif // KINEMATICS__IKIN_H_



