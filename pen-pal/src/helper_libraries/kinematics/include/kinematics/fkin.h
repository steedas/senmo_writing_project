/** ============================================================================
 *  @file    fkin.h
 *  @author  Vinay Chawda (vinay.chawda@disneyresearch.com)
 *  @date    Jan 05, 2016 
 *  @version 2.0 
 *  
 *  @brief Header file for fkin.cpp
 *
 *  @section DESCRIPTION
 *  The calling syntax is:   T = fkin(TBase, dh, TTool, q, &J)
 * 
 *  @section CHANGES
 *  Aug 8, 2018 - Matthew Pan (matt.pan@disneyresearch.com)
 *	The fkin function has been revised to accept generic Eigen matrices to 
 *  allow for other robot configurations other than the KUKA LBR to be able to 
 *  use the function. 
 * 
 * ===========================================================================*/

#ifndef KINEMATICS__FKIN_H_
#define KINEMATICS__FKIN_H_

#include <iostream>
#include <stdio.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "kin.h"
#include "eigen_support.h"
#include "safety.h"

// Define the parameters
#ifndef M_PI
#define M_PI 3.141592653589793
#endif

using namespace std;
using namespace Eigen;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;

Matrix4d FKin(	const Matrix4d& t_base, 
                const Ref<const MatrixX5d> & dh, 
				const Matrix4d& t_tool, 
                const Ref<const VectorXd> & q, 
				Ref<Matrix6Xd> j);
				
Matrix4d FKinToJoint(   const Matrix4d& t_base, 
                        const Ref<const MatrixX5d> & dh, 
						const Ref<const VectorXd> & q, 
                        int joint);

				
double GetJointPosition(double motor_pos, double a, double phi, double p);
double GetMotorPosition(double joint_pos, double a, double phi, double p);
double GetMotorVelocity(double joint_pos, double joint_vel, double a, double phi, double p);

#endif // KINEMATICS__FKIN_H_
