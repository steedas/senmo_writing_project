/** ============================================================================
 *  @file	 	kin.h
 *  @author		Matthew Pan (matthew.pan@queensu.ca)
 *  @date	 	Mar 15, 2024
 *  @version 	1.2 
 *  
 *  @brief Header containing fcn prototypes of some helpful kinematics fcns
 *
 *  @section DESCRIPTION
 *      Header containing fcn prototypes of some helpful kinematics fcns
 *  @section CHANGES
 * ===========================================================================*/
#ifndef KINEMATICS__KIN_H_
#define KINEMATICS__KIN_H_

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

#ifndef MY_SIGN
/** Helper function for determining sign of a variable 
 * @param [in] a Variable to check sign of.
 * @param [out] Sign If a equals negative number, 0 of a equals 0, 
 * 1 if a equals a positive number.
*/
#define mysign(a) (((a)<0) ? -1 : ((a)>0))  
#endif

Matrix3d RotationAboutX(double angle);
Matrix3d RotationAboutY(double angle);
Matrix3d RotationAboutZ(double angle);

Matrix4d TInv(const Matrix4d& T);
Vector3d RErr(const Matrix3d& r0, const Matrix3d& r1); 
Matrix<double,6,1> TErr(const Matrix4d& t0, const Matrix4d& t1);
Matrix3d RVec(const Vector3d& axis, double angle);
Matrix3d RVec(const Vector3d& v0, const Vector3d& v1);
Vector3d ProjectVec(const Vector3d& vector, const Vector3d& plane_normal);
Matrix3d RSlerp(const Matrix3d& r0, const Matrix3d& r1, double t);
Matrix4d TSlerp(const Matrix4d& t0, const Matrix4d& t1, double t);
double ComputeDistance(const Matrix4d& t0, const Matrix4d& t1);
double ComputeDistance(const Vector4d& v0, const Vector4d& v1);
double ComputeDistance(const Vector3d& v0, const Vector3d& v1);
Matrix4d EulerToTmatrix(double x, double y, double z, double a, double b, double c);
Quaterniond QConcat(const Quaterniond& q0, const Quaterniond& q1); 
VectorXd  CubicBezier(	const Ref<const VectorXd>& q_0, const Ref<const VectorXd>& q_1,
						const Ref<const VectorXd>& q_2, const Ref<const VectorXd>& q_3,
						const double rho);

#endif