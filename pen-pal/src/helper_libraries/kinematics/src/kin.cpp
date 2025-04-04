/** ============================================================================
 *  @file	 	kin.cpp
 *  @author		Matthew Pan (matthew.pan@queensu.ca)
 *  @date	 	Mar 15, 2024
 *  @version 	1.2 
 *  
 *  @brief Source file containing definitions of some helpful kinematics fcns
 *
 *  @section DESCRIPTION
 *
 *  @section CHANGES
 * ===========================================================================*/

#include "kinematics/kin.h"

/**
 * @brief Calculate the rotation matrix for angle about X-axis.
 * @param [in] angle about X axis (in radians). 
 * */
Matrix3d RotationAboutX(double angle) {
	// Rotation in X:
	Matrix3d R;
	R << 1,	0,			0,
		 0, cos(angle), -sin(angle),
		 0, sin(angle),  cos(angle);
	return R;
}

/**
 * @brief Calculate the rotation matrix for angle about Y-axis.
 * @param [in] angle about Y axis (in radians). 
 * */
Matrix3d RotationAboutY(double angle) {
	// Rotation in Y:
	Matrix3d R;
	R << 	cos(angle),	0,	sin(angle),
			0,			1,	0,
			-sin(angle),0,	cos(angle);
	return R;
}

/**
 * @brief Calculate the rotation matrix for angle about Z-axis.
 * @param [in] angle about Z axis (in radians). 
 * */
Matrix3d RotationAboutZ(double angle) {   
	// Rotation in Z:
	Matrix3d R;
	R << cos(angle), -sin(angle), 0,
		 sin(angle),  cos(angle), 0,
				  0,		   0, 1;
	return R;
}

/**
 * 	@brief Returns the inverse of a homogenous transform
 * 
 * 	@param [in] t homogeneous transform
 * 
 * */
Matrix4d TInv(const Matrix4d& t) {
	Matrix4d inv; 
	
	inv.topLeftCorner(3,3) = t.topLeftCorner(3,3).transpose();
	inv.topRightCorner(3,1) = -( inv.topLeftCorner(3,3) * t.topRightCorner(3,1));
	inv.block(3,0,1,4) << 0,0,0,1;
	
	return inv; 
}

/**
 * 	@brief Computes the rotational error vector for "r0-r1"
 * 
 * 	Returns e = axis * sin(angle); 
 * 
 * 	@param [in] r0 Destination rotation matrix.
 * 	@param [in] r1 Origin rotation matrix.
 * 
 * */
Vector3d RErr(const Matrix3d& r0, const Matrix3d& r1) {	
	Matrix3d Re;
	Re = r1.transpose()*r0;
	
	// Convert into an error vector (axis * sin(angle)).
	Vector3d e, temp;
	
	temp <<   Re(2,1) - Re(1,2),
			  Re(0,2) - Re(2,0),
			  Re(1,0) - Re(0,1);
	
	e = r1*0.5*temp;
	return e;
}

/**
 * 	@brief Computes the 6x1 error vector for "t0-t1"
 * 
 * 	Returns e = [x_err y_err z_err x_rot_err y_rot_err z_rot_err]; 
 * 
 * 	@param [in] t0 Destination homogeneous transformation matrix.
 * 	@param [in] t1 Origin homogeneous transform matrix.
 * 
 * */
Matrix<double,6,1> TErr(const Matrix4d& t0, const Matrix4d& t1) {
	// Combine the translation and rotation errors
	Matrix<double, 6, 1> e;
	e.block(0,0,3,1) = t0.block(0,3,3,1) - t1.block(0,3,3,1);
	e.block(3,0,3,1) = RErr(t0.topLeftCorner(3,3), t1.topLeftCorner(3,3));
	return e;
}

/**
 * 	@brief Calculate the rotation matrix for angle (angle) about the axis (axis)
 *  
 *  Converts axis-angle rotation representation into a rotation matrix
 * 
 * 	Rotation about r:
 *  R = r*r' + cos(a) (1-r*r') + sin(a) [r cross]
 * 
 * 	@param [in] axis The axis about which rotation is performed.
 * 	@param [in] angle The magnitude angle about the axis to generate the rotation.
 * 
 * */
Matrix3d RVec(const Vector3d& axis, double angle) {
	Matrix3d rrt, rx, R;

	rx  <<	0,			-axis.z(),	axis.y(),
			axis.z(),	0,			-axis.x(),
		   -axis.y(),	axis.x(),	0;
			
	rrt = axis * axis.transpose();
	R = rrt + (cos(angle) * (Matrix3d::Identity() - rrt)) + (sin(angle) * rx);
	return R;
}

/**
 * @brief Obtains the rotation matrix for the rotation defined by vectors v1-v0
 * 
 * @param [in] v0 (3 x 1) First vector (source vector - calculate rotation with reference to this vector)
 * @param [in] v1 (3 x 1) Second vector (destination vector)
*/
Matrix3d RVec(const Vector3d& v0, const Vector3d& v1) {
    //TODO: Handle if angle between v0 and v1 is - degrees
	
    Vector3d v = v0.cross(v1);
	double c = v0.dot(v1);
	
	Matrix3d R, Vx;
	R.setIdentity();
	Vx << 	   0,  -v.z(),   v.y(),
			v.z(),	  0,	-v.x(),
		   -v.y(),	v.x(),	   0;
			
	R = R + Vx + (Vx * Vx) * (1 / (1 + c) );
	return R;
}

/**
 *  @brief Projects a vector (vector) onto a plane defined by its normal 
 * (plane_normal)
 * 
 * Projects a vector (vector) onto a plane defined by its normal 
 * (plane_normal)
 * 
 * @param [in] vector The vector we want to project onto a plane.
 * @param [in] plane_normal The normal of the plane. 
 * */
Vector3d ProjectVec(const Vector3d& vector, const Vector3d& plane_normal) {
	Vector3d projected = vector - ((vector.dot(plane_normal)/plane_normal.norm()) * plane_normal);
	return projected;
}

/**
 * 	@brief Calculates an interpolated rotation between two specified rotations
 *  
 *  Compute the linearly interpolated rotation between the two
 *	R-matricies (r0) and (r1).  A value (t) of zero starts at (r0)
 *	and a value of one ends at (r1).
 * 
 * 	@param [in] r0 Starting/origin rotation matrix.
 * 	@param [in] r1 Ending/destination rotation matrix.
 *  @param [in] t Value between 0 (r0) and 1 (r1) representing a rotation 
 *  percentage between r0 and r1. 
 * 
 * */
Matrix3d RSlerp(const Matrix3d& r0, const Matrix3d& r1, double t) { 
	// Compute the relative rotation between them
	Matrix3d dR, Rmid;
	dR = r0.transpose() * r1;
	
	// Calculate the error vector which is the axis of rotation times sin(angle)
	Vector3d err;
	err <<	0.5 * ( dR(2,1) - dR(1,2) ),
			0.5 * ( dR(0,2) - dR(2,0) ),
			0.5 * ( dR(1,0) - dR(0,1) );

	// The norm of the error is (by definition) the sin(angle).  So if the
	// angle is zero or 180deg, the error will be zero length and we can
	// not recover an axis of rotation.  So then we can not interpolate.
	// Simply return either r0 or r1.

	double err_norm = err.norm();

	double eps = 0.000001; // 1e-6
	if( fabs(err_norm) < eps ) {
		if( t <= 0.5 )
			Rmid = r0;
		else
			Rmid = r1;
		return Rmid;
	}
	// Otherwise, normalize the error to get the axis.
	Vector3d axis;
	axis = err / err_norm;
	
	// Compute the angle
	double w2, w, angle;
	w2 = 0.25 * ( 1 + dR(0,0) + dR(1,1) + dR(2,2) );
	w = sqrt( fmax(w2, 0) );
	angle = 2 * acos( fmin(w, 1) );
	
	// Now we can interpolate the angle of rotation and build the interpolated 
	// rotation
	Rmid = r0 * RVec(axis, t*angle);

	return Rmid;
}

/**
 * 	@brief Calculates an interpolated transform between two specified transforms
 *  
 *  Compute the linearly interpolated transform between the two
 *	t-matricies (t0) and (t1).  A value (t) of zero starts at (t0)
 *	and a value of one ends at (t1).
 * 
 * 	@param [in] t0 Starting/origin transformation matrix.
 * 	@param [in] t1 Ending/destination transformation matrix.
 *  @param [in] t Value between 0 (r0) and 1 (r1) representing a transform 
 *  percentage between r0 and r1. 
 * 
 * */
Matrix4d TSlerp(const Matrix4d& t0, const Matrix4d& t1, double t) {
	Matrix4d Tmid;
	Tmid.block(0,0,3,3) = RSlerp(t0.block(0,0,3,3), t1.block(0,0,3,3), t);
	Tmid.block(0,3,3,1) = t0.block(0,3,3,1)*(1.0 - t) + t1.block(0,3,3,1)*t;
	Tmid.block(3,0,1,4) << 0, 0, 0, 1;
	return Tmid;
}

/**
 *	@brief Computes the Euclidian distance between two transformation matricies.
 *  @param [in] t1 First transform.
 *  @param [in] t2 Second transform.  
 * */
double ComputeDistance(const Matrix4d& t1, const Matrix4d& t2) {
	return( (t1.block(0,3,3,1) - t2.block(0,3,3,1)).norm() );
}
double ComputeDistance(const Vector4d& v0, const Vector4d& v1) {
	return( (v0.block(0,0,3,1) - v1.block(0,0,3,1)).norm() );
}
double ComputeDistance(const Vector3d& v0, const Vector3d& v1) {
	return( (v0-v1).norm() );
}

/**
 *	@brief Function to convert (X,Y,Z,A,B,C) KUKA Euler angle representation to 
 *  T matrix. 
 * 
 *  Rotation about Z (A) is applied first, followed by Y (B) and X (C).
 *  @param [in] X Cartesian coordinate X of frame origin.
 *  @param [in] Y Cartesian coordinate Y of frame origin.
 *  @param [in] Z Cartesian coordinate Z of frame origin.
 *  @param [in] A Rotation about the global Z axis of the frame.
 *  @param [in] B Rotation about the global Y axis of the frame. 
 *  @param [in] C Rotation about the global Z axis of the frame. 
 * */
Matrix4d EulerToTmatrix(double x, double y, double z,
						double a, double b, double c) {
	Matrix4d T_out = Matrix4d::Zero();
	// Position
	T_out.block(0,3,4,1) << x, y, z, 1.0;
	// Rotation matrix
	Matrix3d R_out = Matrix3d::Identity();
	R_out = RotationAboutX(c) * RotationAboutY(b) * RotationAboutZ(a);
	
	T_out.topLeftCorner(3,3) = R_out;
	
	return T_out;
}

/**
 * @brief Concatenates two quaternion rotations q0 then q1 into a single rotation
 * 
 * @param [in] q0 (6 x 1) First quaternion
 * @param [in] q1 (3 x 1) Second quaternion
*/
Quaterniond QConcat(const Quaterniond& q0, const Quaterniond& q1){
	Quaterniond qout;
	qout.w() = q0.w()*q1.w() - q0.x()*q1.x() - q0.y()*q1.y() - q0.z()*q1.z();
	qout.x() = q0.w()*q1.x() + q0.x()*q1.w() + q0.y()*q1.z() - q0.z()*q1.y();
	qout.y() = q0.w()*q1.y() + q0.y()*q1.w() + q0.z()*q1.x() - q0.x()*q1.z();
	qout.z() = q0.w()*q1.z() + q0.z()*q1.w() + q0.x()*q1.y() - q0.y()*q1.x();
	return qout;
}

/**
 * @brief A function which, based on a start point (q_0), end point (q_3), and 
 * two control points, creates a cubic bezier, and spits out an interpolated 
 * point based on rho (between 0 and 1)
 * 
 * @param [in] q_0 (NJOINTS x 1) Start point
 * @param [in] q_1 (NJOINTS x 1) Start control point
 * @param [in] q_2 (NJOINTS x 1) End control point
 * @param [in] q_3 (NJOINTS x 1) End point
 * @param [in] rho Value between 0 and 1 representing interpolation of the
 * bezier curve
*/
VectorXd  CubicBezier(	const Ref<const VectorXd>& q_0, const Ref<const VectorXd>& q_1,
						const Ref<const VectorXd>& q_2, const Ref<const VectorXd>& q_3,
						const double rho) {		
	VectorXd bezier;
	if (rho > 1) 
		bezier = q_3;
	else if (rho < 0)
		bezier = q_0;
	else 	
		bezier  = 	pow(1-rho,3) * q_0 + 3 * pow(1-rho,2) * rho * q_1
					+ 3 * (1-rho) * pow(rho,2) * q_2 + pow(rho,3) * q_3;
	return bezier;
}