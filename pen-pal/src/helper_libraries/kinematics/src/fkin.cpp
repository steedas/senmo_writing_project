/** ============================================================================
 *  @file    fkin.cpp
 *  @author  Vinay Chawda (vinay.chawda@disneyresearch.com)
 *  @date    Jan 05, 2016 
 *  @version 1.2 
 *  
 *  @brief Source file containing definition of fkin() function
 *
 *  @section DESCRIPTION
 * 
 *  The calling syntax is:   t = fkin(t_base, dh, t_tool, q, &j)
 *  
 *  @section CHANGES
 *  Aug 8, 2018 - Matthew Pan (matt.pan@disneyresearch.com)
 *	The fkin function has been revised to accept generic Eigen matrices to 
 *  allow for other robot configurations other than the KUKA LBR to be able to 
 *  use the function. 
 * 
 *  Mar 15, 2024 - Matthew Pan (matthew.pan@queensu.ca)
 *  Tried to make names conform to Google's C++ Style Guide 
 * 
 * ===========================================================================*/
 
#include "kinematics/fkin.h"

/** Function to calculate forward kinematics 
 * @param [in] t_base The transform of the robot base.
 * @param [in] dh [N x 5] Denavit-Hartenberg parameters of the robot.
 * @param [in] t_tool [4 x 4] TCP (Tool Center Point) transformation matrix w.r.t. end effector.
 * @param [in] q [N x 1] Vector of joint angles. 
 * @param [out] j [6 x N] Pointer to Jacobian matrix.  
 * @param [out] t Transformation matrix of the TCP with respect to base. 
*/
Matrix4d FKin(	const Matrix4d& t_base, const Ref<const MatrixX5d>& dh, 
				const Matrix4d& t_tool, const Ref<const VectorXd>& q, 
				Ref<Matrix6Xd> j) {

	const int n_joint = dh.rows();

	//printf("DH in Fkin:\n %s\n", EigenMatrixToString(dh).c_str());

	// Sanity checks
	int errors = ROBOT_SAFETY::InputIntegrityCheck(t_base, dh, t_tool, Matrix4d().setZero(), 
		q, VectorXd().setZero(), VectorXd().setZero(), j); 

	if(errors > 0)
		return Matrix4d::Zero();

	// We are going to walk down the chain in reverse order. This allows us to 
	// obtain the Jacobian with very little effort. 
	
	//So invert the base and tool transforms.
	Matrix4d t_inv_base;
	t_inv_base.topLeftCorner(3,3) = t_base.topLeftCorner(3,3).transpose();
	t_inv_base.block(0,3,3,1) = -t_base.topLeftCorner(3,3).transpose()*t_base.block(0,3,3,1);
	t_inv_base.block(3,0,1,4) = t_base.block(3,0,1,4);
	
	Matrix4d t_inv_tool;
	t_inv_tool.topLeftCorner(3,3) = t_tool.topLeftCorner(3,3).transpose();
	t_inv_tool.block(0,3,3,1) = -t_tool.topLeftCorner(3,3).transpose()*t_tool.block(0,3,3,1);
	t_inv_tool.block(3,0,1,4) = t_tool.block(3,0,1,4);
	
	// Prepare the Jacobian
	Matrix6Xd j_rev(6,n_joint);
	j_rev.setZero();
	
	// Start at the tool tip
	Matrix4d t_inv = t_inv_tool;
	
	// Proceed down the DH chain
	for(int i = n_joint; i > 0; i--) {
		// Pull out the DH values and precompute the sine/cosine terms.
		// In the process, add the current joint value as appropriate
		double stheta, ctheta, d, a, salpha, calpha;
		if(dh(i-1,4)) {
			// Prismatic
			stheta = sin(dh(i-1,0));
			ctheta = cos(dh(i-1,0));
			d = dh(i-1,1) + q(i-1);
		} else {
			// Revolute
			stheta = sin(dh(i-1,0) + q(i-1));
			ctheta = cos(dh(i-1,0) + q(i-1));
			d = dh(i-1,1);
		}
		a = dh(i-1,2);
		salpha = sin(dh(i-1,3));
		calpha = cos(dh(i-1,3));

		//======================================================================		

		//	 Concatenate the transforms. Note this contains two steps.
		//	 Normally, moving up the chain, the z-axis step comes first,
		//	 followed by the x-axis second.  But as we are coming down the
		//	 chain, first apply the a-alpha step along the x-axis.  Then the
		//	 d-theta step along the z-axis.  That also means, at the end of
		//	 the step we have the location and z-axis of the corresponding
		//	 joint.  This is perfect to compute the jacobian information.
		//	 Note, because each step translates and rotates about one axis,
		//	 the transform and inverse simply negate the angle and distance.
		//
		//	 Tdtheta = [ctheta  -stheta	0  0  ;...
		//				stheta   ctheta	0  0  ;...
		//				0		0		1  d  ;...
		//				0		0		0  1  ];
		//
		//	 Taalpha = [1   0		0		a  ;...
		//				0   calpha  -salpha 0  ;...
		//				0   salpha   calpha 0  ;...
		//				0   0		0		1  ];
		//
		//	 Tinvdtheta = [ctheta   stheta  0  0  ;...
		//				  -stheta   ctheta  0  0  ;...
		//				   0		0		1 -d  ;...
		//				   0		0		0  1  ];
		//
		//	 Tinvaalpha = [1   0		0		-a  ;...
		//				   0   calpha   salpha  0  ;...
		//				   0  -salpha   calpha  0  ;...
		//				   0   0		0	   	1  ];
		//
		//	 (Tdtheta * Taalpha)^-1 = Tinvaalpha * Tinvdtheta
		//====================================================================== 
		
		Matrix4d t_current_link;
		t_current_link << 	ctheta,			stheta,			0,		-1*a,
							-stheta*calpha,	ctheta*calpha,	salpha,	-1*d*salpha,
							stheta*salpha,  -ctheta*salpha,	calpha,	-1*d*calpha,
							0,				0,				0,		1;
		//printf("T_current_Link\n %s\n", EigenMatrixToString(t_current_link).c_str());
		t_inv = t_inv * t_current_link;
		//printf("T_inv\n %s\n", EigenMatrixToString(t_inv).c_str());
		// Add to the Jacobian. Note the Jacobian is expressed in tip frame
		Vector3d v1 = t_inv.block(0,3,3,1);
		Vector3d v2 = t_inv.block(0,2,3,1);
		
		j_rev.block(0,i-1,3,1) = v1.cross(v2);
		j_rev.block(3,i-1,3,1) = t_inv.block(0,2,3,1);
	}
	
	// Add the base offset
	t_inv = t_inv * t_inv_base;
	
	// And convert everything to be expressed w.r.t. the base frame
	Matrix4d t;
	t.topLeftCorner(3,3) = t_inv.topLeftCorner(3,3).transpose();
	t.block(0,3,3,1) = -t_inv.topLeftCorner(3,3).transpose()*t_inv.block(0,3,3,1);
	t.block(3,0,1,4) = t_base.block(3,0,1,4);
	
	j.topLeftCorner(3,n_joint) = t.topLeftCorner(3,3)*j_rev.topLeftCorner(3,n_joint);
	j.block(3,0,3,n_joint) = t.topLeftCorner(3,3)*j_rev.block(3,0,3,n_joint);
	
	return t;
}

/** Function to calculate forward kinematics to a specified joint
 * @param [in] t_base The transform of the robot base.
 * @param [in] dh [N x 5] Denavit-Hartenberg parameters of the robot.
 * @param [in] q [N x 1] Vector of joint angles. 
 * @param [in] joint Joint of which we want to obtain transform of. 
*/
Matrix4d FKinToJoint(	const Matrix4d & t_base, const Ref<const MatrixX5d> & dh, 
						const Ref<const VectorXd> & q, int joint) {

	// Sanity checks
	if( ROBOT_SAFETY::InputIntegrityCheck( t_base, dh, Matrix4d().setZero(), Matrix4d().setZero(), 
			q, VectorXd().setZero(), VectorXd().setZero(), Matrix6Xd().setZero()) > 0) {
		return Matrix4d::Zero();
	}

	Matrix4d t;
	double stheta, ctheta, salpha, calpha, d, a;
	
	t = t_base;
	
	int n = joint -1;
	if(n > 0) {
		for(int i = 0; i < n; i++) {
			// Prismatic
			if(dh(i,4)) {
				stheta = sin(dh(i,0));
				ctheta = cos(dh(i,0));
				d	  = dh(i,1) + q(i);
			}
			//Revolute
			else {
				stheta = sin(dh(i,0) + q(i));
				ctheta = cos(dh(i,0) + q(i));
				d	  = dh(i,1);
			}
			a	  = dh(i,2);
			salpha = sin(dh(i,3));
			calpha = cos(dh(i,3));	
			
			// Concatenate the transforms.
			Matrix4d next_t;
			next_t << 	ctheta,	-stheta*calpha,	stheta*salpha,	a*ctheta,
						stheta,	ctheta*calpha,	-ctheta*salpha,	a*stheta,
						0,		salpha,			calpha,			d,
						0,		0,				0,				1;	
			
			t = t * next_t;	
		}
	}
	return t;
}

/** Function to calculate motor position given joint position */
double GetMotorPosition(double joint_pos, double a, double phi, double p) {
	return(joint_pos - (a*sin(2*M_PI*(joint_pos + phi)/p)));
}

/** Function to calculate motor velocity given joint velocity */
double GetMotorVelocity(double joint_pos, double joint_vel, double a, double phi, double p) {
	return( joint_vel - (a*cos(2*M_PI*(joint_pos + phi)/p)*(2*M_PI*joint_vel/p)) );
}

/** Function to calculate joint position given motor parameters */
double GetJointPosition(double motor_pos, double a, double phi, double p) {
	// Returns joint position for given motor position
	
	// Typically converges in either 2 or 3 iterations for eps = 1e-6. Goes up 
	// to 4-5 iterations if eps is reduced to 1e-8 or 1e-10. 
	int Nmax = 10;
	double eps = 0.000001;
	double stepSize = a/10;
	
	double joint_pos, dF;
	double joint_pos_old = motor_pos; // Use motor position as seed
	
	for(int iter = 0; iter < Nmax; iter++) {
		dF = 1 - (a*2*M_PI/p)*cos(2*M_PI*(joint_pos_old + phi)/p);
		joint_pos = joint_pos_old - stepSize*dF;
		joint_pos = motor_pos + a*sin(2*M_PI*(joint_pos + phi)/p);
		
		if( fabs(joint_pos - joint_pos_old) < eps)
			break;
		
		if(iter == (Nmax -1))
			printf("[GetJointPosition()] Error: solution did not converge after "
				"%d iterations!\n", iter);
		
		joint_pos_old = joint_pos;
	}
	return joint_pos;
}









