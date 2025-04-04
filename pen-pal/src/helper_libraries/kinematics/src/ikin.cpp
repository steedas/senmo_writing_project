/** ============================================================================
 *  @file	 	ikin.cpp
 *  @author		Vinay Chawda (vinay.chawda@disneyresearch.com)
 *  @date	 	Jan 4, 2015 
 *  @version 	1.1 
 *  
 *  @brief Source file containing definition of IKin() function and additional 
 *  helper functions
 *
 *  @section DESCRIPTION
 * 
 *  The calling syntax is:   qdot = IKin(xd, xdot, q, k_p)
 *
 *  @section CHANGES
 *  Aug 10, 2018 - Matthew Pan (matt.pan@disneyresearch.com)
 *	The IKin function has been revised to accept generic Eigen matrices to 
 *  allow for other robot configurations other than the KUKA LBR to be able to 
 *  use the function. 
 *	Nov 8, 2018 - Matthew Pan (matt.pan@disneyresearch.com)
 * 	Created an analytic IKin solver specific to the KUKA LBR iiwa R700
 * ===========================================================================*/
 
#include "kinematics/ikin.h"

/**
 * @brief Performs one step inverse kinematics and outputs joint velocities. 
 * Returns joint velocities. 
 * 
 * Computes the Jacobian pseudoinverse using Singular Value Decomposition (SVD). 
 * NOTE: Consider implementing Damped Least Squares method (Levenberg-Marquardt 
 * algorithm) to avoid issues near singularities.
 * 
 * @param [in] t_base (4 x 4) The transform for the base of the robot. 
 * @param [in] dh_params (n_dof x 5) DH parameters of the robot. 
 * @param [in] t_tool (4 x 4) The transform of the tool with respect to the last 
 * joint.
 * @param [in] t_dest (4 x 4) The destination transformation matrix (where we want 
 * the end effector to be). 
 * @param [in] xdot (6 x 1) Vector of desired cartesian trajectories. 
 * @param [in] q_min (N x 1) Minimum joint limits.
 * @param [in] q_max (N x 1) Maximum joint limits.
 * @param [in] q (ndof x 1) Vector of joint positions computed in the last step 
 * (current joint angles). 
 * @param [in] k_p (6 x 6) Inverse kinematics gain matrix.
*/
VectorXd IKin(	const Matrix4d& t_base, 
				const Ref<const MatrixX5d>& dh_params, 
				const Matrix4d& t_tool, 
				const Matrix4d& t_dest, 
				const Vector6d& xdot,
				const Ref<const VectorXd>& q_min,
				const Ref<const VectorXd>& q_max,
				const Ref<const VectorXd>& q, 
				const Matrix6d& k_p) {
				
	const int n_joints = dh_params.rows();

	
	
	if (ROBOT_SAFETY::InputIntegrityCheck(t_base, dh_params, t_tool, t_dest, q_min, q_max, q, Matrix6Xd().setZero()) > 0) {
		printf("[IKin] Returned with errors. See above.");
		return VectorXd::Zero(dh_params.rows());
	}

	// Compute fkin based on previous step's q
	Matrix4d t;
	Matrix<double,6,Dynamic> j(6, n_joints);

	// call the fkin function 
	t = FKin(t_base, dh_params, t_tool, q, j);
  
	Matrix<double, 6, 1> err_x; 
	err_x = TErr(t_dest, t);

	Matrix<double, Dynamic, 6> j_inv(n_joints,6); //7x6
	VectorXd qdot(n_joints), q_0_dot(n_joints); //7x1
	
	Matrix6d identity_6d;
	MatrixXd identity_Nd(n_joints,n_joints); //7x7
	identity_6d.setIdentity();
	identity_Nd.setIdentity();
	
	// Compute pseudo inverse using Singular Value Decomposition (SVD). 
	// NOTE: Consider implementing Damped Least Squares method 
	// (Levenberg-Marquardt algorithm) to avoid issues near singularities
	// j_inv = j.jacobiSvd(ComputeThinU | ComputeThinV).solve(identity_6d); 
	j_inv = j.jacobiSvd( ComputeFullU | ComputeFullV ).solve(identity_6d); 
	
	// Minimizing the norm of joint velocities by setting q_0_dot = zero vector, 
	// can be changed to achieve a different objective
	q_0_dot.setZero(); 
	
	// Cost function to stay close to joint centers
	VectorXd q_avg; //7x1 //changed to 6x1
	q_avg = (q_min + q_max)/2;

	//changed index max to 6, average to 6.o instead of 7
	for(int i = 0; i < 6; i++) {
		//(-NULL_SPACE_CF_GAIN/7) (q_i-q_avg)/q_range^2
		q_0_dot(i,0) = 	(double)NULL_SPACE_CF_GAIN * (-1.0/6.0) * (q(i,0) - q_avg(i,0)) 
						/ ((q_max(i,0) - q_min(i,0)) * (q_max(i,0) - q_min(i,0)));
	}
	
	//qdot = (j.transpose())*kIkin*(err_x);
	qdot = j_inv*(xdot + k_p * err_x) + (identity_Nd - j_inv * j) * q_0_dot;
	
	/*
	printf("err_x: %f %f %f %f %f %f \n", err_x(0,0), err_x(1,0), err_x(2,0), 
		err_x(3,0), err_x(4,0), err_x(5,0));
	printf("qdot: %f %f %f %f %f %f %f\n", qdot(0,0), qdot(1,0), qdot(2,0), 
		qdot(3,0), qdot(4,0), qdot(5,0), qdot(6,0));
	*/
	return qdot;
}

/**
 * @brief Performs iterative inverse kinematics for a desired cartesian position 
 * and velocity using a weighted pseudo inverse method, outputs both joint angles 
 * and joint velocities.
 * 
 * Function to perform inverse kinematics and return q and qdot for a desired 
 * cartesian position and velocity using a weighted pseudo inverse method.
 * 
 * Output:
 * 		1: IKin solution is feasible
 * 		0: IKin solution is infeasible
 * 		-1: IKin solution has not converged 
 * 
 * @param [in] t_base (4 x 4) The transform for the base of the robot. 
 * @param [in] dh_params (ndof x 5) DH parameters of the robot. 
 * @param [in] t_tool (4 x 4) The transform of the tool with respect to the last 
 * joint.
 * @param [in] t_dest (4 x 4) The destination transformation matrix (where we want 
 * the end effector to be). 
 * @param [in] xdot (6 x 1) Vector of desired cartesian trajectories. 
 * @param [in] q_start (ndof x 1) Vector of joint positions computed in the last 
 * step (current joint angles). 
 * @param [in] branch A bitwise interpreted value from 0 to 7 representing which 
 * kineamtic branch to find the solution in. Each bit represents joints q2, q4, 
 * and q6. A bit value of 0 denotes that the IKin solution should be on the 
 * negative side of the joint; whereas 1 represents positive side of the joint. 
 * Note that the q_start joint angles must be within the selected branch for 
 * the IKin to properly run. 
 * @param [in] n_steps The maximum number of steps that should be taken by the 
 * solver to reach a solution. 
 * @param [in] w_inv (N x N) Inverse kinematic weights.
 * @param [in] eps (6 x 6) eps Damping parameters.
 * @param [in] q_min (N x 1) Min joint positions.
 * @param [in] q_max (N x 1) Max joint positions.
 * @param [out] q (N x 1) Vector of IKin solved joint angles corresponding to 
 * the t_dest pose.
 * @param [out] qdot (N x 1) Vector of IKin solved joint velocities 
 * corresponding to xdot cartesian velocity vector.
*/
int IKin (	const Matrix4d& t_base, 
			const Ref<const MatrixX5d>& dh_params, 
			const Matrix4d& t_tool, 
			const Matrix4d& t_dest, 
			const Matrix<double, 6, 1>& xdot, 
			const Ref<const VectorXd>& q_start, 
			int branch, 
			int n_steps, 
			const Ref<const MatrixXd>& w_inv, 
			const Matrix6d& eps,
			const Ref<const VectorXd>& q_min,
			const Ref<const VectorXd>& q_max, 
			const Ref<const VectorXd>& qdot_max,
			Ref<VectorXd> q, 
			Ref<VectorXd> qdot) {
		
	const int n_joints = dh_params.rows();
	
	if (ROBOT_SAFETY::InputIntegrityCheck(t_base, dh_params, t_tool, t_dest, q_min, q_max, q, Matrix6Xd().setZero()) > 0) {
		printf("[IKin] Returned with errors. See above.");
		return -1;
	}

	// Function to perform inverse kinematics and return q and qdot for a 
	// desired cartesian position and velocity using a weighted pseudo inverse 
	// method
	// Output:
	//			1: feasible
	//			0: infeasible
	//			-1: solution not converged 
	
	int is_feasible = 1;
	// Initialize q_max and q_min
	VectorXd q_min_actual = q_min; //7x1
	VectorXd q_max_actual = q_max; //7x1

	// Sanity check if branch is valid
	if( (branch < 0) || (branch > 7) ) {
		printf("[IKin] Illegal branch input\n. Setting branch to 7\n");
		branch = 7;
	}
	
	if(branch & 0x01) {
		// q2 (+)
        q_min_actual(1) = q_min(1);
		q_max_actual(1) = q_max(1);
	} else {
		// q2 (-)
		q_min_actual(1) = -q_max(1);
		q_max_actual(1) = -q_min(1);
	}
	
    if(branch & 0x02) {
		// q4 (+)
		q_min_actual(3) = q_min(3);
    	q_max_actual(3) = q_max(3);
	} else {
		// q4 (-)
		q_min_actual(3) = -q_max(3);
		q_max_actual(3) = -q_min(3);
	} 
    
    if(branch & 0x04) {
		// q6 (+)
    	q_min_actual(5) = q_min(5);
		q_max_actual(5) = q_max(5);

	} else {
		// q6 (-)
		q_min_actual(5) = -q_max(5);
		q_max_actual(5) = -q_min(5);

	}
	
	if(!ROBOT_SAFETY::IsInsideJointLimits(q_start, q_max_actual, q_min_actual)) {
		printf("[IKin] q_start is outside the selected branch!\n");
		return 0;
	}

	VectorXd q_avg = (q_max_actual + q_min_actual)/2;
	q = q_start; // initialize q to q_start
	Matrix4d t;
	Matrix<double, 6, Dynamic> j(6, n_joints); 
	j = Matrix<double, 6, Dynamic>::Zero(6, n_joints);

	Matrix<double, 6, 1> delta_x;
	Matrix<double, Dynamic, 6> j_inv(n_joints, 6);
	Matrix6d identity_6d;
	identity_6d.setIdentity();


	// printf("T_Base:\n %s\n", EigenMatrixToString(t_base).c_str());
	// printf("DH_PARAMS::\n %s\n", EigenMatrixToString(dh_params).c_str());
	// printf("T_tool::\n %s\n", EigenMatrixToString(t_tool).c_str());
	// printf("T_dest:\n %s\n", EigenMatrixToString(t_dest).c_str());
	// printf("Xdot:\n %s\n", EigenMatrixToString(xdot).c_str());
	// printf("Q_start\n %s\n", EigenMatrixToString(q_start).c_str());
	// printf("Branch: %d\n", branch);
	// printf("N_steps: %d\n", n_steps);
	// printf("Weighted inverse\n %s\n", EigenMatrixToString(w_inv).c_str());
	// printf("Error Pos\n %s\n", EigenMatrixToString(eps).c_str());
	// printf("Q_Min\n %s\n", EigenMatrixToString(q_min).c_str());
	// printf("Q_Max\n %s\n", EigenMatrixToString(q_max).c_str());
	// printf("Q_Dot_MAx\n %s\n", EigenMatrixToString(qdot_max).c_str());
	


	for(int iter = 0; iter < n_steps; iter++) {
		//printf("q_current_step\n %s\n", EigenMatrixToString(q).c_str());
		// Compute fkin based on previous step's q
		t = FKin(t_base, dh_params, t_tool, q, j);
		printf("T_Current_Step\n %s\n", EigenMatrixToString(t).c_str());
		printf("q_current_step\n %s\n", EigenMatrixToString(q*(180/3.1415926)).c_str());


		// Compute delta_x
		delta_x = TErr(t_dest, t);

		printf("Normalized DeltaX: %f",delta_x.norm());

		// Compute weighted pseudo inverse
		//j_inv = WinvHalf*((j*WinvHalf).jacobiSvd( ComputeFullU | ComputeFullV ).solve(identity_6d)); 

		// Compute damped weighted pseudo inverse
		j_inv = w_inv * j.transpose() *
			((j * w_inv * j.transpose() + eps).colPivHouseholderQr().solve(identity_6d)); 

		// Compute q (Open house IKin, trying to compute a solution that stays 
		// close to q_avg always, and not previous q)
		//q = q_avg + j_inv*(delta_x + j*(q - q_avg));

		//printf("DeltaQ\n %s\n", EigenMatrixToString(j_inv*(delta_x + j * (q - q_start))).c_str());

		// Compute q: IKin tries to find a solution close to previous solution, 
		// so that joint space motion appears more continuouss
		q = q_start + j_inv*(delta_x + j * (q - q_start));
	}
	
	// Check if the IKin output is within the selected branch. If not, scale 
	// back the offending joint and set is_feasible = 0.
	//cout << "q inside_ikin: " << q.transpose() << "\n";
	if(!ROBOT_SAFETY::EnsureJointLimits(q, q_max, q_min)) {
		is_feasible = 0;
		printf("q OOB\n");
	}
	
	// Check if the cartesian position converged
	if( delta_x.norm() > 1e-3) {
		printf("Delta X not converged\n");
		is_feasible = -1;
	}
	
	// Compute qdot
	qdot = j_inv * xdot;
	
	double scale = ((qdot.cwiseQuotient(qdot_max)).cwiseAbs()).maxCoeff();
	if(scale > 1)
		qdot = qdot/scale;
	
	return is_feasible;
}

/**
 * @brief Performs iterative inverse kinematics for a desired cartesian position 
 * and velocity using a weighted pseudo inverse method, outputs both joint angles 
 * and joint velocities. Performs more safety checks than 
 * 
 * Function to perform inverse kinematics and return q and qdot for a desired 
 * cartesian position and velocity using a weighted pseudo inverse method.
 * 
 * Output:
 * 		1: IKin solution is feasible
 * 		0: IKin solution is infeasible
 * 		-1: IKin solution has not converged 
 * 
 * @param [in] t_base (4 x 4) The transform for the base of the robot. 
 * @param [in] dh_params (ndof x 5) DH parameters of the robot. 
 * @param [in] t_tool (4 x 4) The transform of the tool with respect to the last 
 * joint.
 * @param [in] t_dest (4 x 4) The destination transformation matrix (where we want 
 * the end effector to be). 
 * @param [in] h_tool Length of tool + media flange + distance to joint 6 
 * @param [in] xdot (6 x 1) Vector of desired cartesian trajectories. 
 * @param [in] q_start (ndof x 1) Vector of joint positions computed in the last 
 * step (current joint angles). 
 * @param [in] branch A bitwise interpreted value from 0 to 7 representing which 
 * kineamtic branch to find the solution in. Each bit represents joints q2, q4, 
 * and q6. A bit value of 0 denotes that the IKin solution should be on the 
 * negative side of the joint; whereas 1 represents positive side of the joint. 
 * Note that the q_start joint angles must be within the selected branch for 
 * the IKin to properly run. 
 * @param [in] n_steps The maximum number of steps that should be taken by the 
 * solver to reach a solution. 
 * @param W (N x N) Joint velocity weights.
 * @param w_inv (N x N) Inverse kinematic weights.
 * @param [in] eps (6 x 6) eps Damping parameters.
 * @param [in] q_min [N x 1] Min joint positions.
 * @param [in] q_max [N x 1] Max joint positions. 
 * @param [in] min_workspace_radius Minimum radius of the robot workspace.
 * @param [in] max_workspace_radius Maximum radius of the robot workspace.
 * @param [in] wrist_safety_radius The radius of the sphere centered 
 *  at wrist center.
 * @param [in] protected_space [N x 1] Defines the base coordinate system for the 
 * no-go zone for the robot
 * @param [in] protected_space_lengths [3 x 1] Defines the sizes of the no-go zone for the robot
 * @param [out] q (N x 1) Vector of IKin solved joint angles corresponding to 
 * the t_dest pose.
 * @param [out] qdot (N x 1) Vector of IKin solved joint velocities 
 * corresponding to xdot cartesian velocity vector.
*/
int IKin (	const Matrix4d& t_base, 
			const Ref<const MatrixX5d>& dh_params, 
			const Matrix4d& t_tool, 
			Matrix4d& t_dest,
			double h_tool,
			const Matrix<double, 6, 1>& xdot, 
			const Ref<const VectorXd>& q_start, 
			int branch, 
			int n_steps, 
			const Ref<const MatrixXd>& W, 
			const Ref<const MatrixXd>& w_inv, 
			const Matrix6d& eps,
			const Ref<const VectorXd>& q_min,
			const Ref<const VectorXd>& q_max,
			const Ref<const VectorXd>& qdot_max,
			double min_workspace_radius,
			double max_workspace_radius,
			double wrist_safety_radius,
			const Ref<const VectorXd>& protected_space,
			const Vector3d& protected_space_lengths,
			Ref<VectorXd> q, 
			Ref<VectorXd> qdot) {
	// Mirroring study version
	const int n_joints = dh_params.rows();
		
	if (ROBOT_SAFETY::InputIntegrityCheck(t_base, dh_params, t_tool, t_dest, q_min, q_max, q, Matrix6Xd().setZero()) > 0) {
		printf("[IKin] Returned with errors. See above.");
		return -1;
	} 
	
	int is_feasible = 1;
	// Initialize q_max and q_min
	VectorXd q_max_actual = q_max;
	VectorXd q_min_actual = q_min;

	// Check if t_dest is within the reachable workspace
	// radius of sphere (for wrist center, 99%) = 0.99*(0.4 + 0.4) = 0.7920
	// If not, then scale back the wrist center and update the t_dest accordingly 
	// w.r.t scaled wrist location
	
	/*
	is_feasible = isInsideSphere(t_base, 
								t_dest, 
								h_tool, 
								(double)kRobotWorkSpaceMinRadiusTheoretical, 
								(double)kRobotWorkSpaceMaxRadiusTheoretical);
								
	Matrix4d T_protected = convertEulerToTmatrix(	protectedSpace_X, 
													protectedSpace_Y, 
													protectedSpace_Z, 
													protectedSpace_A, 
													protectedSpace_B, 
													protectedSpace_C);
	int isInPSpace  = int isInsideProtectedSpace(	
							const Matrix4d& T_protected, 
							double xdim, 
							double ydim, 
							double zdim, 
							double h_tool,
							double wrist_safety_radius,
							Matrix4d& t);
	*/
	is_feasible = ROBOT_SAFETY::IsInsideSphere(t_base, t_dest, h_tool, min_workspace_radius, max_workspace_radius);
	
	Matrix4d T_protected = EulerToTmatrix(	protected_space(0), 
											protected_space(1), 
											protected_space(2), 
											protected_space(3), 
											protected_space(4), 
											protected_space(5));

	int isInPSpace = ROBOT_SAFETY::IsInsideProtectedSpace(T_protected, protected_space_lengths(0), 
											protected_space_lengths(1), protected_space_lengths(2),
											h_tool, wrist_safety_radius, t_dest);
	
	if( (is_feasible == 1) && (isInPSpace == 0) )
		is_feasible = 0;
	if( is_feasible == -1)
		return is_feasible;
	
	
	// Sanity check if branch is valid
	if( (branch < 0) || (branch > 7) ) {
		printf("cartToJoint(): Illegal branch input\n. Setting branch to 7\n");
		branch = 7;
	}
	
	if(branch & 0x01) {
		// q2 (+)
		q_max_actual(1) = q_max(1);
		q_min_actual(1) = q_min(1);
	} else {
		// q2 (-)
		q_max_actual(1) = -q_min(1);
		q_min_actual(1) = -q_max(1);
	}
	
	if(branch & 0x02) {
		// q4 (+)
		q_max_actual(3) = q_max(3);
		q_min_actual(3) = q_min(3);
	} else {
		// q4 (-)
		q_max_actual(3) = -q_min(3);
		q_min_actual(3) = -q_max(3);
	}
	
	if(branch & 0x04) {
		// q6 (+)
		q_max_actual(5) = q_max(5);
		q_min_actual(5) = q_min(5);
	} else {
		// q6 (-)
		q_max_actual(5) = -q_min(5);
		q_min_actual(5) = -q_max(5);
	}
	
	//if(!isInsideOf(q_start, q_max, q_min))
	//	cout << "q_start is outside the selected branch!\n";
				
	VectorXd q_avg = (q_max_actual + q_min_actual)/2;	   
	q = q_start; // initialize q to q_start
	Matrix4d t;
	Matrix<double, 6, Dynamic> j(6, n_joints);
	Matrix<double, 6, 1> delta_x;
	Matrix<double, Dynamic, 6> j_inv(n_joints, 6);
	Matrix6d identity_6d;
	identity_6d.setIdentity();
	MatrixXd Ident7d(n_joints, n_joints);
	Ident7d.setIdentity();
	
	for(int iter = 0; iter < n_steps; iter++) {
		// Compute fkin based on previous step's q
		t = FKin(t_base, dh_params, t_tool, q, j);
		
		// Compute delta_x
		delta_x = TErr(t_dest, t);
		
		//DEBUG:
		//cout << "delta_x norm: " << delta_x.norm() << "\n";
		
		// Compute weighted pseudo inverse
		//j_inv = WinvHalf*((j*WinvHalf).jacobiSvd( ComputeFullU | ComputeFullV ).solve(identity_6d)); 
		
		// Compute damped weighted pseudo inverse
		j_inv = w_inv * j.transpose() *
			((j * w_inv * j.transpose() + eps).colPivHouseholderQr().solve(identity_6d)); 
		// Compute q (Open house IKin, trying to compute a solution that stays 
		// close to q_avg always, and not previous q)
		//q = q_avg + j_inv*(delta_x + j*(q - q_avg));
		
		// Compute q: IKin tries to find a solution close to previous solution, 
		// so that joint space motion appears more continuous
		//q = q_start + j_inv*(delta_x + j*(q - q_start));
		//q = q_start + j_inv*(delta_x + j*(q - q_start));
		q = q + j_inv * (delta_x);
	}
	
	// Check if the IKin output is within the selected branch. If not, scale 
	// back the offending joint and set is_feasible = 0.
	//cout << "q inside_ikin: " << q.transpose() << "\n";
	if(!ROBOT_SAFETY::EnsureJointLimits(q, q_max, q_min)) 
		is_feasible = 0;

	// Check if the cartesian position converged
	if( delta_x.norm() > 1e-3) 
		is_feasible = -1;

	// Compute qdot
	//VectorXd v;
	//v = -2.0*(q - q_start);
	//qdot = j_inv*xdot + (Ident7d - (j_inv*(j*W)))*v;
	//qdot = j_inv*(xdot + k_p*err_x) + (identity_Nd - j_inv*j)*q_0_dot;
	
	qdot = j_inv * xdot;
	double scale = ((qdot.cwiseQuotient(qdot_max)).cwiseAbs()).maxCoeff();
	if(scale > 1)
	{
		//printf("Vel OOB. Scale: %f\n", scale);
		qdot = qdot/scale;
	}
	
	/*		
	// Check if the computed velocity is within bounds
	if(!isInsideOf(qdot, qdot_max, -1*qdot_max))
	{
		//printf("Velocity OOB, scaling down\n");
		//cout << "Velocity OOB: " << qdot.transpose() << "\n";
	}
	*/
	
	return is_feasible;
}

/**
 * This IKin solver searches for an analytic solution for the KUKA lbr 
 * specifically. This assumes that the end effector and target are symmetric 
 * about their Z (roll) axis leading to a 6 DOF robot (not using A7) solving a
 * 5 DOF problem (ignoring roll). Due to the gimbal structure of the shoulder, 
 * elbow, and wrist configurations, we are able to calculate the location of the 
 * wrist gimbal. Knowing link lengths we can calculate the angle of A4 (elbow)
 * based on the wrist gimbal position. The rest of the joint angles can be 
 * determined by specifying a secondary goal.  In this case, we control the 
 * height of the elbow such that the wrist joint (A6) angle is within a certain 
 * range.
 * 
 * Output:
 * 		1: IKin solution is feasible
 * 		0: IKin solution is infeasible, joint limits exceeded, returns modified q
 * 		-1: errors in input parameters
 * 
 * @param [in] t_base (4 x 4) The transform for the base of the robot. 
 * @param [in] dh_params (ndof x 5) DH parameters of the robot. 
 * @param [in] t_tool (4 x 4) The transform of the tool with respect to the last 
 * joint.
 * @param [in] t_dest (4 x 4) The destination transformation matrix (where we want 
 * the end effector to be). 
 * @param [in] q_min [N x 1] Min joint positions.
 * @param [in] q_max [N x 1] Max joint positions. 
 * @param [in][out] q (N x 1) Vector of IKin solved joint angles corresponding to 
 * the t_dest pose.
*/
int LBRAnalyticIKin(	const Matrix4d& t_base, 
				const Ref<const MatrixX5d>& dh_params, 
				const Matrix4d& t_tool, 
				const Matrix4d& t_dest, 
				const Ref<const VectorXd>& q_min,
				const Ref<const VectorXd>& q_max,
				Ref<VectorXd> q)
{
	Vector3d shoulderPos, elbowPos, wristPos;
	Vector3d desDir, shoulderToWristDir, shoulderToElbowDir, elbowToWristDir;
	Matrix4d TShoulder, TWrist;
	double a1, a2, a3, a4, a5, a6;
	double dist, elbowRiseAngle = -M_PI/2;
	int is_feasible = 1;
	
	//use information from input q to determine branch of a5
	double a5context = q(4);
	
	q << 0,0,0,0,0,0,0;
	
	if (ROBOT_SAFETY::InputIntegrityCheck(t_base, dh_params, t_tool, t_dest, q_min, q_max, q, Matrix6Xd().setZero()) > 0) {
		printf("[IKin] Returned with errors. See above.");
		q = VectorXd::Zero(dh_params.rows());
		return -1;
	}

	//find location of shoulder gimbal
	TShoulder = t_base;
	shoulderPos = TShoulder.block(0,3,3,1);
	
	//find location of wrist gimbal
	{
		Vector3d wristCart;
		desDir = t_dest.topLeftCorner(3,3) * t_tool.topRightCorner(3,1);	
		wristCart = t_dest.topRightCorner(3,1) - desDir;
		TWrist.topLeftCorner(3,3) = t_dest.topLeftCorner(3,3);
		TWrist.topRightCorner(3,1) = wristCart; 
		TWrist.block(3,0,1,4) << 0, 0, 0, 1;
		wristPos = TWrist.block(0,3,3,1);
		dist = (wristPos - shoulderPos).norm();
		
		if(dist > 0.8)
		{
			is_feasible = 0;
			//scale back wrist location to point on sphere 
			wristPos = ((wristPos - shoulderPos).normalized())*0.8 + shoulderPos;
			dist = 0.8;
		}
	}
	
	//algorithm for figuring out elbow rise angle (loosely based off of human phys)
	elbowRiseAngle = -M_PI/2;
	
	if( wristPos.x() < 0 && wristPos.x() >= -0.4)
		elbowRiseAngle = -M_PI/4 + wristPos.x()*(M_PI - M_PI/2)/0.8;
	else if( wristPos.x() < -0.4 && wristPos.x() >= -0.4)
		elbowRiseAngle = -wristPos.x()*(M_PI - M_PI/2)/0.8 - M_PI/2;
	else 
		elbowRiseAngle = -M_PI/4;
	
	//algorithm to obtain position of elbow gimbal with specific elbow rise angle 
	{
		Vector3d halfShoulderToWristPos = (wristPos - shoulderPos) / 2;
		shoulderToWristDir = (wristPos - shoulderPos).normalized();
		Vector3d halfToElbow, shoulderToWristPerpendicular;
		Matrix3d riseRotation = RVec(shoulderToWristDir, elbowRiseAngle);
		shoulderToWristPerpendicular << 0, 
										-shoulderToWristDir(2) * mysign(shoulderToWristDir(1)),
										abs(shoulderToWristDir(1));
		shoulderToWristPerpendicular.normalize();
		halfToElbow = riseRotation * shoulderToWristPerpendicular;
		halfToElbow = sqrt(pow(0.4,2) - pow(dist/2,2)) * halfToElbow;
		elbowPos =  shoulderPos + halfShoulderToWristPos + halfToElbow;
	}
	
	desDir.normalize();
	shoulderToElbowDir = (elbowPos-shoulderPos).normalized();
	elbowToWristDir = (wristPos-elbowPos).normalized();
	/*
	cout << " Elbow: " << elbowPos.transpose() 
		 << " Wrist: " << wristPos.transpose()
		 << " Dist: " << dist 
		 << " Rise: " << elbowRiseAngle*180/M_PI << "\n";
	*/
	//checks
	if( abs((elbowPos-shoulderPos).norm() - 0.4) > 0.001)
		printf("[IKin] Shoulder to elbow length is not 0.4.\n");
	if ( abs((wristPos-elbowPos).norm() - 0.4) > 0.001)
		printf("[IKin] Elbow to wrist length is not 0.4.\n");
		
	//calculate a1 & a2
	{
		Vector3d shoulderToElbowXY,shoulderToElbowXZ;
		shoulderToElbowXY << shoulderToElbowDir.x(), shoulderToElbowDir.y(), 0;
		shoulderToElbowXY.normalize(); 
		a1 = acos(shoulderToElbowXY.dot(-Vector3d::UnitX()));
		a2 = acos((elbowPos.z() - shoulderPos.z())/0.4);
		
		if(shoulderToElbowDir.y() >= 0)
			{a1 = -a1; a2 = -a2;}
		else
			{a2 = -a2;}
		
		q(0) = a1; q(1) = a2;
	}
	
	//we need to find a4 before attempting to calculate a3, since we can only 
	//create a desired vector once the elbow is raised
	
	//calulate a4 (elbow angle)
	a4 = (M_PI-2*asin(dist/(2*0.4))); //since d3 and d5 are both 0.4
	if(wristPos.y() >= 0)
		a4 = -a4;
	q(3) = a4;
	
	//calculate a3
	{
		Matrix4d T5_current = FKinToJoint(t_base, dh_params, q, 5); 
		Vector3d z3_current = shoulderToElbowDir;
		Vector3d z5_current = T5_current.block(0,2,3,1); //T5_current.topLeftCorner(3,3) * Vector3d::UnitZ();	
		Vector3d z5_desired = elbowToWristDir;
		//project onto the a3 joint plane defined by its normal shoulderToElbowDir
		z5_current = (z5_current - (z5_current.dot(z3_current) * z3_current)).normalized();
		z5_desired = (z5_desired - (z5_desired.dot(z3_current) * z3_current)).normalized();
	
		a3 = acos(z5_current.dot(z5_desired));
		if(z5_current.dot(z5_desired.cross(z3_current)) < 0)
			a3 = -a3;
		q(2) = a3;	
	}

	//calculate a5
	{
		Matrix4d T6_current = FKinToJoint(t_base, dh_params, q, 6); 
		Vector3d z5_current = elbowToWristDir;
		Vector3d z6_current = -T6_current.block(0,0,3,1);//T6_current.topLeftCorner(3,3) * (-Vector3d::UnitX());
		Vector3d z6_desired = t_dest.block(0,2,3,1);//t_dest.topLeftCorner(3,3) * Vector3d::UnitZ();
		//project z6_desired onto a5 joint plane defined by normal jointNormal
		z6_desired = (z6_desired - (z6_desired.dot(z5_current) * z5_current)).normalized();

		a5 = acos(z6_current.dot(z6_desired));
		if( z6_current.dot(z6_desired.cross(z5_current)) < 0)
			a5 = -a5;	 

		if ( a5 >= 100 * M_PI /180)
		{
			if (abs(a5context-a5) > abs(a5context-(a5 - M_PI)) && a5context != 0)	
				a5 = a5 - M_PI;
			else if (a5context == 0 ) 
				a5 = a5 - M_PI;
		}
		else if ( a5 <= -100 * M_PI /180)
		{
			if (abs(a5context-a5) > abs(a5context-(a5 + M_PI)) && a5context != 0)	
				a5 = a5 + M_PI;
			else if (a5context == 0 ) 
				a5 = a5 + M_PI;
		}
		
		q(4) = a5;
	}
	
	//calculate a6
	{
		Matrix4d T6_current = FKinToJoint(t_base, dh_params, q, 6); 
		
		a6 = acos(elbowToWristDir.dot((Vector3d)t_dest.block(0,2,3,1)));
		Vector3d desired = t_dest.block(0,2,3,1);
		if( elbowToWristDir.dot(desired.cross((Vector3d)T6_current.block(0,2,3,1))) < 0)
			a6 = -a6;
		q(5) = a6;
	}

	if(!ROBOT_SAFETY::EnsureJointLimits(q, q_max, q_min))
		return 0;
	else
		return is_feasible;
}

/**
 * This IKin solver searches for an analytic solution for the KUKA lbr 
 * specifically with the following configuration:
 * 
 * A1 - Torso bending (forward/backwards)
 * A2 - Torso twist 
 * A3 - Shoulder flexion/extension (raise/lower)
 * A4 - Shoulder abduction/adduction (away from torso/towards torso) [Location of shoulder gimbal]
 * A5 - Elbow external/internal rotation (away from torso/towards torso)
 * A6 - Elbow flexion/extension (raise/lower) [Location of elbow gimbal]
 * A7 - Forearm pronation/supination (palm down/palm up) 
 * 
 * Output:
 * 		1: IKin solution is feasible
 * 		0: IKin solution is infeasible, joint limits exceeded, returns modified q
 * 		-1: errors in input parameters
 * 
 * @param [in] t_base (4 x 4) The transform for the base of the robot. 
 * @param [in] dh_params (ndof x 5) DH parameters of the robot. 
 * @param [in] t_tool (4 x 4) The transform of the tool with respect to the last 
 * joint.
 * @param [in] t_dest (4 x 4) The destination transformation matrix (where we want 
 * the end effector to be). 
 * @param [in] ring_radius The radius of the handover object ring.
 * @param [in] ring_checkpoints The number of positions on the half of the ring 
 * closest to the robot to check.
 * @param [in] elbow_mode Specifies whether the elbow position should be the lowest 
 * possible (mode 0), or closest to the previous elbow position specified in q 
 * (mode 1).
 * @param [in] q_min [N x 1] Min joint positions.
 * @param [in] q_max [N x 1] Max joint positions. 
 * @param [out] p_on_ring [3 x 1] Point on the ring. 
 * @param [in][out] q (N x 1) Vector of IKin solved joint angles corresponding to 
 * the t_dest pose.
*/
int lbr_humanlike_ikin(	const Matrix4d& t_base, 
						const Ref<const MatrixX5d>& dh_params, 
						const Matrix4d& t_tool, 
						const Matrix4d& t_dest,
						const double ring_radius,
						const int ring_checkpoints,
						const int elbow_mode,
						const Ref<const VectorXd>& q_min,
						const Ref<const VectorXd>& q_max,
						Ref<Vector3d >p_on_ring,
						Ref<VectorXd> q)
{
	// Sanity check
	if (ROBOT_SAFETY::InputIntegrityCheck(t_base, dh_params, t_tool, t_dest, q_min, q_max, q, Matrix6Xd().setZero()) > 0) {
		printf("[IKin] Returned with errors. See above.");
		q = VectorXd::Zero(dh_params.rows());
		return -1;
	}
	
	Vector3d P_lShoulder, P_rShoulder, P_elbow, P_wrist, P_elbowCurrent;
	Matrix4d T_lShoulder, T_rShoulder, T_wrist;
	double a1, a2, a3, a4, a5, a6, a7;
	int is_feasible = 1;
	VectorXd inputq = q;
			
	q << M_PI/2, 0, 0, 0, 0, 0, 0;
	//find location of shoulder gimbal
	T_lShoulder = FKinToJoint(t_base, dh_params, q, 1);
	T_rShoulder = FKinToJoint(t_base, dh_params, q, 4);
	P_lShoulder = T_lShoulder.block(0,3,3,1);
	P_rShoulder = T_rShoulder.block(0,3,3,1);
	
	// Algorithm to calculate torso twist and bend.  Twist is informed by 
	// distance outside of shoulder-hand radius (~80 cm). Maximum reach is 
	// 120 cm. Bend is informed by angle from horizontal if object is 
	// outside of the 80 cm radius 
	
	//calculate a1 - torso bend angle - only works for right hand gripper configuration
	a1 = atan2(t_dest(0,3), -t_dest(1,3))/1.5; //with scale back factor of 2
	q(0) = M_PI_2 + a1; //torso starts upright at 90 deg
	
	T_lShoulder = FKinToJoint(t_base, dh_params, q, 1);
	P_lShoulder = T_lShoulder.block(0,3,3,1);
	
	double yDistance = -t_dest(1,3);
	if (ComputeDistance(T_lShoulder, t_dest) > 1.3)
	{
		p_on_ring = Vector3d::Zero();
		return 0;
		//int is_feasible = 0; 
		//TODO scale back t_dest to something reasonable such that robot can attempt
	}
	else if(yDistance <= 1.3 && yDistance >= 0.8)
		a2 = -((yDistance - 0.8)/0.4)* M_PI/3;
	else if(yDistance >= 0.1 && yDistance < 0.5)
		a2 = ((0.5 - yDistance)/0.4) * M_PI/3;
	else if(yDistance >= 0.5 && yDistance < 0.8)
		a2 = 0;
	else
		return 0;
	
	q(1) = a2;

	T_rShoulder = FKinToJoint(t_base, dh_params, q, 4);
	P_rShoulder = T_rShoulder.block(0,3,3,1);
	
	// given a location on the ring (x,y,z) and an axis in kuka world 
	// coordinates, figure out the quarter circle arc of where the elbow gimbal 
	// can be located (not a half arc since the elbow shouldn't be turned 
	// inwards). Then find where that arc intersects the surface of the sphere 
	// representing the range of motion of the elbow gimbal as the shoulder is 
	// moving. There should be at most one solution (I think).  
		
	// T_des is the ring center - Find points that are on the robot-half of the ring
	Vector3d V_ringX = t_dest.block(0,0,3,1);
	Vector3d V_ringUp = ProjectVec(Vector3d::UnitX(), V_ringX).normalized(); //global up vector prejected into the ring plane
	Vector3d P_ring, V_ringTangent; // position on the ring and tangent vector
	Vector3d P_ringCenter = t_dest.block(0,3,3,1);
	Matrix3d R_ringStart = RVec(V_ringX, M_PI/6.0);	
	Matrix3d R_ringEnd = RVec(V_ringX, 5.0*M_PI/6.0);	
	Vector3d P_elbowOptimal(1000.0,0,0);
	Vector3d V_ringTangentOptimal(0,0,0);
	Vector3d P_ringOptimal(0,0,0);
	double modeOneNorm = 1000.0; // large number
	
	is_feasible = 0;
	
	//finds transform of elbow's position from incoming q JointVector if mode = 1
	if( elbow_mode == 1)
		P_elbowCurrent = FKinToJoint(t_base, dh_params, inputq, 6).topRightCorner(3,1);
	
	// Search for position of elbow gimbal using an analytic method	
	double r_forearm = t_tool(2,3);
	double r_upperarm = dh_params(4,1);
	for(int i = 0; i <= ring_checkpoints; ++i)
	{	
		double j = (double)i;
		Matrix3d R_ringSlerp = RSlerp(R_ringStart, R_ringEnd, j/20.0);
		P_ring = (R_ringSlerp * V_ringUp)*ring_radius + P_ringCenter;

		V_ringTangent = (R_ringSlerp * V_ringUp).cross(V_ringX); 
		Vector3d V_rShoulder_ring = (P_rShoulder - P_ring).normalized();  
		Vector3d V_z_world = Vector3d::UnitZ();
		//projection of elbow gimbal location search limits
		Vector3d V_shoulder_projected_ringPlane = ProjectVec(V_rShoulder_ring, V_ringTangent).normalized();
		Vector3d V_z_world_projected_ringPlane = ProjectVec(V_z_world, V_ringTangent).normalized();
		double angle = acos(V_shoulder_projected_ringPlane.dot(V_z_world_projected_ringPlane));

		//V_ref is plane normal 
		Matrix4d T_ringPoint_world, T_world_ringPoint;
		T_ringPoint_world.block(0,0,3,1) = V_z_world_projected_ringPlane.cross(V_ringTangent).normalized();
		T_ringPoint_world.block(0,1,3,1) = V_z_world_projected_ringPlane.normalized();
		T_ringPoint_world.block(0,2,3,1) = V_ringTangent.normalized();
		T_ringPoint_world.block(0,3,3,1) = P_ring;
		T_ringPoint_world.block(3,0,1,4) << 0,0,0,1; 
		T_world_ringPoint = TInv(T_ringPoint_world);
		
		Vector4d P4_rShoulder, P4_soln;
		Vector3d P_rShoulder_ringPoint, P_soln;
		P4_rShoulder << P_rShoulder.x(), P_rShoulder.y(), P_rShoulder.z(), 1;
		P_rShoulder_ringPoint = (T_world_ringPoint * P4_rShoulder).block(0,0,3,1);
		
		double a,b,c,d,x,y,found_angle;
		double X_s = P_rShoulder_ringPoint.x();
		double Y_s = P_rShoulder_ringPoint.y();
		double Z_s = P_rShoulder_ringPoint.z();
		d = (pow(r_forearm,2) - pow(r_upperarm,2) + pow(X_s,2) + pow(Y_s,2) + pow(Z_s,2))/2;
		a = 1 + (pow(X_s,2)/pow(Y_s,2));
		b = -2*d*X_s/pow(Y_s,2);
		c = (pow(d,2)/pow(Y_s,2))- pow(r_forearm,2);
		
		//if solution is feasible
		if(pow(b,2) >= 4*a*c) {
			x = (-b - sqrt(pow(b,2) - 4*a*c))/(2*a);
			y = (d - (X_s * x))/Y_s;
			P_soln << x, y, 0; 
			found_angle = acos(P_soln.normalized().dot(Vector3d::UnitY()));
			
			if( found_angle <= angle) {
				//cout<<"soln1: " <<angle << " " << found_angle <<"\n";
				P4_soln << x, y, 0, 1;
				P_elbow = (T_ringPoint_world * P4_soln).block(0,0,3,1);
			} else {
				//soln 2
				x = (-b + sqrt(pow(b,2) - 4*a*c))/(2*a);
				y = (d - (X_s * x))/Y_s;
				P_soln << x, y, 0; 
				found_angle = acos(P_soln.normalized().dot(Vector3d::UnitY()));
					
				if( found_angle <= angle) {
					//cout<<"soln2: " <<angle << " " << found_angle <<"\n";
					P4_soln << x, y, 0, 1;
					P_elbow = (T_ringPoint_world * P4_soln).block(0,0,3,1);
				}
				else
					P_elbow = Vector3d::Zero();
			}
		} else {
			P_elbow = Vector3d::Zero();
		}
		
		// find optimal elbow position
		if(P_elbow.x() != 0.0 &&  P_elbow.y() != 0.0 && P_elbow.z() != 0.0) {
			if(elbow_mode == 0 && P_elbow.x() <= P_elbowOptimal.x()) {
				P_elbowOptimal = P_elbow;
				P_ringOptimal = P_ring;
				V_ringTangentOptimal = V_ringTangent;
				is_feasible = 1;
			} else if (elbow_mode == 1 && (P_elbow - P_elbowCurrent).norm() <= modeOneNorm) {
				P_elbowOptimal = P_elbow;
				P_ringOptimal = P_ring;
				V_ringTangentOptimal = V_ringTangent;
				modeOneNorm = (P_elbow - P_elbowCurrent).norm();
				is_feasible = 1;
			}
		}
	}
	
	if(is_feasible) {
		P_elbow = P_elbowOptimal;
		P_ring = P_ringOptimal;
		p_on_ring = P_ring;
		V_ringTangent = V_ringTangentOptimal;
	} else {
		p_on_ring = Vector3d::Zero();
		return 0;
	}
	Vector3d V_rShoulder_lShoulder = (P_rShoulder - P_lShoulder).normalized();
	Vector3d V_elbow_rShoulder = (P_elbow - P_rShoulder).normalized();
	Vector3d V_hand_elbow = (P_ring - P_elbow).normalized();
	
	//calculate a3
	{
		Matrix4d T_q2Current = FKinToJoint(t_base, dh_params, q, 2);
		Vector3d V_z2Current = T_q2Current.block(0,2,3,1);
		Vector3d V_z4Desired = V_elbow_rShoulder.cross(V_rShoulder_lShoulder).normalized();
		
		a3 = acos(V_z2Current.dot(V_z4Desired));
		if(V_z2Current.dot(V_z4Desired.cross(V_rShoulder_lShoulder)) < 0)
			a3 = -a3;
			
		q(2) = a3;
	}
	
	//calculate a4
	{
		a4 = acos(V_rShoulder_lShoulder.dot(V_elbow_rShoulder));
		//want to only use positive range of axis for right handed gripper
		a4 = abs(a4);
		q(3) = a4;
	}
	
	//calculate a5
	{
		Matrix4d T_q4Current = FKinToJoint(t_base, dh_params, q, 4);
		Vector3d V_z4Current = T_q4Current.block(0,2,3,1);
		Vector3d V_z6Desired = V_elbow_rShoulder.cross(V_hand_elbow).normalized();
		a5 = acos(V_z4Current.dot(V_z6Desired));
		if(V_z4Current.dot(V_z6Desired.cross(V_elbow_rShoulder)) < 0)
			a5 = -a5;
		q(4) = a5;
	}
	
	//calculate a6
	{
		a6 = acos(V_elbow_rShoulder.dot(V_hand_elbow));
		//want to only use negative range of axis for right handed gripper
		a6 = -a6;
		q(5) = a6;
	}
	
	//calculate a7
	{
		Vector3d V_z5 =  V_hand_elbow.cross(V_elbow_rShoulder).normalized();
		a7 = acos( V_z5.dot(V_ringTangent));
		q(6) = a7 - M_PI;
	}
	
	return is_feasible;
}









