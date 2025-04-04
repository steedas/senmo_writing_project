/** ============================================================================
 *  @file	 	safety.cpp
 *  @author		Matthew Pan (matthew.pan@queensu.ca)
 *  @date	 	Mar 15, 2024
 *  @version 	1.0 
 *  
 *  @brief Functions for sanity checking and various safety precautions. 
 * Additionally a SafetyController class to implement virtual bounds in joint 
 * space to stay within the specified joint limits. 
 *
 *  @section DESCRIPTION
 *      Functions for sanity checking and various safety precautions
 * 
 *  @section CHANGES
 * ===========================================================================*/

#include "kinematics/safety.h"

/**
 *	@brief Function to ensure that the wrist of the robot is within a band
 *  defined by concentric spheres (the working space of the robot) as the end 
 *  effector moves towards transformation matrix t. 
 *	
 * 	Returns (1) if transform is inside the spherical shell; (0) if it was 
 *  was outside the shell, but was able to be scaled back into the shell, or,
 *  (-1) if the transform could not be moved back into the shell ( in the 
 *  specific case that the transform is at the base of the robot). 
 * 
 *  @param [in] t_base Transformation matrix of the base of the robot. 
 *	@param [in][out] t End effector transformation to check. Scales the position
 *  if the position lies beyond the workspace of the robot.
 * 	@param [in] wrist_length Length of the tool + distance of flange to J6
 * 	@param [in] min_radius Minimum radius of the worksapce to check (forms the 
 *  smaller of the concentric spheres centered at the robot base which makes up
 *  the workspace region. 
 *	@param [in] max_radius Maximum radius of the workspace to check (forms the 
 *  larger of the concentric spheres centered at the robot base which makes up 
 *  the workspace region. 
 * */
int ROBOT_SAFETY::IsInsideSphere(	const Matrix4d& t_base, Matrix4d& t, 
					double wrist_length, double min_radius, double max_radius) {
	Matrix4d t_wrist = t;
	Matrix3d r_init = t.block(0,0,3,3);
	
	//subtract tool length from the end effector transform to get location of wrist
	t_wrist.block(0,3,3,1) = t_wrist.block(0,3,3,1) - (r_init.block(0,2,3,1)*wrist_length);
	double d = ComputeDistance(t_base, t_wrist);

	if(d > max_radius)
	{	
		// Scale back in the wrist center to the max. radius
		t_wrist.block(0,3,3,1) = (max_radius/d)*(t_wrist.block(0,3,3,1) - t_base.block(0,3,3,1)) + t_base.block(0,3,3,1);
		// Get the new scaled t
		t.block(0,3,3,1) = t_wrist.block(0,3,3,1) + (r_init.block(0,2,3,1) * wrist_length);
		return 0;
	}
	else if (d >= min_radius)
		return 1;
	else if( d > 0.0 )
	{
		// Scale out to min_radius
		t_wrist.block(0,3,3,1) = (min_radius/d)*(t_wrist.block(0,3,3,1) - t_base.block(0,3,3,1)) + t_base.block(0,3,3,1);
		// Get the new scaled t
		t.block(0,3,3,1) = t_wrist.block(0,3,3,1) + (r_init.block(0,2,3,1) * wrist_length);
		return 0;
	}
	else
		return -1;	// if d == 0, can't do anything! 
}

/**
 * @brief A function to ensure that joint angles are between specified min and 
 * max values. If not, set offending joints to min/max. 
 * 
 * @param [in,out] q Joint angles to check. If any of the joint angles exceed 
 * the min/max values, that angle gets set back to the min/max.
 * @param [in] q_max Maximum joint angles.
 * @param [in] q_min Minimum joint angles.
 * */ 
bool ROBOT_SAFETY::EnsureJointLimits(Ref<VectorXd> q, const Ref<const VectorXd>& q_max, 
				const Ref<const VectorXd>& q_min) {
	bool flag = true;
	//for(int i = 0; i < N_JOINTS; i++)
	for(int i = 0; i < q.rows(); i++) {
		if(q(i,0) < q_min(i,0)) {
			q(i,0) = q_min(i,0);
			flag = false;
		}
		else if(q(i,0) > q_max(i,0)) {
			q(i,0) = q_max(i,0);
			flag = false;
		}
	}		
	return flag;
}

/**
 * @brief A function to check if joint angles are between specified min and 
 * max values. Returns false if joint limits exceeded.
 * 
 * @param [in] q Joint angles to check. 
 * @param [in] q_max Maximum joint angles.
 * @param [in] q_min Minimum joint angles.
 * */ 
bool ROBOT_SAFETY::IsInsideJointLimits(const Ref<const VectorXd>& q, const Ref<const VectorXd>& q_max, 
				const Ref<const VectorXd>& q_min) {
	bool flag = true;
	for(int i = 0; i < q.rows(); i++) {
		if(q(i,0) < q_min(i,0))
			flag = false;
		else if(q(i,0) > q_max(i,0))
			flag = false;
	}
	return flag;
}

/**
 *	@brief Function to detect if "t" is inside the protected space as defined in 
 *  KUKA safety configuration. If inside, it is projected back out.  
 *	
 * 	Returns (1) if transform is outside protected space; (0) if it is inside the
 *  protected space. In the latter case, scaling is performed to the t matrix. 
 * 
 *  @param [in] t_protected	Frame defining the protected space coordinate system.
 *	@param [in] x_dim Length of the space along X-axis.
 * 	@param [in] y_dim Length of the space along Y-axis.
 * 	@param [in] z_dim Length of the space along Z-axis.
 *  @param [in] h_tool Length of the tool.
 *  @param [in] wrist_safety_radius The radius of the sphere centered 
 *  at wrist center.
 *	@param [in,out] t Point to be checked, function returns scaled back point.
 * */
int ROBOT_SAFETY::IsInsideProtectedSpace(	const Matrix4d& t_protected, 
							double x_dim, double y_dim, double z_dim, 
							double h_tool,
							double wrist_safety_radius,
							Matrix4d& t)
{
	
	// DEBUG:
	//t << 0.7733,	0.4082,	0.4852,	0.1000,
		 //0.4836,	0.1152,	-0.8676,-1.3000,
		 //-0.4102,	0.9056,	-0.1083,0.5670,
		 //0,		 0,		 0,		1.0000;
	
	//cout << "t:" << "\n" << t << "\n";
	//cout << "t_protected:" << "\n" << t_protected << "\n";
	
	// Get the wrist center
	Matrix4d t_wrist = t;
	Matrix3d r_init = t.block(0,0,3,3);
	t_wrist.block(0,3,3,1) = t_wrist.block(0,3,3,1) - (r_init.block(0,2,3,1)*h_tool);
		
	Matrix4d t_protected_inv = t_protected.jacobiSvd( ComputeFullU | ComputeFullV ).solve(Matrix4d::Identity());
	Matrix4d t_wrist_ps = t_protected_inv * t_wrist; //t_wrist with respect to protected space
		
	int status = 1;
	
	double epsX = 0.01;
	
	// Check if the point is in the protected space
	double x = t_wrist_ps(0,3);
	double y = t_wrist_ps(1,3);
	double z = t_wrist_ps(2,3);
	
	

	// This is going to have to change
	if((x > -wrist_safety_radius) && (x < x_dim) && 
		(y > -wrist_safety_radius) && (y < y_dim))
	{
		status = 0;
		// Project the point back to Y-Z plane, with a small -X offset so that 
		// we are well outside the protected space
		t_wrist_ps(0,3) = -1 * (wrist_safety_radius + epsX);
		// Get t_wrist back from t_wrist_ps
		t_wrist = t_protected * t_wrist_ps;
	}
	else
		status = 1;
	
	// Get t from t_wrist
	t.block(0,3,3,1) = t_wrist.block(0,3,3,1) + (r_init.block(0,2,3,1) * h_tool);
	
	//printf("x: %f, y: %f, IsInsideProtectedSpace: %d\n", x, y, status);
		
	return status;	
}


/**
 * @brief: Function to check the integrity of dh parameter matrix against 
 * number of joints. Returns the number of errors found.
 * 
 * @param [in] dh [Xx5] DH parameter table
 * @param [in] n_joints Number of joints
*/
int ROBOT_SAFETY::CheckDH(const Ref<const MatrixX5d>& dh, int n_joints) {
	int errors = 0; 

	// Sanity check for DH Params
	if(dh.rows() != n_joints) {
		printf("[kin] Error: The number of joints and rows in the DH table do not match.\n");
		errors++; 
	} 
    if(dh.cols() != 5) {
		printf("[kin] Error: There should be 5 columns in the DH table.\n");
		errors++;
	} 
    return errors;
}

/**
 * @brief: Function to check the integrity of dh parameter matrix against joint 
 * vector. Returns the number of errors found.
 * 
 * @param [in] dh [Xx5] DH parameter table
 * @param [in] q Joint vector
*/
int ROBOT_SAFETY::CheckDH(const Ref<const MatrixX5d>& dh, const Ref<const VectorXd>& q) {
	int errors = 0; 

	// Sanity check for DH Params
	if(dh.rows() != q.rows()) {
		printf("[kin] Error: The number of joints and rows in the DH table do not match.\n");
		errors++; 
	} 
    if(dh.cols() != 5) {
		printf("[kin] Error: There should be 5 columns in the DH table.\n");
		errors++;
	} 
    return errors;
}

/**
 * @brief: Function to check the integrity of homogenous transform. Returns the 
 * number of errors found.
 * 
 * @param [in] t [4x4] Homogeneous transform
*/
int ROBOT_SAFETY::CheckT(const Matrix4d& t) {
    int errors = 0; 

	Matrix3d ident3d;
	Array4d bottom_row = t.bottomRows(1).transpose().array();
	Matrix3d temp_mat = ((t.topLeftCorner(3,3)*t.topLeftCorner(3,3).transpose()) - ident3d.setIdentity());
	double l2norm = temp_mat.norm();
	if(t.rows() != 4 || t.cols() != 4) {
		printf("[kin] Error: t_base should be 4x4.\n");
		errors++;	
	} 
    if( (bottom_row(0) != 0) || (bottom_row(1) != 0) || (bottom_row(2) != 0) 
		|| (bottom_row(3) != 1) ) {
		printf("[kin] Error: The last row of transform is not 0,0,0,1.\n");
		errors++;
	}
    if( l2norm > 1e-12 ) {
		printf("[kin] Error: Rotation part of transform is not normalized.\n");
		errors++;
	}
    return errors;
}

/**
 * @brief: Function to check the integrity of a vector holding joint angles. 
 * Returns the number of errors found.
 * 
 * @param [in] q Joint vector
 * @param [in] q_min Joint vector
 * @param [in] q_max Joint vector
*/
int ROBOT_SAFETY::CheckQ( const Ref<const VectorXd>& q, const Ref<const VectorXd>& q_min, 
            const Ref<const VectorXd>& q_max) {
    int errors = 0;

    if( q_max.rows() != q.rows() || q_min.rows() != q.rows() ) {
		printf("[kin] Error: The number of rows in q_max or q_min do not equal the number of joints.\n");
		errors++;
	}
    return errors;
}

/**
 * @brief: Function to check the integrity of a jacobian. Returns
 * the number of errors found.
 * 
 * @param [in] j 6xN Jacobian
 * @param [in] n_joints Number of joints 
*/
int ROBOT_SAFETY::CheckJ(const Ref<const Matrix6Xd>& j, int n_joints){
    int errors = 0; 
	// Sanity check for Jacobian 
	if( j.cols() != n_joints) {
		printf("[kin] Error: The number of joints and columns in the Jacobian "
			"do not match.\n");
		errors++;
	} 
    if( j.rows() != 6) {
		printf("[kin] Error: The number of rows in the Jacobian does not equal 6.\n");
		errors++;
	}	
    return errors;
}

/**
 * Function to check the integrity of data passed to kin functions. Returns
 * the number of errors found. If you don't want to check an item, just pass
 * in a zero matrix/vector.
*/
int ROBOT_SAFETY::InputIntegrityCheck(    
	const Matrix4d& t_base
	, const Ref<const MatrixX5d>& dh
	, const Matrix4d& t_tool
	, const Matrix4d& t_dest
	, const Ref<const VectorXd>& q
	, const Ref<const VectorXd>& q_min
	, const Ref<const VectorXd>& q_max
	, const Ref<const Matrix6Xd>& j) {
					
	int errors = 0; 
	const int n_joints = dh.rows();

    if(!dh.isZero(0) && !q.isZero(0)) {
		//printf("[kin] Checking DH...");
		errors += CheckDH(dh, q);
	}
    else if(!dh.isZero(0)) {
		//printf("[kin] Checking DH...");
        errors += CheckDH(dh, n_joints);
	}
    if(!t_base.isZero(0)) {
		//printf("[kin] Checking T_base...");
        errors += CheckT(t_base);
	}
    if(!t_tool.isZero(0)) {
		//printf("[kin] Checking T_tool...");
        errors += CheckT(t_tool);
	}
    if(!t_dest.isZero(0)) {
		//printf("[kin] Checking T_dest...");
		errors += CheckT(t_dest);
		// printf("Error in Transformation Matrix");
	}
    if(!q_min.isZero(0) && !q_max.isZero(0) && !q.isZero(0) ) {
		//printf("[kin] Checking joint vectors...");
        errors += CheckQ(q, q_min, q_max);
	}
    if(!j.isZero(0)) {
		//printf("[kin] Checking Jacobian J...");
        errors += CheckJ(j, n_joints);
	}

	return errors; 
}

//*** Safety Controller Class ***

// Default constructor
ROBOT_SAFETY::SafetyController::SafetyController() {
	safety_status_ = SafetyStatus::CONTROLLER_NOT_SET;
}

ROBOT_SAFETY::SafetyController::SafetyController(SafetyParams *params )
	: n_joints_(params->n_joints)
	, tau_runaway_percent_(params->tau_runaway_percent)
	, pos_min_(params->pos_min)
	, pos_max_(params->pos_max)
	, vel_limit_(params->vel_limit)
	, acc_limit_(params->acc_limit)
	, jrk_limit_(params->jrk_limit)
	, tau_limit_(params->tau_limit)
	, inertia_limit_(params->inertia_limit)
	, safety_k_p_(params->safety_k_p)
	, safety_k_d_(params->safety_k_d) {

	safety_status_ = SafetyStatus::CONTROLLER_INITIALIZED;
}
 
ROBOT_SAFETY::SafetyStatus ROBOT_SAFETY::SafetyController::SetControllerParams(SafetyParams params) {
	n_joints_ = params.n_joints;
	tau_runaway_percent_ = params.tau_runaway_percent;
	tau_rate_max_ = params.tau_rate_max;
	pos_min_ = params.pos_min;
	pos_max_ = params.pos_max;
	vel_limit_ = params.vel_limit;
	acc_limit_ = params.acc_limit;
	jrk_limit_ = params.jrk_limit;
	tau_limit_ = params.tau_limit;
	inertia_limit_ = params.inertia_limit;
	safety_k_p_ = params.safety_k_p;
	safety_k_d_ = params.safety_k_d; 

	safety_status_ = SafetyStatus::CONTROLLER_INITIALIZED;
	return safety_status_;
}

ROBOT_SAFETY::SafetyController::~SafetyController() {
	// No dynamic memory allocation done at this time
}

ROBOT_SAFETY::SafetyStatus ROBOT_SAFETY::SafetyController::GetSafetyControlStatus() {
	return safety_status_;
}

ROBOT_SAFETY::SafetyStatus ROBOT_SAFETY::SafetyController::ImposeSafetyBounds(
	const Ref<const VectorXd> pos
	, const Ref<const VectorXd> vel
	, Ref<VectorXd> pos_cmd
	, Ref<VectorXd> tau_cmd) {

	if (safety_status_ == CONTROLLER_NOT_SET) {
		return safety_status_;
	}

	double lambda, c, b, v_ub, v_lb, v_up, v_dn;
	safety_status_ = CMD_WITHIN_LIMITS;

	for(int i = 0; i < n_joints_; i++) {	
		// Check for runaway torque
		if( fabs(tau_cmd(i,0)) > (tau_runaway_percent_ * tau_limit_(i,0)/100.) ) {
			tau_cmd(i,0) = 0.;
			safety_status_ = CMD_KILLED;
			continue;
		}
		
		lambda = safety_k_p_(i,i)/safety_k_d_(i,i);
		b = acc_limit_(i) / ( 2. * lambda * lambda );
		c = 2. * b;
		
		// Compute velocity upper bound
		if(pos(i) <= pos_max_(i) - c) {
			v_ub = sqrt( 2. * acc_limit_(i) * (pos_max_(i) - pos(i) - b));
		}else if(pos(i) > pos_max_ (i) - c) {
			v_ub = lambda * ( pos_max_(i) - pos(i) );
		}
		
		// Compute velocity lower bound
		if(pos(i) >= pos_min_(i) + c) {
			v_lb = -sqrt(2. * acc_limit_(i) * (pos(i) - pos_min_(i) - b));
		}else if(pos(i) < pos_min_(i) + c) {
			v_lb = lambda * ( pos_min_(i) - pos(i) );
		}

		// Impose velocity limits
		v_up = fmin( vel_limit_(i), v_ub);
		v_dn = fmax(-vel_limit_(i), v_lb);
					
		// Compute torque limits
		double tau_max = -safety_k_d_(i,i) * ( vel(i,0) - v_up );
		double tau_min = -safety_k_d_(i,i) * ( vel(i,0) - v_dn );
	
		// Impose computed torque bounds
		if(tau_cmd(i,0) > tau_max) {
			tau_cmd(i,0) = tau_max;
			safety_status_ = CMD_MODIFIED;
		}
			
		if(tau_cmd(i,0) < tau_min) {
			tau_cmd(i,0) = tau_min;
			safety_status_ = CMD_MODIFIED; 
		}
		
		// Rate bounds
		// if(tau_cmd(i,0) > tau_rate_max_ ) {
		// 	tau_cmd(i,0) = tau_rate_max_ ;
		// 	safety_status_ = CMD_MODIFIED; 
		// } else if(tau_cmd(i,0) < tau_rate_max_) {
		// 	tau_cmd(i,0) = tau_rate_max_;
		// 	safety_status_ = CMD_MODIFIED; 
		// }
		
		// Apply max torque bounds
		if(tau_cmd(i,0) > tau_limit_(i,0)) {
			tau_cmd(i,0) = tau_limit_(i,0);
			safety_status_ = CMD_MODIFIED; 
		} else if(tau_cmd(i,0) < -1. * tau_limit_(i,0)) {
			tau_cmd(i,0) = -1*tau_limit_(i,0);
			safety_status_ = CMD_MODIFIED; 
		}
			
		// Apply max joint limits
		if(pos_cmd(i,0) > pos_max_(i,0)) {
			pos_cmd(i,0) = pos_max_(i,0);
			safety_status_ = CMD_MODIFIED; 
		} else if(pos_cmd(i,0) < pos_min_(i,0)) {
			pos_cmd(i,0) = pos_min_(i,0);
			safety_status_ = CMD_MODIFIED; 
		}
	}

	return safety_status_;
}
