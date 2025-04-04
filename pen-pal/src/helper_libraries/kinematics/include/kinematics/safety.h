/** ============================================================================
 *  @file	 	safety.h
 *  @author		Matthew Pan (matthew.pan@queensu.ca)
 *  @date	 	Mar 15, 2024
 *  @version 	1.2 
 *  
 *  @brief Header containing fcn prototypes of some helpful safety check fcns
 *
 *  @section DESCRIPTION
 *      Header containing fcn prototypes of some helpful safety check fcns
 * 
 *  @section CHANGES
 * ===========================================================================*/
#ifndef KINEMATICS__SAFETY_H_
#define KINEMATICS__SAFETY_H_

#include <Eigen/Core>
#include "kin.h"
#include "eigen_support.h"
#include <cmath>

using namespace std;
using namespace Eigen;

namespace ROBOT_SAFETY {
	
int IsInsideSphere(	const Matrix4d& t_base, Matrix4d& t, 
					double wrist_length, double min_radius, double max_radius); 

bool EnsureJointLimits( Ref<VectorXd> q, const Ref<const VectorXd>& q_max, 
				        const Ref<const VectorXd>& q_min);

bool IsInsideJointLimits(const Ref<const VectorXd>& q, const Ref<const VectorXd>& q_max, 
				        const Ref<const VectorXd>& q_min);

int IsInsideProtectedSpace(	const Matrix4d& t_protected, 
							double x_dim, double y_dim, double z_dim, 
							double h_tool,
							double wrist_safety_radius,
							Matrix4d& t);

int CheckDH(const Ref<const MatrixX5d>& dh, int n_joints);

int CheckDH(const Ref<const MatrixX5d>& dh, const Ref<const VectorXd>& q);

int CheckT( const Matrix4d& t);

int CheckQ( const Ref<const VectorXd>& q, const Ref<const VectorXd>& q_min, 
            const Ref<const VectorXd>& q_max);

int CheckJ( const Ref<const Matrix6Xd>& j, int n_joints); 

int InputIntegrityCheck(    const Matrix4d& t_base, 
                            const Ref<const MatrixX5d>& dh, 
	                        const Matrix4d& t_tool, 
                            const Matrix4d& t_dest,
                            const Ref<const VectorXd>& q,
							const Ref<const VectorXd>& q_min,
							const Ref<const VectorXd>& q_max,  
                            const Ref<const Matrix6Xd>& j);

typedef struct safetyParams{ 
	int n_joints;
	double tau_runaway_percent;
	double tau_rate_max;
	VectorXd pos_min;
	VectorXd pos_max;
	VectorXd vel_limit;
	VectorXd acc_limit;
	VectorXd jrk_limit;
	VectorXd tau_limit;
	VectorXd inertia_limit;
	MatrixXd safety_k_p;
	MatrixXd safety_k_d;
} SafetyParams;

// Safety controller status.  
// CMD_WITHIN_LIMITS: no modification to the commanded torque
// CMD_MODIFIED: some modification to the commanded torque
// CMD_KILLED killed commanded torque completely
// CONTROLLER_NOT_SET: safety controller not set
typedef enum {CONTROLLER_NOT_SET, CONTROLLER_INITIALIZED, CMD_WITHIN_LIMITS, CMD_MODIFIED, CMD_KILLED} SafetyStatus;

class SafetyController {	
	private:
		int n_joints_;

		SafetyStatus safety_status_;			
		double tau_runaway_percent_;
		double tau_rate_max_;
		VectorXd pos_min_;
		VectorXd pos_max_;
		VectorXd vel_limit_;
		VectorXd acc_limit_;
		VectorXd jrk_limit_;
		VectorXd tau_limit_;
		VectorXd inertia_limit_;
		MatrixXd safety_k_p_;
		MatrixXd safety_k_d_;

	public:
		/**
		* \brief Constructor.
		*/
		SafetyController();
		SafetyController(SafetyParams *params);

		/**
		* \brief Destructor.
		*/		
		~SafetyController();
		
		/**
		* \brief Sets controller parameters.
		*/
		SafetyStatus SetControllerParams(SafetyParams params);
		/**
		* \brief Gets the current status of safety controller.
		*/		
		SafetyStatus GetSafetyControlStatus();
		
		/**
		* \brief Function to implement the virtual bounds based on position
		* @param pos: current joint position
		* @param vel: current joint velocity
		* @param pos_cmd: reference to the current commanded position that will be shaped by safety controller before sending to the robot
		* @param tau_cmd: reference to the current commanded torque that will be shaped by safety controller before sending to the robot
		*/		
		SafetyStatus ImposeSafetyBounds(const Ref<const VectorXd> pos, const Ref<const VectorXd> vel, Ref<VectorXd> pos_cmd, Ref<VectorXd> tau_cmd);


};

}

#endif