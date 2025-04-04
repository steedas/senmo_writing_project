/** ============================================================================
 *  @file	 	test.cpp
 *  @author		Matthew Pan (matthew.pan@queensu.ca)
 *  @date	 	Mar 18, 2024
 *  @version 	1.2 
 *  
 *  @brief Short program to test the functionality of the InputIntegrityCheck 
 * fcn in safety.cpp. Wanted to ensure that the function can correctly 
 * distinguish true input paramters from dummy paramters (those matricies set 
 * to zero). 
 *
 *  @section DESCRIPTION
 *
 *  @section CHANGES
 * ===========================================================================*/


#include "kinematics/safety.h"
#include <Eigen/Core>
#include <iostream>

using namespace std;
using namespace Eigen;

int main() {
    Matrix4d fake_t_base = Matrix4d().setZero();
    Matrix4d fake_t_tool = Matrix4d().setRandom();
    Matrix4d fake_t_dest = Matrix4d().setZero();
    int x = InputIntegrityCheck(fake_t_base, MatrixX5d(), fake_t_tool, fake_t_dest, VectorXd(), VectorXd(), VectorXd(), Matrix6Xd());

    cout << x << endl;

    cout << fake_t_tool;

    return 0;

}