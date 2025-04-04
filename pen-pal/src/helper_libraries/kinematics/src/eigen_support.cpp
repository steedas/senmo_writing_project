#include "kinematics/eigen_support.h"

std::string EigenMatrixToString(const Eigen::MatrixXd& mat){
    std::stringstream ss;
    ss << mat;
    return ss.str();
}