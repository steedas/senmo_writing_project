/** ============================================================================
 *  @file    eigen_support.h
 *  @author  Matthew Pan (matthew.pan@queensu.ca)
 *  @date    March 25, 2024
 *  @version 1.0
 *
 *  @brief Header file to implement Eigen support functions
 *
 * ===========================================================================*/

#ifndef KINEMATICS__EIGEN_SUPPORT_H_
#define KINEMATICS__EIGEN_SUPPORT_H_

#include <Eigen/Core>
#include <math.h>
#include <kinematics/kin.h>

typedef Eigen::Matrix<double, Eigen::Dynamic, 5> MatrixX5d;
typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6Xd;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/**
 * @brief A templated function converts a std::vector to an Eigen matrix.
 * Returns a zero matrix if size of vector does not equal rows*cols
 *
 * @param [in] vec Vector
 * @param [in] rows Number of rows in output matrix
 * @param [in] cols Number of columns in output matrix
 * @returns Eigen matrix

*/
template <typename Scalar>
inline static Eigen::Matrix<Scalar, -1, -1> ToEigenMatrix(const std::vector<Scalar> vec, int rows, int cols)
{
    Eigen::Matrix<Scalar, -1, -1> e_mat(rows, cols);
    if (vec.size() != rows * cols)
        e_mat.setZero();

    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
            e_mat(i, j) = vec[(i * cols) + j];
    }
    return e_mat;
}

inline static Eigen::Matrix3d NormalizeMatrix(Eigen::Matrix3d &matrix_to_normalize)
{
    Eigen::Matrix3d normalized_matrix = matrix_to_normalize;
    for (int i = 0; i < normalized_matrix.cols(); ++i)
    {
        double norm = normalized_matrix.col(i).norm(); // Calculate the norm (magnitude) of the i-th column
        if (norm != 0)
        {
            normalized_matrix.col(i) /= norm; // Normalize the column by dividing each element by the norm
        }
    }

    return normalized_matrix;
}

/**
 * @brief A templated function converts euler angles and cartesian coordinates (Kinova syntax) to an Eigen matrix.
 *
 * @param [in] cartesian_coordinates_ cartesian coordinates in form [x,y,z]
 * @param [in] rotation euler angles in the form [theta_x, theta_y, theta_z];
 * @param [in] rows Number of rows in output matrix
 * @param [in] cols Number of columns in output matrix
 * @returns Eigen matrix

*/
inline static Eigen::Matrix4d ToEigenMatrixEuler(const std::vector<float> cartesian_coordinates_and_rotation, int rows = 4, int cols = 4)
{
    Eigen::Matrix4d e_mat;
    Eigen::Matrix3d rot_z_ = RotationAboutZ(cartesian_coordinates_and_rotation[5] * M_PI / 180.0);
    Eigen::Matrix3d rot_y_ = RotationAboutY(cartesian_coordinates_and_rotation[4] * M_PI / 180.0);
    Eigen::Matrix3d rot_x_ = RotationAboutX(cartesian_coordinates_and_rotation[3] * M_PI / 180.0);
    Eigen::Matrix3d extrinsic_rotation = rot_z_ * rot_y_ * rot_x_;
    extrinsic_rotation = NormalizeMatrix(extrinsic_rotation);
    e_mat << extrinsic_rotation(0, 0), extrinsic_rotation(0, 1), extrinsic_rotation(0, 2), cartesian_coordinates_and_rotation[0],
        extrinsic_rotation(1, 0), extrinsic_rotation(1, 1), extrinsic_rotation(1, 2), cartesian_coordinates_and_rotation[1],
        extrinsic_rotation(2, 0), extrinsic_rotation(2, 1), extrinsic_rotation(2, 2), cartesian_coordinates_and_rotation[2],
        0, 0, 0, 1;

    //printf("Matrix4d using Eigen::Matrix template:\n");

    return e_mat;
}

template <typename Scalar, std::size_t SIZE>
inline static Eigen::Matrix<Scalar, -1, -1> ToEigenMatrix(const std::array<Scalar, SIZE> arr, int rows, int cols)
{
    Eigen::Matrix<Scalar, -1, -1> e_mat(rows, cols);
    if (arr.size() != rows * cols)
        e_mat.setZero();

    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
            e_mat(i, j) = arr[(i * cols) + j];
    }
    return e_mat;
}

template <typename Scalar, typename Matrix>
inline static std::vector<std::vector<Scalar>> fromEigenMatrix(const Matrix &M)
{
    std::vector<std::vector<Scalar>> m;
    m.resize(M.rows(), std::vector<Scalar>(M.cols(), 0));
    for (size_t i = 0; i < m.size(); i++)
        for (size_t j = 0; j < m.front().size(); j++)
            m[i][j] = M(i, j);
    return m;
}

std::string EigenMatrixToString(const Eigen::MatrixXd &mat);

#endif // KINEMATICS__EIGEN_SUPPORT_H_