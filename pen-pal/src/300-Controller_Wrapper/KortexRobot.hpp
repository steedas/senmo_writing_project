// KortexRobot.hpp

#ifndef KORTEXROBOT_HPP
#define KORTEXROBOT_HPP

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <numeric>
#include <sstream>
#include <iomanip>
#include <unordered_set>
#include <unordered_map>

#include <stdio.h>
#include <csignal>

#include <KDetailedException.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ControlConfigClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <DeviceConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include "kinematics/fkin.h"
#include "kinematics/ikin.h"
#include "kinematics/safety.h"
#include "kinematics/iir_filter.h"
#include "kinematics/eigen_support.h"

#include <google/protobuf/util/json_util.h>

#include "utilities.h"
#include "pid.cpp"

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

#define PORT 10000
#define PORT_REAL_TIME 10001
#define DURATION 100 // Network timeout (seconds)
#define X_MIN 0.26   // Network timeout (seconds)
#define X_MAX 0.6    // Network timeout (seconds)
#define Y_MIN -0.28  // Network timeout (seconds)
#define Y_MAX 0.28   // Network timeout (seconds)

namespace k_api = Kinova::Api;

const auto ACTION_WAITING_TIME = std::chrono::seconds(1);
//float time_duration = DURATION; // Duration of the example (seconds)

class KortexRobot
{
private:
    std::string ip_address;
    std::string username;
    std::string password;
    bool running_demo;

    k_api::TransportClientTcp *transport;
    k_api::RouterClient *router;
    k_api::TransportClientUdp *transport_real_time;
    k_api::RouterClient *router_real_time;

    k_api::Session::CreateSessionInfo create_session_info;

    k_api::SessionManager *session_manager;
    k_api::SessionManager *session_manager_real_time;

    k_api::Base::BaseClient *base;
    k_api::BaseCyclic::BaseCyclicClient *base_cyclic;

    k_api::ActuatorConfig::ActuatorConfigClient *actuator_config;
    k_api::DeviceConfig::DeviceConfigClient *device_config;
    k_api::ControlConfig::ControlConfigClient *control_config;

    std::function<void(k_api::Base::ActionNotification)>
    check_for_end_or_abort(bool &finished);
    void printException(k_api::KDetailedException &ex);

    int64_t GetTickUs();

public:
    // form of theta, d, a, alpha, prismatic
    KortexRobot(const std::string &ip_address, const std::string &username, const std::string &password, bool inputdemo = false);
    struct Node
    {
        std::vector<float> data; 
        Node *next;              
        Node *prev;              

       
        Node(const std::vector<float> &value)
            : data(value), next(nullptr), prev(nullptr) {}
    };
    typedef Node *traj_link_list;

    struct TrajectoryData {
        int id;
        std::vector<std::vector<float>> current_vector;
    };


    void go_to_point(const std::string &actionName);
    void go_to_point_jointangle(std::vector<float>);
    void connect();
    vector<vector<float>> derivative_of_vector(vector<vector<float>> vec_to_differentiate, float refresh_rate);
    vector<vector<float>> numerical_inverse_kinematics_trajectory(vector<vector<float>> cartesian_position_and_euler_angles, float refresh_rate, string filename);
    vector<Vector6d> solve_for_xdot(vector<vector<float>> cartesian_position_and_euler_angles, float refresh_rate);
    void disconnect();
    void writeVectorToCSV(const std::vector<std::vector<float>> &data, const std::string &filename);
    ~KortexRobot();
    void set_actuator_control_mode(int mode_control, int actuator_indx = -1);
    std::vector<TrajectoryData> pre_compute_q_trajectories(const std::string& input, float scale_factor, float kTheta_x, float kTheta_y, float kTheta_z, float originX, float originY, float originZ);
    void writing_mode();
    bool at_target_position(std::vector<float> measured, std::vector<float> actual, float eps);
    vector<vector<float>> move_cartesian(std::vector<std::vector<float>> waypointsDefinition, bool repeat = false,
                                         float kTheta_x = 180.0f, float kTheta_y = 0.0f, float kTheta_z = 90.0f, bool joints_provided = false);
    vector<vector<float>> return_valid_trajectory(std::vector<std::vector<float>> trajectory, float refresh_rate);
    int gcd(int a, int b);
    void findMultiplier(double x, int &num, int &den);
    vector<vector<float>> adjust_trajectory_origin(vector<vector<float>> trajectory, float start_x, float start_y, float start_z);
    vector<vector<float>> linear_interp_trajectory(vector<vector<float>> trajectory, float scale);
    vector<vector<float>> rotate_cartesian_points(vector<vector<float>> cartesian_traj);
    vector<vector<float>> forward_kinematics_trajectory(vector<vector<float>> q_trajactory);
    vector<vector<float>> move_cartesian_precompute(std::vector<std::vector<float>> waypointsDefinition, bool repeat = false,
                                                    float kTheta_x = 180.0f, float kTheta_y = 0.0f, float kTheta_z = 90.0f, bool joints_provided = false);
    vector<vector<float>> absolute_value_kinova_syntax(vector<vector<float>> trajectory);
    void insert_next(traj_link_list ptr, traj_link_list element_to_add);
    void insert_prev(traj_link_list ptr, traj_link_list element_to_add);
    void run_trajectory_control(const std::string& input, vector<TrajectoryData>);
    std::vector<std::vector<float>> convert_points_to_angles(std::vector<vector<float>> target_points);

    std::vector<std::vector<float>> read_csv(const std::string &filename, int scale = 1000);
    std::vector<std::vector<float>> convert_csv_to_cart_wp(std::vector<std::vector<float>> csv_points,
                                                           float kTheta_x, float kTheta_y,
                                                           float kTheta_z);
    std::vector<std::vector<float>> generate_constant_rotation_vector(float kTheta_x, float kTheta_y, float kTheta_z, int size);
    void print_current_elements(traj_link_list head);
    bool first_point_larger(traj_link_list element1, traj_link_list element3, float refresh_);
    bool velocity_check(traj_link_list element1, traj_link_list element2, float refresh_rate);
    void recursive_interpolation(traj_link_list head, traj_link_list tail);
    bool acceleration_check(traj_link_list element1, traj_link_list element2, float refresh_rate);
    bool move_cartesian_PControl(std::vector<std::vector<float>> waypointsDefinition, bool repeat, float scale_factor, float kTheta_x,
                                                           float kTheta_y, float kTheta_z, float originX, float originY, float originZ, bool joints_provided);
    void insert_interpolated_point_after(traj_link_list ptr);
    void calculate_bias(std::vector<float> first_waypoint);
    void output_arm_limits_and_mode();
    bool execute_jointspace_trajectory(std::vector<std::vector<float>> target_joint_angles_IK);
    std::vector<std::vector<float>> insert_pre_trajectory_elevation_change(std::vector<std::vector<float>> xyz_traj, float elevation_delta, float elevation_time, float delay_time_after_change);
    std::vector<std::vector<float>> insert_post_trajectory_elevation_change(std::vector<std::vector<float>> xyz_traj, int index, float elevation_delta, float elevation_time, float delay_time_after_change);
    traj_link_list convert_arr_to_linked_list(std::vector<std::vector<float>> &trajectory);
    std::vector<std::vector<float>> convert_linked_list_to_arr(KortexRobot::traj_link_list trajectory);
    const vector<float> actuator_pos_tolerance = {0.085,
                                                  0.105,
                                                  0.105,
                                                  0.105,
                                                  0.105,
                                                  0.105};
    const vector<float> max_joint_speed = {1.39, 1.39, 1.39, 1.22, 1.22, 1.22};
    const vector<float> max_joint_acceleration = {5.2, 5.2, 5.2, 10.0, 10.0, 10.0};
    const vector<int> actuator_control_types = {0, 0, 0, 0, 0, 0};
    const vector<float> command_max = {100.0, 30, 30.0, 15.0, 30, 25.0};
    const vector<float> command_min = {-100.0, -30.0, -30.0, -15.0, -30, -25.0};
    const vector<float> step_change_limit = {20.0, 30, 2, 20.0, 20.0, 20.0};
    std::vector<float> motor_command = {10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f}; // Vector of current_velocities/torques to use in calculation for next command

    void init_pids();
    void get_gain_values(const std::string &filename);

    vector<float> altered_origin;
    vector<float> bais_vector;

    int actuator_count = 6;
    vector<Pid_Loop> pids;

    const vector<float> surface_cords = {0.455, 0, 0.115};
    void find_paper();
    // Plotting and performance functions
    vector<float> measure_joints(k_api::BaseCyclic::Feedback base_feedback, int64_t start_time);
    int start_plot();
    void plot(vector<vector<float>> expected_data, vector<vector<float>> measured_data);
    int create_plot_file(string file_name, vector<vector<float>> data);
    vector<float> rms_error(vector<vector<float>> expected, vector<vector<float>> measured);
    vector<vector<float>> generate_log(const std::string &filename, vector<vector<float>> data);
    void output_joint_values_to_csv(std::vector<std::vector<float>> joint_angles, const std::string &filename);
    void execute_demo();
    void set_origin_point();
    void populatePreComputedJointTrajectory(k_api::Base::PreComputedJointTrajectory *traj,
                                            std::vector<std::vector<float>> trajectoryData);

    FILE *gnu_plot;

protected:
    // data
};

#endif // KORTEXROBOT_HPP
