// TODO: Check if we need to modify/delete the license below since the structure
//       of our code will likely resemble the examples in their repo and video

/*
 * KINOVA (R) KORTEX (TM)
 *
 * Copyright (c) 2019 Kinova inc. All rights reserved.
 *
 * This software may be modified and distributed
 * under the terms of the BSD 3-Clause license.
 *
 * Refer to the LICENSE file for details.
 *
 */
#include "KortexRobot.hpp"
#include <cstdint>
#include <sstream>


namespace k_api = Kinova::Api;
float time_duration = DURATION;

KortexRobot::KortexRobot(const std::string &ip_address, const std::string &username, const std::string &password, bool demo)
{
    KortexRobot::ip_address = ip_address;
    KortexRobot::username = username;
    KortexRobot::password = password;
    KortexRobot::running_demo = demo;
    //KortexRobot::connect();
    init_pids();
}

std::vector<KortexRobot::TrajectoryData> KortexRobot::pre_compute_q_trajectories(const std::string& input, float scale_factor, float kTheta_x, float kTheta_y, float kTheta_z, float originX, float originY, float originZ){
    std::stringstream ss(input);
    std::unordered_set<int> unique_numbers;
    std::unordered_map<int, TrajectoryData> trajectory_map;
    std::vector<TrajectoryData> result;
    std::string token;
    
    // Parse input string
    while (std::getline(ss, token, ',')) {
        int num = std::stoi(token);
        unique_numbers.insert(num);
    }

    // Process each unique number
    for (int num : unique_numbers) {
        std::string filename = "../../Coordinates/" + std::to_string(num) + ".csv";
        TrajectoryData traj_data;
        traj_data.id = num;
        std::vector<std::vector<float>> target_waypoints = read_csv(filename, 1000);
        target_waypoints = convert_csv_to_cart_wp(target_waypoints, kTheta_x, kTheta_y, kTheta_z);
        target_waypoints = rotate_cartesian_points(target_waypoints);
        target_waypoints = adjust_trajectory_origin(target_waypoints, originX, originY, originZ);
        target_waypoints = insert_pre_trajectory_elevation_change(target_waypoints, 0.1, 2.0 / scale_factor, 3.0 / scale_factor);
        target_waypoints = insert_post_trajectory_elevation_change(target_waypoints, target_waypoints.size() - 1, 0.1, 2.0 / scale_factor, 0.5 / scale_factor);
        std::vector<vector<float>> target_joint_angles_IK_unscaled = numerical_inverse_kinematics_trajectory(target_waypoints, 0.001, filename);
        std::vector<vector<float>> target_joint_angles_IK_scaled = linear_interp_trajectory(target_joint_angles_IK_unscaled, scale_factor);
        std::vector<vector<float>> target_joint_angles_IK_velocity_bounded = return_valid_trajectory(target_joint_angles_IK_scaled, 0.001);
        std::vector<std::vector<float>> target_joint_angles_IK = absolute_value_kinova_syntax(target_joint_angles_IK_velocity_bounded);
        traj_data.current_vector = target_joint_angles_IK;
        trajectory_map[num] = traj_data;
    }

    // Store in result vector
    for (const auto& pair : trajectory_map) {
        result.push_back(pair.second);
    }
    
    return result;
}

void KortexRobot::run_trajectory_control(const std::string& input, vector<KortexRobot::TrajectoryData> trajectoryelements){
    std::stringstream ss(input);
    std::string token;
    while (std::getline(ss, token, ',')) {
        int num = std::stoi(token);
        int index = 0;
        while(trajectoryelements[index].id != num){
            index++;
        }
        go_to_point_jointangle(trajectoryelements[index].current_vector[0]);
        std::vector<std::vector<float>> copy(trajectoryelements[index].current_vector.begin(), trajectoryelements[index].current_vector.begin() + 2000);
        execute_jointspace_trajectory(copy);
        cout << "\nPress SPACEBAR and ENTER to continue";
        while (true) {
            char ch = std::cin.get(); // Waits for user input
            if (ch == ' ') {  // Check if spacebar was pressed
                break;
            }
        }
        std::vector<std::vector<float>> rest(trajectoryelements[index].current_vector.begin() + 2000, trajectoryelements[index].current_vector.end());
        execute_jointspace_trajectory(rest);
        cout << "\nPress SPACEBAR and ENTER to continue";
        while (true) {
            char ch = std::cin.get(); // Waits for user input
            if (ch == ' ') {  // Check if spacebar was pressed
                break;
            }
        }
    }
}


std::function<void(k_api::Base::ActionNotification)>
create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent> &finish_promise)
{
    return [&finish_promise](k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch (action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}

void KortexRobot::plot(vector<vector<float>> expected_data, vector<vector<float>> measured_data)
{
    start_plot();

    string expected_file = "tmp_expected.txt";
    string measured_file = "tmp_measured.txt";
    create_plot_file(expected_file, expected_data);
    create_plot_file(measured_file, measured_data);

    string cmd = "plot '" + expected_file + "' with lines, \\\n" \ 
  "'" + measured_file +
                 "' with line \n";
    fprintf(gnu_plot, cmd.c_str());
    fflush(gnu_plot);
    std::cout << "Press Enter to continue...";
    std::cin.get();

    // Clean up
    std::remove(expected_file.c_str());
    std::remove(measured_file.c_str());
}

vector<vector<float>> KortexRobot::derivative_of_vector(vector<vector<float>> vec_to_differentiate, float refresh_rate)
{
    vector<vector<float>> vec_differentiated(vec_to_differentiate.size(), std::vector<float>(vec_to_differentiate[0].size(), 0.0f));
    if (vec_to_differentiate.size() <= 1)
    {
        cout << "ERROR! VEC_TO_DIFFERENTIATE IS TOO SMALL AND CANNOT BE DIFFERENTIATED. RETURNED ORIGINAL VECTOR!" << endl;
        return vec_to_differentiate;
    }
    for (int i = 0; i < vec_to_differentiate.size(); i++)
    {
        if (i != vec_to_differentiate.size() - 1)
        {
            for (int j = 0; j < vec_to_differentiate[i].size(); j++)
            {
                vec_differentiated[i][j] = (vec_to_differentiate[i + 1][j] - vec_to_differentiate[i][j]) / refresh_rate;
            }
        }
        else
        {
            for (int j = 0; j < vec_to_differentiate[i].size(); j++)
            {
                vec_differentiated[i][j] = 0;
            }
        }
    }
    return vec_differentiated;
}

vector<Vector6d> KortexRobot::solve_for_xdot(vector<vector<float>> cartesian_positions_and_euler_angles, float refresh_rate)
{
    vector<Vector6d> xdot(cartesian_positions_and_euler_angles.size(), Vector6d::Zero());
    vector<vector<float>> cartesianeuler_derivative = derivative_of_vector(cartesian_positions_and_euler_angles, refresh_rate);
    for (int i = 0; i < cartesian_positions_and_euler_angles.size(); i++)
    {
        Vector6d xdot_at_index;
        xdot_at_index[0] = cartesianeuler_derivative[i][0];
        xdot_at_index[1] = cartesianeuler_derivative[i][1];
        xdot_at_index[2] = cartesianeuler_derivative[i][2];
        xdot_at_index[3] = cartesianeuler_derivative[i][3];
        xdot_at_index[4] = cartesianeuler_derivative[i][4];
        xdot_at_index[5] = cartesianeuler_derivative[i][5];
        xdot[i] = xdot_at_index;
    }
    return xdot;
}

vector<vector<float>> KortexRobot::forward_kinematics_trajectory(vector<vector<float>> q_trajectory)
{
    vector<vector<float>> xyz_cartesian(q_trajectory.size(), std::vector<float>(3, 0.0f));
    Matrix4d t_base_;
    t_base_ << 1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;
    std::vector<double> DHParams = {0, -0.28481, 0, M_PI / 2, 0,
                                    -M_PI / 2, -0.00538, 0.410, M_PI, 0,
                                    -M_PI / 2, -0.00638, 0, M_PI / 2, 0,
                                    M_PI, -0.31436, 0, M_PI / 2, 0,
                                    M_PI, 0, 0, M_PI / 2, 0,
                                    M_PI, -0.16746, 0, M_PI, 0};
    const MatrixX5d DH_params_matrix = ToEigenMatrix(DHParams, 6, 5);
    Matrix4d t_tool_;
    t_tool_.setIdentity();
    VectorXd q_current(6);
    Matrix<double, 6, Dynamic> j(6, 6);
    j = Matrix<double, 6, Dynamic>::Zero(6, 6);
    for (int i = 0; i < xyz_cartesian.size(); i++)
    {
        q_current << q_trajectory[i][0], q_trajectory[i][1], q_trajectory[i][2], q_trajectory[i][3], q_trajectory[i][4], q_trajectory[i][5];
        Matrix4d t_current_step = FKin(t_base_, DH_params_matrix, t_tool_, q_current, j);
        printf("T_Base:\n %s\n", EigenMatrixToString(t_current_step).c_str());
        xyz_cartesian[i][0] = t_current_step(0, 3);
        xyz_cartesian[i][1] = t_current_step(1, 3);
        xyz_cartesian[i][2] = t_current_step(2, 3);
    }
    return xyz_cartesian;
}

vector<vector<float>> KortexRobot::numerical_inverse_kinematics_trajectory(vector<vector<float>> cartesian_position_and_euler_angles, float refresh_rate, string filename)
{
    float num_joints = 6;
    vector<vector<float>> q_calculated(cartesian_position_and_euler_angles.size(), std::vector<float>(num_joints, 0.0f));
    Matrix4d t_base_;
    t_base_ << 1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;
    // t_base_.setIdentity();
    //  cout << "T_Base" << EigenMatrixToString(t_base_) << endl;
    std::vector<double> DHParams = {0, -0.28481, 0, M_PI / 2, 0,
                                    -M_PI / 2, -0.00538, 0.410, M_PI, 0,
                                    -M_PI / 2, -0.00638, 0, M_PI / 2, 0,
                                    M_PI, -0.31436, 0, M_PI / 2, 0,
                                    M_PI, 0, 0, M_PI / 2, 0,
                                    M_PI, -0.16746, 0, M_PI, 0};
    const MatrixX5d DH_params_matrix = ToEigenMatrix(DHParams, 6, 5);
    // cout << "DH_PARAMS" << EigenMatrixToString(DHParamsMatrix) << endl;
    Matrix4d t_tool;
    t_tool.setIdentity();
    // cout << "T_TOOL: " << t_tool << endl;
    VectorXd q_min(6);
    q_min << -360, -128.9, -147.8, -360, -120.3, -360;
    q_min = q_min * (M_PI / 180.0);
    // cout << "Q_MIN " << EigenMatrixToString(q_min) << endl;
    VectorXd q_max(6);
    q_max << 360, 128.9, 147.8, 360, 120.3, 360;
    q_max = q_max * (M_PI / 180.0);
    vector<Vector6d> xdot = solve_for_xdot(cartesian_position_and_euler_angles, refresh_rate);
    // cout << "Q_MAX " << EigenMatrixToString(q_max) << endl;
    Matrix6d w_inv;
    w_inv.setIdentity();
    Matrix6d eps;
    VectorXd q_dot_max(6);
    q_dot_max << 1.39, 1.39, 1.39, 1.22, 1.22, 1.22;
    for (int i = 0; i < cartesian_position_and_euler_angles.size(); i++)
    {
        cout << i << endl;
        VectorXd q_last(6);
        q_last.setZero();
        VectorXd q_current(6);
        q_current.setZero();
        VectorXd qdot_current(6);
        qdot_current.setZero();
        // since this isn't running real time not necessary to optimize this
        const Matrix4d t_dest = ToEigenMatrixEuler(cartesian_position_and_euler_angles[i]);
        int ikin_status;
        if (i == 0)
        {
            eps << 0.000001, 0, 0, 0, 0, 0,
                0, 0.000001, 0, 0, 0, 0,
                0, 0, 0.000001, 0, 0, 0,
                0, 0, 0, 0.000001, 0, 0,
                0, 0, 0, 0, 0.000001, 0,
                0, 0, 0, 0, 0, 0.000001;
            q_last << 0.003985, 1.39738, -2.24637, 0.00455031, 2.07295, -3.1394;
            ikin_status = IKin(t_base_, DH_params_matrix, t_tool, t_dest, xdot[i], q_last, 6, 10, w_inv, eps, q_min, q_max, q_dot_max, q_current, qdot_current);
            // cout << "Ikin Status: " << ikin_status;
            for (int j = 0; j < num_joints; j++)
            {
                q_calculated[i][j] = q_current[j];
                // cout << q_current[j] << " ";
            }
        }
        else
        {
            eps << 0.02, 0, 0, 0, 0, 0,
                0, 0.02, 0, 0, 0, 0,
                0, 0, 0.02, 0, 0, 0,
                0, 0, 0, 0.02, 0, 0,
                0, 0, 0, 0, 0.02, 0,
                0, 0, 0, 0, 0, 0.02;
            q_last << q_calculated[i - 1][0], q_calculated[i - 1][1], q_calculated[i - 1][2], q_calculated[i - 1][3], q_calculated[i - 1][4], q_calculated[i - 1][5];
            ikin_status = IKin(t_base_, DH_params_matrix, t_tool, t_dest, xdot[i], q_last, 6, 2, w_inv, eps, q_min, q_max, q_dot_max, q_current, qdot_current);
            for (int j = 0; j < num_joints; j++)
            {
                q_calculated[i][j] = q_current[j];
            }
        }
        if (ikin_status <= 0)
        {
            cout << "Inverse Kinematics had a failure point (either Q OOB or no convergence) at " << i << "index of passed trajectory, terminating program" << endl;
            cout << "This is file: " << filename << endl;
            abort();
        }
    }
    return q_calculated;
}

vector<float> KortexRobot::rms_error(vector<vector<float>> expected_data, vector<vector<float>> measured_data)
{

    vector<float> rms;
    float spatial_error_sum = 0;
    float velocity_error_sum = 0;
    bool first_pass = true;
    float velocity_sum_measured = 0;
    float velocity_sum_expected = 0;
    // measured_data.size()-1 because last data point seems to have crazy high error
    for (int i = 0; i < measured_data.size() - 1; i++)
    {
        if (!first_pass)
        {

            float time_diff_expected = (expected_data[i][0] - expected_data[i - 1][0]) * 1000; // *1000 to undo error in read_csv scale
            float x_diff_expected = (expected_data[i][1] - expected_data[i - 1][1]);
            float y_diff_expected = (expected_data[i][2] - expected_data[i - 1][2]);
            float velocity_expected = (sqrt(pow(y_diff_expected, 2) + pow(x_diff_expected, 2)) / time_diff_expected);

            float time_diff_measured = (measured_data[i][0] - measured_data[i - 1][0]);
            float x_diff_measured = (measured_data[i][1] - measured_data[i - 1][1]);
            float y_diff_measured = (measured_data[i][2] - measured_data[i - 1][2]);
            float velocity_measured = (sqrt(pow(y_diff_measured, 2) + pow(x_diff_measured, 2)) / time_diff_measured);

            velocity_sum_measured += pow(velocity_measured, 2);
            velocity_sum_expected += pow(velocity_expected, 2);

            // cout<<"EXPECTED X DIFF: " << setprecision(5) << x_diff_expected << " EXPECTED Y DIFF: " << setprecision(5) << y_diff_expected << " MEASURED X DIFF: " << setprecision(5) << x_diff_measured <<    " MEASURED Y DIFF: " << setprecision(5) << y_diff_measured << endl;

            //   cout << "Expected\t" << velocity_expected << "Measured:\t" <<velocity_measured << endl;
            float velocity_error = 0;
            if (velocity_expected != 0 && velocity_measured != 0)
                velocity_error = pow((1 - (velocity_measured / velocity_expected)), 2);
            velocity_error_sum += velocity_error;
        }
        else
        {
            first_pass = false;
        }

        // *1000 for m->mm conversion
        float x_error = pow((expected_data[i][1] - measured_data[i][1]) * 1000, 2);
        float y_error = pow((expected_data[i][2] - measured_data[i][2]) * 1000, 2);

        float line_error = sqrt(x_error + y_error);

        /*
        cout << "x maesured :"<< measured_data[i][1] << endl;
        cout << "x expected:"<< expected_data[i][1] << endl;
        cout << "y maesured :"<< measured_data[i][2] << endl;
        cout << "y expected:"<< expected_data[i][2] << endl;
        */
        // cout << "line error:"<< expected_data[i][2] << endl;
        // cout << "x| measured:" << measured_data[i][1] << " expected:"<< expected_data[i][1] << " error: "<< x_error << endl;

        float spatial_error_sqr = pow(line_error, 2);
        // cout << "error sum: " << error_sum << endl;
        spatial_error_sum += spatial_error_sqr;
    }

    float rms_velocity_measured = sqrt(velocity_sum_measured / (measured_data.size() - 1));
    float rms_velocity_expected = sqrt(velocity_sum_expected / (measured_data.size() - 1));

    float rms_spatial = sqrt(spatial_error_sum / (measured_data.size() - 1));
    float rms_velocity_error = sqrt(velocity_error_sum / (measured_data.size() - 1));
    float expected_end_time = expected_data[expected_data.size() - 1][0] * 1000;

    float temporal_error = abs(1 - (expected_end_time / (measured_data[measured_data.size() - 1][0])));

    rms.push_back(rms_spatial);
    rms.push_back(rms_velocity_expected);
    rms.push_back(rms_velocity_measured);
    rms.push_back(rms_velocity_error * 100);
    rms.push_back(temporal_error * 100);

    return rms;
}

int KortexRobot::start_plot()
{
    FILE *gnuplotPipe = popen("gnuplot -persist", "w");
    KortexRobot::gnu_plot = gnuplotPipe;
    if (!gnu_plot)
    {
        std::cerr << "Error: Gnuplot not found." << std::endl;
        return 1;
    }
    return 0;
}

int KortexRobot::create_plot_file(string file_name, vector<vector<float>> data)
{
    // Open a file for writing
    ofstream file_stream;
    file_stream.open(file_name);
    if (!file_stream.is_open())
    {
        std::cerr << "Error: Unable to open data file." << std::endl;
        return 1;
    }

    vector<vector<float>> data_subset(data);
    for (auto &points : data_subset)
    {
        points = {points.begin() + 1, points.end() - 1};
    }

    for (auto points : data_subset)
    {
        file_stream << points[0] << " " << points[1] << endl;
    }
    file_stream.close();
    return 0;
}

void KortexRobot::init_pids()
{
    bool verbose = true;
    vector<vector<float>> pid_inputs = {
        {0.13, 0.015, 0.0},  // tuned
        {0.21, 0.28, 0.037}, // tuned
        {0.2, 0.25, 0.05},   // tuned
        {0.2, 0.01, 0.001},  // tuned
        {0.18, 0.07, 0.01},  // tuned
        {1, 0.0, 0.0}        // tuned
    }; // This will turn into reading from a json later
    // Probably should include angle limits, max & min control sig limits
    // possibly position thresholds for all angles?

    for (int i = 0; i < actuator_count - 1; i++)
    {
        float k_p = pid_inputs[i][0];
        float k_i = pid_inputs[i][1];
        float k_d = pid_inputs[i][2];

        Pid_Loop pid(k_p, k_i, k_d);
        pids.push_back(pid);
    }
    if (verbose)
    {
        std::cout << "Gain values used (default): " << std::endl;
        for (int i = 0; i < 6; i++)
        {
            std::cout << i << ": {" << pid_inputs[i][0] << ", " << pid_inputs[i][1] << ", " << pid_inputs[i][2] << "}" << std::endl;
        }
    }
}

void KortexRobot::get_gain_values(const std::string &filename)
{
    // Gain files should be a text file of 6 lines, 3 values on each line separated by a space
    // All should have a decimal
    bool verbose = true;
    std::vector<vector<float>> pid_inputs = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    std::ifstream file(filename);
    float value;
    if (!file.is_open())
    {
        std::cerr << "Error opening gain file from path: " << filename << std::endl;
        return;
    }
    int acc_indx = 0;
    int gain_indx = 0;

    while (file >> value)
    {
        pid_inputs[acc_indx][gain_indx] = value;
        gain_indx += 1;
        if (gain_indx == 3)
        {
            acc_indx += 1;
            gain_indx = 0;
        }

        if (acc_indx == 5)
        {
            break;
        }
    }

    for (int i = 0; i < actuator_count - 1; i++)
    {
        float k_p = pid_inputs[i][0];
        float k_i = pid_inputs[i][1];
        float k_d = pid_inputs[i][2];

        Pid_Loop pid(k_p, k_i, k_d);
        pids.push_back(pid);
    }

    if (verbose)
    {
        std::cout << "Gain values used: " << std::endl;
        for (int i = 0; i < 6; i++)
        {
            std::cout << i << ": {" << pid_inputs[i][0] << ", " << pid_inputs[i][1] << ", " << pid_inputs[i][2] << "}" << std::endl;
        }
    }
}

void KortexRobot::connect()
{
    // Create API objects
    auto error_callback = [](k_api::KError err)
    { cout << "_________ callback error _________" << err.toString(); };
    transport = new k_api::TransportClientTcp();
    router = new k_api::RouterClient(transport, error_callback);
    transport->connect(ip_address, PORT);

    transport_real_time = new k_api::TransportClientUdp();
    router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(ip_address, PORT_REAL_TIME);
    // Set session data connection information
    create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(username);
    create_session_info.set_password(password);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    base = new k_api::Base::BaseClient(router);
    base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);

    control_config = new k_api::ControlConfig::ControlConfigClient(router);
    actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);
    device_config = new k_api::DeviceConfig::DeviceConfigClient(router);
    actuator_count = base->GetActuatorCount().count();

    // resets all current errors
    device_config->ClearAllSafetyStatus();
    base->ClearFaults();
    std::cout << "Cleared all errors on Robot" << std::endl;

    set_origin_point();

    bais_vector = {0.0, 0.0, 0.0};
}

void KortexRobot::disconnect()
{
    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();
}

KortexRobot::~KortexRobot()
{
    KortexRobot::disconnect();

    delete base;
    delete base_cyclic;
    delete control_config;
    delete actuator_config;
    delete device_config;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;
}

void KortexRobot::writing_mode()
{
    float kTheta_x = 0.0;
    float kTheta_y = -180.0;
    float kTheta_z = 90.0;

    auto current_pose = base->GetMeasuredCartesianPose();
    vector<vector<float>> current_cartiesian = {{current_pose.x(), current_pose.y(), current_pose.z()}};
    KortexRobot::move_cartesian(current_cartiesian, false, kTheta_x, kTheta_y, kTheta_z);
}

void KortexRobot::go_to_point_jointangle(std::vector<float> q)
{

    // Make sure the arm is in Single Level Servoing before executing an Action
    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);

    auto action = k_api::Base::Action();
    action.set_name("Example angular action movement");
    action.set_application_data("");

    auto reach_joint_angles = action.mutable_reach_joint_angles();
    auto joint_angles = reach_joint_angles->mutable_joint_angles();

    // auto actuator_count = base->GetActuatorCount();

    // Arm to first position in trajectory
    for (size_t i = 0; i < actuator_count; ++i)
    {
        auto joint_angle = joint_angles->add_joint_angles();
        joint_angle->set_joint_identifier(i);
        joint_angle->set_value(q[i]);
    }

    // Connect to notification action topic
    // (Promise alternative)
    // See cartesian examples for Reference alternative
    std::promise<k_api::Base::ActionEvent> finish_promise;
    auto finish_future = finish_promise.get_future();
    auto promise_notification_handle = base->OnNotificationActionTopic(
        create_event_listener_by_promise(finish_promise),
        k_api::Common::NotificationOptions());

    std::cout << "Executing action" << std::endl;
    base->ExecuteAction(action);

    std::cout << "Waiting for movement to finish ..." << std::endl;

    // Wait for future value from promise
    // (Promise alternative)
    // See cartesian examples for Reference alternative
    constexpr auto TIMEOUT_DURATION = std::chrono::seconds{20};

    const auto status = finish_future.wait_for(TIMEOUT_DURATION);
    base->Unsubscribe(promise_notification_handle);

    if (status != std::future_status::ready)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
    }
    const auto promise_event = finish_future.get();

    std::cout << "Angular movement completed" << std::endl;
    std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl;
}

void KortexRobot::go_to_point(const std::string &actionName)
{
    set_actuator_control_mode(0);
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);

    // Move arm to ready position
    std::cout << "Looking for Action named: " << actionName << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_POSE);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);

    for (auto action : action_list.action_list())
    {
        cout << "ACTUON: " << action.name() << endl;
        if (action.name() == actionName)
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0)
    {
        std::cout << "Could not find Action you wanted to exectute" << std::endl;
        std::cout << "Please confirm the action is loaded on the robot by going to the Kinova portal" << std::endl;
    }
    else
    {
        cout << "Executing Action Item: " << actionName << endl;
        bool action_finished = false;
        // Notify of any action topic event
        auto options = k_api::Common::NotificationOptions();
        auto notification_handle = base->OnNotificationActionTopic(
            check_for_end_or_abort(action_finished),
            options);

        base->ExecuteActionFromReference(action_handle);

        while (!action_finished)
        {
            std::this_thread::sleep_for(ACTION_WAITING_TIME);
        }

        base->Unsubscribe(notification_handle);
    }
    cout << "Completed Action";
}

void KortexRobot::find_paper()
{
    // Should have an if statement for config mode where you can change this hardcoded value in real time using the sequence of:
    // 1) Pause the system
    // 2) Wait till user manually moves arm to writing position
    // 3) User presses "enter" in cli to confirm they found the paper
    // 4) Continue
    KortexRobot::move_cartesian({surface_cords});
}

std::function<void(k_api::Base::ActionNotification)>
KortexRobot::check_for_end_or_abort(bool &finished)
{
    return [&finished](k_api::Base::ActionNotification notification)
    {
        std::cout << "EVENT : " << k_api::Base::ActionEvent_Name(notification.action_event()) << std::endl;

        // The action is finished when we receive a END or ABORT event
        switch (notification.action_event())
        {
        case k_api::Base::ActionEvent::ACTION_ABORT:
        case k_api::Base::ActionEvent::ACTION_END:
            finished = true;
            break;
        default:
            break;
        }
    };
}

int64_t KortexRobot::GetTickUs()
{
#if defined(_MSC_VER)
    LARGE_INTEGER start, frequency;

    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);

    return (start.QuadPart * 1000000) / frequency.QuadPart;
#else
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}

void KortexRobot::set_actuator_control_mode(int mode_control, int actuator_indx)
{
    auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
    std::cout << "changed the mode to: ";

    if (mode_control == 0)
    {
        std::cout << "POSITION" << std::endl;
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
    }
    else if (mode_control == 1)
    {
        std::cout << "VELOCITY" << std::endl;
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::VELOCITY);
    }
    else if (mode_control == 2)
    {
        std::cout << "TORQUE" << std::endl;
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
    }
    else
    {
        std::cout << "TORQUE W HIGH VELOCITY" << std::endl;
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE_HIGH_VELOCITY);
    }

    if (actuator_indx == -1)
    {
        actuator_config->SetControlMode(control_mode_message, 1);
        actuator_config->SetControlMode(control_mode_message, 2);
        actuator_config->SetControlMode(control_mode_message, 3);
        actuator_config->SetControlMode(control_mode_message, 4);
        actuator_config->SetControlMode(control_mode_message, 5);
        actuator_config->SetControlMode(control_mode_message, 6);
    }
    else
    {
        actuator_config->SetControlMode(control_mode_message, actuator_indx);
    }
    cout << "Updated Acutators: " << actuator_indx << std::endl;
}

std::vector<std::vector<float>> KortexRobot::read_csv(const std::string &filename, int scale)
{
    std::vector<std::vector<float>> result;

    std::ifstream file(filename);

    int skipper = 0;
    if (!file.is_open())
    {
        std::cerr << "Error opening csv file from path: " << filename << std::endl;
        std::cout << "Terminating program for safety reasons! Please ensure your CSV file is in the correct folder and has the correct filename." << endl;
        abort();
        return result;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::vector<float> row;
        std::string cell;

        while (std::getline(ss, cell, ','))
        {
            try
            {
                float value = std::stof(cell) / scale;
                row.push_back(value);
            }
            catch (const std::invalid_argument &e)
            {
                std::cerr << "Invalid number format in line: " << line << std::endl;
                // Handle the error or skip the invalid value
            }
        }
        if (fmod(skipper, 1) == 0)
        {
            result.push_back(row);
        }
        skipper++;
    }

    file.close();

    return result;
}

vector<vector<float>> KortexRobot::generate_log(const std::string &filename, vector<vector<float>> data)
{
    vector<vector<float>> waypoints;

    std::ofstream file(filename);
    std::ostringstream buffer;

    k_api::Base::Pose pose;

    // populate data
    for (auto angles : data)
    {
        vector<float> point;

        buffer << angles[0] << ','; // Seconds
        point.push_back(angles[0]);
        angles.erase(angles.begin());
        Kinova::Api::Base::JointAngles joint_angles;

        for (auto angle : angles)
        {
            Kinova::Api::Base::JointAngle *joint_angle = joint_angles.add_joint_angles();
            joint_angle->set_value(angle);
        }

        try
        {
            pose = base->ComputeForwardKinematics(joint_angles);
            point.insert(point.end(), {pose.x() + bais_vector[0], pose.y() + bais_vector[1], pose.z() + bais_vector[2]});
            waypoints.push_back(point);
            buffer << pose.x() + bais_vector[0] << ',' << pose.y() + bais_vector[1] << ',' << pose.z() + bais_vector[2] << endl;
        }
        catch (const Kinova::Api::KDetailedException e)
        {
            buffer << "FK failure for the following joint angles:";
            for (int i = 0; i < joint_angles.joint_angles_size(); i++)
            {
                buffer << "\tJoint\t" << i << " : " << joint_angles.joint_angles(i).value();
            }
            buffer << endl;
        }
        // std::cout<< "COORDINATES: " << pose.x()+bais_vector[0] << " " << pose.y()+bais_vector[1] << " " << pose.z()+bais_vector[2] << " " <<endl;
    }

    file << buffer.str();
    file.close();
    return waypoints;
}

void KortexRobot::writeVectorToCSV(const std::vector<std::vector<float>> &data, const std::string &filename)
{
    std::ofstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    for (size_t i = 0; i < data.size(); ++i)
    {
        for (size_t j = 0; j < data[i].size(); ++j)
        {
            file << std::fixed << std::setprecision(8) << data[i][j]; // format float to 2 decimal places
            if (j < data[i].size() - 1)
            {
                file << ","; // separate values with a comma
            }
        }
        file << "\n"; // new line for the next row
    }

    file.close();
    std::cout << "Data written to " << filename << " successfully." << std::endl;
}

std::vector<std::vector<float>> KortexRobot::convert_csv_to_cart_wp(std::vector<std::vector<float>> csv_points, float kTheta_x,
                                                                    float kTheta_y, float kTheta_z)
{
    // bool verbose = false;
    // // Assuming the format of the csv_file will be in (time, x, y, z) for each line
    // vector<float> temp_first_points(3, 0.0);
    // int indx = 0;
    // float offset = 0;
    // int last = 0;
    // float lifted_thresh = 0.0;
    // int lifted_flag = 0;
    // for (int i = 0; i < csv_points.size(); i++)
    // {
    //     lifted_thresh += csv_points[i][3];
    // }
    // lifted_thresh /= csv_points.size();
    // cout << lifted_thresh << endl;
    // for (int i = 0; i < csv_points.size(); i++)
    // {
    //     auto &point = csv_points[i];
    //     if (point.size() > 3)
    //     {
    //         point.erase(point.begin()); // remove seconds value for IR sensor
    //     }
    //     if (indx == 0)
    //     {
    //         temp_first_points = {point[0], point[1], point[2]};
    //         calculate_bias(temp_first_points);
    //         indx = 1;
    //     }
    //     point[0] -= bais_vector[0];
    //     point[1] -= bais_vector[1];

    //     // set flag here for first raise
    //     if (point[2] - lifted_thresh > 0.0)
    //     {
    //         if (lifted_flag == 0)
    //         {
    //             lifted_flag = 1;
    //         }
    //         else
    //         {
    //             if (offset < 0.003)
    //             {
    //                 offset += 0.0002;
    //             }
    //         }
    //         point[2] = altered_origin[2] + offset;
    //     }
    //     else
    //     {
    //         if (lifted_flag == 1)
    //         {
    //             lifted_flag = 0;
    //             last = 1;
    //         }
    //         point[2] = altered_origin[2];
    //     }

    //     // Check if point is outside of boundaries provided (X_MIN/MAX, Y_MIN/MAX)
    //     if (point[0] > X_MAX || point[0] < X_MIN || point[1] > Y_MAX || point[1] < Y_MIN)
    //     {
    //         cout << "The following point (index : " << i << ") is found outside of the set Boundaries imposed on the Robot after the initial bias is applied" << endl;
    //         cout << "Calculated: \tX: " << point[0] << "\tY: " << point[1] << "\tZ: " << point[2] << endl;
    //         cout << "Move the starting point of robot arm over so the entire trajectory fits within the acceptable bounds. Unless the overall trajectory is too large" << endl;
    //         cout << "BOUNDARY Values: " << "\tX_min: " << X_MIN << "\tX_max: " << X_MAX << "\tY_min: " << Y_MIN << "\tY_min: " << Y_MIN;
    //         return {{}};
    //     }
    //     // use this to find the last couple points, and adjust them
    //     if (last == 1)
    //     {
    //         for (int k = 1; k < 31; k++)
    //         {
    //             offset -= 0.0001;
    //             csv_points[i - k][3] = csv_points[i - k][3] - offset;
    //         }
    //         last = 0;
    //     }
    //     point.insert(point.end(), {0, kTheta_x, kTheta_y, kTheta_z});
    // }
    // if (verbose)
    // {
    //     cout << "Modified Vector" << endl;
    //     for (auto point : csv_points)
    //     {
    //         cout << endl;
    //         for (auto element : point)
    //         {
    //             cout << element << " ";
    //         }
    //     }
    //     cout << endl;
    // }

    for (int i = 0; i < csv_points.size(); i++)
    {
        auto &point = csv_points[i];
        if (point.size() > 3)
        {
            point.erase(point.begin()); // remove seconds value for IR sensor
        }
        point.insert(point.end(), {kTheta_x, kTheta_y, kTheta_z});
    }

    // for (auto point : csv_points)
    //     {
    //         cout << endl;
    //         for (auto element : point)
    //         {
    //             cout << element << " ";
    //         }
    //     }
    //     cout << endl;

    cout << csv_points.size() << endl;
    cout << "Sucessfully converted points to cartesian waypoints!";
    return csv_points;
}

void KortexRobot::calculate_bias(std::vector<float> first_waypoint)
{
    bais_vector[0] = first_waypoint[0] - altered_origin[0];
    bais_vector[1] = first_waypoint[1] - altered_origin[1];
    bais_vector[2] = first_waypoint[2] - altered_origin[2];
    cout << "BIAS: X: " << bais_vector[0] << "\tY: " << bais_vector[1] << "\tZ: " << bais_vector[2] << endl;
}

void KortexRobot::populatePreComputedJointTrajectory(k_api::Base::PreComputedJointTrajectory *trajectory, std::vector<vector<float>> jointinput)
{

    // rounds data to reduce runoff in velocity and acceleration values

    int arrsize = (int)jointinput.size();

    for (auto &row : jointinput)
    {
        for (auto &val : row)
        {
            val = std::round(val * 100000.0f) / 100000.0f; // Round to 3 decimal places
        }
    }

    std::vector<std::vector<float>> jointvelocity(arrsize, std::vector<float>(6));
    std::vector<std::vector<float>> jointacceleration(arrsize, std::vector<float>(6));

    writeVectorToCSV(jointinput, "trajectories/jointinputaccepted.csv");

    // takes DT derivative of position to populate velocity array

    for (int i = 0; i < arrsize; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            if (i == arrsize - 1)
            {
                jointvelocity[i][j] = 0;
            }
            else
            {
                jointvelocity[i][j] = (jointinput[i + 1][j] - jointinput[i][j]) / 0.001;
            }
        }
    }

    writeVectorToCSV(jointvelocity, "trajectories/jointvelocity.csv");

    // takes DT derivative of velocity to populate acceleration array

    for (int i = 0; i < arrsize; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            if (i == arrsize - 1 || i == arrsize - 2)
            {
                jointacceleration[i][j] = 0;
            }
            else
            {
                jointacceleration[i][j] = (jointvelocity[i + 1][j] - jointvelocity[i][j]) / 0.001;
            }
        }
    }

    writeVectorToCSV(jointacceleration, "trajectories/jointacceleration.csv");

    // populates trajectory of pre-computed-elements w/ position, velocity, acceleration and timefromstart data

    for (int i = 0; i < jointinput.size(); i++)
    {
        float timefromstart = i * 0.001;
        k_api::Base::PreComputedJointTrajectoryElement *element = trajectory->add_trajectory_elements();
        for (int j = 0; j < 6; j++)
        {
            element->add_joint_angles(jointinput[i][j]);
            element->add_joint_speeds(jointvelocity[i][j]);
            element->add_joint_accelerations(jointacceleration[i][j]);
            element->set_time_from_start(timefromstart);
        }
    }

    // set mode of trajectory to continuity position mode

    trajectory->set_mode(Kinova::Api::Base::TrajectoryContinuityMode::TRAJECTORY_CONTINUITY_MODE_POSITION);
}

vector<vector<float>> KortexRobot::move_cartesian_precompute(std::vector<std::vector<float>> waypointsDefinition, bool repeat, float kTheta_x,
                                                             float kTheta_y, float kTheta_z, bool joints_provided)
{
    std::vector<vector<float>> target_joint_angles_IK;
    std::vector<vector<float>> target_waypoints;

    if (joints_provided)
    {
        target_joint_angles_IK = waypointsDefinition;
    }
    else
    {
        target_waypoints = convert_csv_to_cart_wp(waypointsDefinition, kTheta_x, kTheta_y, kTheta_z);
        target_joint_angles_IK = convert_points_to_angles(target_waypoints);
    }

    cout << "Using the following Joints: ";
    for (int i = 0; i < actuator_count; i++)
    {
        cout << "Joint " << i + 1 << ": ";
        if (actuator_control_types[i] == 0)
        {
            cout << "NO" << endl;
        }
        else
        {
            cout << "YES" << endl;
        }
    }

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    // callback function used in Refresh_callback (Not being used in our code thats why its empty)
    auto lambda_fct_callback = [](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data) {

    };

    std::cout << "Starting angular action movement ..." << std::endl;

    auto action = k_api::Base::Action();
    action.set_name("Example angular action movement");
    action.set_application_data("");

    auto reach_joint_angles = action.mutable_reach_joint_angles();
    auto joint_angles = reach_joint_angles->mutable_joint_angles();

    // auto actuator_count = base->GetActuatorCount();

    // Arm to first position in trajectory
    for (size_t i = 0; i < actuator_count; ++i)
    {
        auto joint_angle = joint_angles->add_joint_angles();
        joint_angle->set_joint_identifier(i);
        joint_angle->set_value(target_joint_angles_IK[0][i]);
    }

    // Connect to notification action topic
    // (Promise alternative)
    // See cartesian examples for Reference alternative
    std::promise<k_api::Base::ActionEvent> finish_promise;
    auto finish_future = finish_promise.get_future();
    auto promise_notification_handle = base->OnNotificationActionTopic(
        create_event_listener_by_promise(finish_promise),
        k_api::Common::NotificationOptions());

    std::cout << "Executing action" << std::endl;
    base->ExecuteAction(action);

    std::cout << "Waiting for movement to finish ..." << std::endl;

    // Wait for future value from promise
    // (Promise alternative)
    // See cartesian examples for Reference alternative
    constexpr auto TIMEOUT_DURATION = std::chrono::seconds{20};

    const auto status = finish_future.wait_for(TIMEOUT_DURATION);
    base->Unsubscribe(promise_notification_handle);

    if (status != std::future_status::ready)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
    }
    const auto promise_event = finish_future.get();

    std::cout << "Angular movement completed" << std::endl;
    std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl;

    // done starting position

    auto precomputedtrajectoryaction = k_api::Base::Action();
    action.set_name("PrecomputedtrajectoryExample");
    action.set_application_data("");

    std::promise<k_api::Base::ActionEvent> finish_promise_precompute;
    auto finish_future_precompute = finish_promise_precompute.get_future();
    auto promise_notification_handle_precompute = base->OnNotificationActionTopic(
        create_event_listener_by_promise(finish_promise_precompute),
        k_api::Common::NotificationOptions());

    std::cout << "Executing action" << std::endl;

    k_api::Base::PreComputedJointTrajectory traj_to_execute;
    populatePreComputedJointTrajectory(&traj_to_execute, target_joint_angles_IK);

    precomputedtrajectoryaction.clear_play_pre_computed_trajectory();
    precomputedtrajectoryaction.set_allocated_play_pre_computed_trajectory(&traj_to_execute);

    // cout << "Is there a precomputed trajectory?" << precomputedtrajectoryaction.has_play_pre_computed_trajectory() << endl;

    k_api::Base::PreComputedJointTrajectory validate = precomputedtrajectoryaction.play_pre_computed_trajectory();

    k_api::Base::PreComputedJointTrajectoryElement analyze_element_1 = validate.trajectory_elements(0);
    k_api::Base::PreComputedJointTrajectoryElement analyze_element_2 = validate.trajectory_elements(1);
    cout << " First Element Position: " << analyze_element_1.joint_angles(0) << "First Element Velocity: " << analyze_element_1.joint_speeds(0)
         << "First Element Acceleration: " << analyze_element_1.joint_accelerations(0) << endl;
    cout << " Last Element Position: " << analyze_element_2.joint_angles(0) << "Last Element Velocity: " << analyze_element_2.joint_speeds(0)
         << "Last Element Acceleration: " << analyze_element_2.joint_accelerations(0) << endl;

    base->ExecuteAction(precomputedtrajectoryaction);

    k_api::Base::TrajectoryErrorReport errorreport = base->GetTrajectoryErrorReport();
    for (int i = 0; i < errorreport.trajectory_error_elements_size(); i++)
    {
        k_api::Base::TrajectoryErrorElement errelement = errorreport.trajectory_error_elements(i);
        cout << "Error Type: " << errelement.error_type() << " " << "Error Message: " << errelement.message() << endl;
    }

    const auto status_precompute = finish_future_precompute.wait_for(TIMEOUT_DURATION);
    base->Unsubscribe(promise_notification_handle_precompute);

    if (status_precompute != std::future_status::ready)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
    }
    const auto promise_event_precompute = finish_future_precompute.get();
}

vector<vector<float>> KortexRobot::adjust_trajectory_origin(vector<vector<float>> trajectory, float start_x, float start_y, float start_z)
{
    float offset_x = start_x - trajectory[0][0];
    float offset_y = start_y - trajectory[0][1];
    // float offset_z = start_z - trajectory[0][2];
    for (int i = 0; i < trajectory.size(); i++)
    {
        trajectory[i][0] += offset_x;
        trajectory[i][1] += offset_y;
        trajectory[i][2] = start_z; // hard-assigns z-values here
    }
    return trajectory;
}

vector<vector<float>> KortexRobot::linear_interp_trajectory(vector<vector<float>> trajectory, float scale)
{
    float unmodified_scale = scale;
    if(scale-floor(scale)!=0){
        scale = ceil(scale);
    }
    int newarrsize = trajectory.size() + (trajectory.size() - 1) * (scale - 1);
    std::vector<std::vector<float>> scaled_arr(newarrsize, std::vector<float>(6));
    for (int p = 1; p < trajectory.size(); p++)
    {

        std::vector<float> q_delta(6);
        for (int j = 0; j < 6; j++)
        {
            q_delta[j] = (trajectory[p][j] - trajectory[p - 1][j]) / scale;
        }
        for (int g = 0; g < scale; g++)
        {
            if (g == 0)
            {
                scaled_arr[(p - 1) * scale] = trajectory[p - 1];
            }
            else
            {
                for (int j = 0; j < 6; j++)
                {
                    scaled_arr[(p - 1) * scale + g][j] = trajectory[p - 1][j] + q_delta[j] * g;
                }
            }
        }
        if (p == trajectory.size() - 1)
            scaled_arr[newarrsize - 1] = trajectory[trajectory.size() - 1];
    }
    float fractional_value_dec = scale- unmodified_scale;
    int num, den;
    findMultiplier(fractional_value_dec, num, den);
    float elems_to_keep = num + floor(unmodified_scale)*den;
    float interval_of_elems = scale*den;

    return scaled_arr;
}

// Function to compute GCD using Euclidean algorithm
int KortexRobot::gcd(int a, int b) {
    return (b == 0) ? a : gcd(b, a % b);
}

// Function to find the smallest multiplier m such that m*x is an integer
void KortexRobot::findMultiplier(double x, int &num, int &den) {
    const int precision = 1e6;  // Scale factor (adjust for precision)
    int scaledNum = round(x * precision);
    int scaledDen = precision;

    int divisor = gcd(scaledNum, scaledDen);
    
    num = scaledNum / divisor;
    den = scaledDen / divisor;
}

vector<vector<float>> KortexRobot::rotate_cartesian_points(vector<vector<float>> cartesian_traj)
{
    vector<vector<float>> modified_traj(cartesian_traj.size(), std::vector<float>(6));
    for (int i = 0; i < cartesian_traj.size(); i++)
    {
        modified_traj[i][0] = -cartesian_traj[i][0];
        modified_traj[i][1] = cartesian_traj[i][1];
        for (int j = 2; j < cartesian_traj[0].size(); j++)
        {
            modified_traj[i][j] = cartesian_traj[i][j];
        }
    }
    return modified_traj;
}

vector<vector<float>> KortexRobot::generate_constant_rotation_vector(float kTheta_x, float kTheta_y, float kTheta_z, int size)
{
    std::vector<std::vector<float>> rot_vector(size, std::vector<float>(3));
    for (int i = 0; i < size; i++)
    {
        rot_vector[i][0] = kTheta_x;
        rot_vector[i][1] = kTheta_y;
        rot_vector[i][2] = kTheta_z;
    }
    return rot_vector;
}

vector<vector<float>> KortexRobot::absolute_value_kinova_syntax(vector<vector<float>> trajectory)
{
    for (int i = 0; i < trajectory.size(); i++)
    {
        for (int j = 0; j < trajectory[0].size(); j++)
        {
            trajectory[i][j] *= 180 / (3.1415926);
            if (trajectory[i][j] < 0)
            {
                trajectory[i][j] += 360;
            }
        }
    }
    return trajectory;
}

KortexRobot::traj_link_list KortexRobot::convert_arr_to_linked_list(std::vector<std::vector<float>> &trajectory)
{
    traj_link_list head = nullptr;
    traj_link_list tail = nullptr;

    for (const auto &vec : trajectory)
    {
        Node *newNode = new Node(vec); // Create a new node with vector<float> data

        if (!head)
        {
            head = newNode; // First node becomes the head
            tail = newNode;
        }
        else
        {
            tail->next = newNode; // Link the new node to the tail
            newNode->prev = tail;
            tail = newNode; // Update the tail
        }
    }

    return head;
}

void KortexRobot::insert_next(traj_link_list ptr, traj_link_list element_to_add)
{
    KortexRobot::traj_link_list ptr_next = ptr->next;
    element_to_add->next = ptr->next;
    element_to_add->prev = ptr;
    if (element_to_add->next != NULL)
    {
        element_to_add->next->prev = element_to_add;
    }
    ptr->next = element_to_add;
}

void KortexRobot::insert_prev(traj_link_list ptr, traj_link_list element_to_add)
{
    KortexRobot::traj_link_list ptr_prev = ptr->prev;
    element_to_add->prev = ptr_prev;
    element_to_add->next = ptr;
    if (ptr_prev)
    {
        ptr_prev->next = element_to_add;
    }
    ptr->prev = element_to_add;
}

void KortexRobot::insert_interpolated_point_after(traj_link_list ptr)
{
    KortexRobot::traj_link_list next = ptr->next;
    std::vector<float> ptr_array = ptr->data;
    std::vector<float> next_array = next->data;
    if (ptr_array.size() != next_array.size())
    {
        cout << "ptr_array size does not equal next_array size" << endl;
        return;
    }
    std::vector<float> interpolated_array(ptr_array.size());
    for (int i = 0; i < ptr_array.size(); i++)
    {
        interpolated_array[i] = (next_array[i] - ptr_array[i]) / 2 + ptr_array[i];
    }
    KortexRobot::traj_link_list interpolated_ptr = new Node(interpolated_array);
    insert_next(ptr, interpolated_ptr);
}

// could be wrong?
bool KortexRobot::velocity_check(KortexRobot::traj_link_list element1, KortexRobot::traj_link_list element2, float refresh_rate)
{
    // assumes size of array in element 1 is the same as element 2, or else fails
    std::vector<float> element1arr = element1->data;
    std::vector<float> element2arr = element2->data;
    if (element1arr.size() != element2arr.size())
    {
        cout << "Error! Arrays do not have same number of indexes!";
        return false;
    }
    else
    {
        for (int i = 0; i < element1arr.size(); i++)
        {
            float speedatindex = abs(element1arr[i] - element2arr[i]) / refresh_rate;
            if (speedatindex > max_joint_speed[i])
            {
                return false;
            }
        }
        return true;
    }
}

// could be wrong?
bool KortexRobot::acceleration_check(KortexRobot::traj_link_list element1, KortexRobot::traj_link_list element3, float refresh_rate)
{
    // assumes element1 is first, element1->next (element2) is mid and element3 is last
    traj_link_list element2 = element1->next;
    std::vector<float> element1arr = element1->data;
    std::vector<float> element2arr = element2->data;
    std::vector<float> element3arr = element3->data;
    if (element1arr.size() != element2arr.size() || element1arr.size() != element3arr.size() || element2arr.size() != element3arr.size())
    {
        cout << "Error! Arrays do not have same number of indexes!";
        return false;
    }
    else
    {
        for (int i = 0; i < element1arr.size(); i++)
        {
            float accelerationatindex = abs(element1arr[i] + element3arr[i] - element2arr[i] * 2) / (refresh_rate * refresh_rate);
            if (accelerationatindex > max_joint_acceleration[i])
            {
                return false;
            }
        }
        return true;
    }
}

vector<vector<float>> KortexRobot::return_valid_trajectory(std::vector<std::vector<float>> trajectory, float refresh_rate)
{
    int traj_size_start = trajectory.size();
    traj_link_list head = convert_arr_to_linked_list(trajectory);
    traj_link_list traverser = head;
    while (traverser->next != nullptr)
    {
        while (!velocity_check(traverser, traverser->next, 0.001))
        {
            insert_interpolated_point_after(traverser);
        }
        traverser = traverser->next;
    }
    vector<vector<float>> valid_traj = convert_linked_list_to_arr(head);
    int traj_size_end = valid_traj.size();
    return valid_traj;
}

std::vector<std::vector<float>> KortexRobot::convert_linked_list_to_arr(KortexRobot::traj_link_list trajectory)
{
    int numberofelements = 0;
    int internalarrsize = trajectory->data.size();
    KortexRobot::traj_link_list traverser = trajectory;
    while (traverser != nullptr)
    {
        numberofelements++;
        traverser = traverser->next;
    }
    std::vector<std::vector<float>> traj_to_return(numberofelements, std::vector<float>(internalarrsize, 0.0f));
    traverser = trajectory;
    int index = 0;
    while (traverser != nullptr)
    {
        traj_to_return[index] = traverser->data;
        index++;
        traverser = traverser->next;
    }
    return traj_to_return;
}

// only affects the start of a trajectory
std::vector<std::vector<float>> KortexRobot::insert_pre_trajectory_elevation_change(std::vector<std::vector<float>> xyz_traj, float elevation_delta, float elevation_time, float delay_time_after_change)
{
    traj_link_list head = convert_arr_to_linked_list(xyz_traj);
    float num_elems_add = (elevation_time + delay_time_after_change) * 1000;
    float z_delta = elevation_delta / (num_elems_add - delay_time_after_change * 1000);
    traj_link_list traverser = head;
    for (int i = 0; i < num_elems_add; i++)
    {
        if (i < delay_time_after_change * 1000)
        {
            std::vector<float> array_at_next_index = traverser->data;
            KortexRobot::traj_link_list prev_ptr = new Node(array_at_next_index);
            insert_prev(traverser, prev_ptr);
            traverser = traverser->prev;
        }
        else
        {
            std::vector<float> array_at_next_index = {traverser->data[0], traverser->data[1], traverser->data[2] + z_delta, traverser->data[3], traverser->data[4], traverser->data[5]};
            KortexRobot::traj_link_list prev_ptr = new Node(array_at_next_index);
            insert_prev(traverser, prev_ptr);
            traverser = traverser->prev;
        }
    }
    return convert_linked_list_to_arr(traverser);
}

std::vector<std::vector<float>> KortexRobot::insert_post_trajectory_elevation_change(std::vector<std::vector<float>> xyz_traj, int index, float elevation_delta, float elevation_time, float delay_time_after_change)
{
    traj_link_list head = convert_arr_to_linked_list(xyz_traj);
    float num_elems_add = (elevation_time + delay_time_after_change) * 1000;
    float z_delta = elevation_delta / (num_elems_add - delay_time_after_change * 1000);
    traj_link_list traverser = head;
    for (int i = 0; i < index; i++)
    {
        traverser = traverser->next;
    }
    for (int i = 0; i < num_elems_add; i++)
    {
        if (i > num_elems_add - (delay_time_after_change * 1000))
        {
            std::vector<float> array_at_next_index = traverser->data;
            KortexRobot::traj_link_list next_ptr = new Node(array_at_next_index);
            insert_next(traverser, next_ptr);
        }
        else
        {
            std::vector<float> array_at_next_index = {traverser->data[0], traverser->data[1], traverser->data[2] + z_delta, traverser->data[3], traverser->data[4], traverser->data[5]};
            KortexRobot::traj_link_list next_ptr = new Node(array_at_next_index);
            insert_next(traverser, next_ptr);
        }
        traverser = traverser->next;
    }
    float fixed_z = traverser->data[2];
    while (traverser->next != NULL)
    {
        traverser->data[2] = fixed_z;
        traverser = traverser->next;
    }
    return convert_linked_list_to_arr(head);
}

bool KortexRobot::at_target_position(std::vector<float> measured, std::vector<float> desired, float eps)
{

    for (int i = 0; i < measured.size(); i++)
    {
        if (abs(measured[i] - desired[i]) > eps)
        {
            return false;
        }
    }
    return true;
}


bool KortexRobot::execute_jointspace_trajectory(std::vector<std::vector<float>> target_joint_angles_IK){
    // setting up trajectory
     // int actuator_count = base->GetActuatorCount().count();
     int actuator_count = 6;
     bool return_status = true;
    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;

    std::vector<float> commands;

    auto servoingMode = k_api::Base::ServoingModeInformation();

    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;

    int timeout = 0;

    auto lambda_fct_callback = [](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data) {};

    std::cout << "Initializing the arm for position low-level control example" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode);
        base_feedback = base_cyclic->RefreshFeedback();

        int actuator_count = base->GetActuatorCount().count();

        // Initialize each actuator to its current position
        for (int i = 0; i < actuator_count; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        float target_index = 0;
        time_duration = target_joint_angles_IK.size()-1;
        // Real-time loop
        while (timer_count < (time_duration))
        {
            base_feedback = base_cyclic->RefreshFeedback();
            now = GetTickUs();
            if (now - last > 1000)
            {
                
                for (int i = 0; i < actuator_count; i++)
                {
                    base_command.mutable_actuators(i)->set_position(target_joint_angles_IK[target_index][i]);
                }
                target_index++;
            
                try
                {
                    base_cyclic->Refresh_callback(base_command, lambda_fct_callback, 0);
                }
                catch (k_api::KDetailedException &ex)
                {
                    // Catch kinova specific errors during RT loop. Output the message and codes
                    // TODO: See if we need to return for all errors, is there a way to recover and continue mid run?
                    std::cout << "Error during RT Loop" << std::endl;
                    printException(ex);
                }

                timer_count++;
                last = GetTickUs();
            }
        }
    }
    catch (k_api::KDetailedException &ex)
    {
        std::cout << "Kortex error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error &ex2)
    {
        std::cout << "Runtime error: " << ex2.what() << std::endl;
        return_status = false;
    }

    // Set back the servoing mode to Single Level Servoing
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}


bool KortexRobot::move_cartesian_PControl(std::vector<std::vector<float>> waypointsDefinition, bool repeat, float scale_factor, float kTheta_x,
                                          float kTheta_y, float kTheta_z, float originX, float originY, float originZ, bool joints_provided)
{

    // NEED TO RE-COMMENT BEFORE RUNNING
    int actuator_count = base->GetActuatorCount().count();
    bool return_status = true;
    std::vector<vector<float>> target_joint_angles_IK;
    std::vector<vector<float>> target_waypoints;
    if (joints_provided)
    {
        target_joint_angles_IK = waypointsDefinition;
    }
    else
    {
        target_waypoints = convert_csv_to_cart_wp(waypointsDefinition, kTheta_x, kTheta_y, kTheta_z);
        target_waypoints = rotate_cartesian_points(target_waypoints);
        target_waypoints = adjust_trajectory_origin(target_waypoints, originX, originY, originZ);
        target_waypoints = insert_pre_trajectory_elevation_change(target_waypoints, 0.1, 2.0 / scale_factor, 3.0 / scale_factor);
        target_waypoints = insert_post_trajectory_elevation_change(target_waypoints, target_waypoints.size() - 1, 0.1, 2.0 / scale_factor, 0.5 / scale_factor);
        //writeVectorToCSV(target_waypoints, "InsertionTest2.csv");
        std::vector<vector<float>> target_joint_angles_IK_unscaled = numerical_inverse_kinematics_trajectory(target_waypoints, 0.001,"a");
        std::vector<vector<float>> target_joint_angles_IK_scaled = linear_interp_trajectory(target_joint_angles_IK_unscaled, scale_factor);
        std::vector<vector<float>> target_joint_angles_IK_velocity_bounded = return_valid_trajectory(target_joint_angles_IK_scaled, 0.001);
        target_joint_angles_IK = absolute_value_kinova_syntax(target_joint_angles_IK_velocity_bounded);
    }
    go_to_point_jointangle(target_joint_angles_IK[0]);

    // setting up trajectory

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;

    std::vector<float> commands;

    auto servoingMode = k_api::Base::ServoingModeInformation();

    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;

    int timeout = 0;

    auto lambda_fct_callback = [](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data) {};

    std::cout << "Initializing the arm for position low-level control example" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode);
        base_feedback = base_cyclic->RefreshFeedback();

        int actuator_count = base->GetActuatorCount().count();

        // Initialize each actuator to its current position
        for (int i = 0; i < actuator_count; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        float target_index = 0;
        time_duration = target_joint_angles_IK.size()-1;
        // Real-time loop
        while (timer_count < (time_duration))
        {
            base_feedback = base_cyclic->RefreshFeedback();
            now = GetTickUs();
            if (now - last > 1000)
            {
                // std::vector<float> measured_pos(actuator_count, 0.0f);
                // for (int i = 0; i < actuator_count; i++)
                // {
                //     measured_pos[i] = base_feedback.actuators(i).position();
                // }
                // bool at_target_pos = at_target_position(measured_pos, target_joint_angles_IK[target_index], 0.085);
                // if (at_target_pos)
                // {
                for (int i = 0; i < actuator_count; i++)
                {
                    base_command.mutable_actuators(i)->set_position(target_joint_angles_IK[target_index][i]);
                }
                target_index++;
                //}
                try
                {
                    base_cyclic->Refresh_callback(base_command, lambda_fct_callback, 0);
                }
                catch (k_api::KDetailedException &ex)
                {
                    // Catch kinova specific errors during RT loop. Output the message and codes
                    // TODO: See if we need to return for all errors, is there a way to recover and continue mid run?
                    std::cout << "Error during RT Loop" << std::endl;
                    printException(ex);
                }

                timer_count++;
                last = GetTickUs();
            }
        }
    }
    catch (k_api::KDetailedException &ex)
    {
        std::cout << "Kortex error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error &ex2)
    {
        std::cout << "Runtime error: " << ex2.what() << std::endl;
        return_status = false;
    }

    // Set back the servoing mode to Single Level Servoing
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}

vector<vector<float>> KortexRobot::move_cartesian(std::vector<std::vector<float>> waypointsDefinition, bool repeat, float kTheta_x,
                                                  float kTheta_y, float kTheta_z, bool joints_provided)
{
    // callback function used in Refresh_callback (Not being used in our code thats why its empty)
    auto lambda_fct_callback = [](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data) {

    };

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;

    auto servoingMode = k_api::Base::ServoingModeInformation();

    std::vector<vector<float>> target_joint_angles_IK;
    std::vector<vector<float>> target_waypoints;

    // DEV-NICK: REMOVE WHEN DONE TESTING
    // TESTING PURPOSES: Use the second set of assignments for target_Waypoints and target_joint_angles_IK for
    //                      general tuning of individual actuators.
    //                      ALSO: Need to change the ready positions check to move onto new stages and which joints are skipped in RT loop

    if (joints_provided)
    {
        target_joint_angles_IK = waypointsDefinition;
    }
    else
    {
        target_waypoints = convert_csv_to_cart_wp(waypointsDefinition, kTheta_x, kTheta_y, kTheta_z);
        target_waypoints = rotate_cartesian_points(target_waypoints);
        target_waypoints = adjust_trajectory_origin(target_waypoints, 0.7, 0, 0.15);
        std::vector<vector<float>> target_joint_angles_IK_unscaled = numerical_inverse_kinematics_trajectory(target_waypoints, 0.001,"a");
        std::vector<vector<float>> target_joint_angles_IK_scaled = linear_interp_trajectory(target_joint_angles_IK_unscaled, 1);
        for (int i = 0; i < 6; i++)
        {
            cout << "J" << i + 1 << target_joint_angles_IK_scaled[target_joint_angles_IK_scaled.size() - 1][i] << " ";
        }
        std::vector<vector<float>> target_joint_angles_IK_velocity_bounded = return_valid_trajectory(target_joint_angles_IK_scaled, 0.001);
        target_joint_angles_IK = absolute_value_kinova_syntax(target_joint_angles_IK_scaled);

        // writeVectorToCSV(target_joint_angles_IK_scaled, "ScaledUnboundedJointsHarrison.csv");
        // writeVectorToCSV(target_joint_angles_IK_velocity_bounded, "ScaledBoundedJointsHarrison.csv");
        //  std::vector<std::vector<float>> XYZ_poststretch = forward_kinematics_trajectory(target_joint_angles_IK_stretched);
        //  writeVectorToCSV(XYZ_poststretch, "HarrisonXYZ.csv");
    }

    //    std::vector<std::vector<float>> target_joint_angles_IK(2000, std::vector<float>(6, 0.0f));
    //     for(int i = 0; i<target_joint_angles_IK.size();i++){
    //         target_joint_angles_IK[i][0] = 0.09;
    //         target_joint_angles_IK[i][1] = 45.04;
    //         target_joint_angles_IK[i][2] = 242.62;
    //         target_joint_angles_IK[i][3] = 358.09;
    //         target_joint_angles_IK[i][4] = 63.99;
    //         target_joint_angles_IK[i][5] = 90.31;
    //     }

    cout << "Using the following Joints: ";
    for (int i = 0; i < actuator_count; i++)
    {
        cout << "Joint " << i + 1 << ": ";
        if (actuator_control_types[i] == 0)
        {
            cout << "NO" << endl;
        }
        else
        {
            cout << "YES" << endl;
        }
    }

    // system("pause");

    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;
    int timeout = 0;
    vector<vector<float>> measured_angles;

    int active_joints = 0;
    try
    {
        // Set the base in low-level servoing mode
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to its current position
        for (int i = 0; i < actuator_count; i++)
        {
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }
        base_feedback = base_cyclic->Refresh(base_command);

        // Set all of the actuators to the appropriate control mode (position or velocity currently)
        for (int i = 1; i <= actuator_count; i++)
        {
            set_actuator_control_mode(actuator_control_types[i - 1], i); // Set actuators to velocity mode dont use index values
            if (actuator_control_types[i - 1] == 0)
            {
                active_joints += 1;
            }
        }

        int stage = 0;
        int num_of_targets = target_joint_angles_IK.size();
        std::vector<int> reachPositions(5, 0);
        std::vector<float> temp_pos(5, 0);
        int64_t start_time = GetTickUs();
        int64_t end_time;
        measured_angles.push_back(measure_joints(base_feedback, start_time)); // Get reference point for start position
        // Real-time loop
        while (timer_count < (time_duration * 1000))
        {
            now = GetTickUs();
            if (now - last > 1000)
            {
                base_feedback = base_cyclic->RefreshFeedback();
                // PID LOOPS WOULD GO WITHIN HERE
                for (int i = 0; i < actuator_count; i++)
                {

                    // TODO: DEV-NICK Possibly remove?
                    // Skip specific Actuators during testing
                    if (actuator_control_types[i] != 0)
                    {
                        continue;
                    }

                    float current_pos = base_feedback.actuators(i).position();
                    float current_velocity = base_feedback.actuators(i).velocity();
                    float current_torque = base_feedback.actuators(i).torque();

                    if (actuator_control_types[i] == 2)
                    {
                        current_velocity = current_torque;
                    }
                    float target_pos = target_joint_angles_IK[stage][i];

                    // Only use position error here aswell to see if we can signal joint reached its target
                    float position_error = target_pos - current_pos;
                    if (abs(position_error) >= 180)
                    {
                        if (position_error > 0)
                        {
                            position_error = position_error - 360;
                        }
                        else
                        {
                            position_error = position_error + 360;
                        }
                    }

                    float control_sig = pids[i].calculate_pid(current_pos, target_pos, i);
                    if (!running_demo)
                    {
                        cout << "[" << stage + 1 << ", " << i << "]\tCurr vs Target Pos: (";
                        cout << std::fixed << std::setprecision(3) << current_pos << ", " << std::fixed << std::setprecision(3) << target_pos << ")";
                        cout << "\tCurrent vs New Vel: (" << std::fixed << std::setprecision(3) << current_velocity << ", " << std::fixed << std::setprecision(3) << control_sig << ")";
                    }

                    // Limit the max absolute value of the new velocity/command being sent
                    if (abs(motor_command[i] - control_sig) > step_change_limit[i])
                    {
                        // Reduce control signal to be atmost
                        if (motor_command[i] < control_sig)
                        {
                            control_sig = motor_command[i] + step_change_limit[i];
                        }
                        else
                        {
                            control_sig = motor_command[i] - step_change_limit[i];
                        }
                        if (!running_demo)
                        {
                            cout << "\t CAP SIG: " << control_sig;
                        }
                    }

                    if (control_sig > command_max[i])
                    {
                        control_sig = command_max[i];
                        if (!running_demo)
                        {
                            cout << "\t MAXED SIG: " << control_sig;
                        }
                    }
                    else if (control_sig < command_min[i])
                    {
                        control_sig = command_min[i];
                        if (!running_demo)
                        {
                            cout << "\t MIN SIG: " << control_sig;
                        }
                    }

                    // Mark if joint is at destination
                    if (std::abs(position_error) > actuator_pos_tolerance[i])
                    {
                        // Update command based on control signal (And control mode of actuator)
                        reachPositions[i] = 0;
                    }
                    else
                    {
                        // std::cout << "\t ARRIVED";
                        reachPositions[i] = 1;
                    }
                    if (!running_demo)
                    {
                        cout << endl;
                    }
                    motor_command[i] = control_sig;
                    temp_pos[i] = current_pos + motor_command[i] * 0.001;
                }
                // sets up frame ids to skip incorrecly sequenced requests
                base_command.set_frame_id(base_command.frame_id() + 1);
                if (base_command.frame_id() > 65535)
                {
                    base_command.set_frame_id(0);
                }

                // decrease the time between motors recieving commands and decrease interference between one another
                for (int i = 0; i < actuator_count - 1; i++)
                {
                    // Update the joints position/velocity/torque based on its control mode
                    if (actuator_control_types[i] == 0)
                    {
                        base_command.mutable_actuators(i)->set_position(temp_pos[i]);
                    }
                    else if (actuator_control_types[i] == 1)
                    {
                        base_command.mutable_actuators(i)->set_position(temp_pos[i]);
                        base_command.mutable_actuators(i)->set_velocity(motor_command[i]);
                    }
                    else
                    {
                        base_command.mutable_actuators(i)->set_position(temp_pos[i]);
                        base_command.mutable_actuators(i)->set_torque_joint(motor_command[i]);
                    }
                    base_command.mutable_actuators(i)->set_command_id(base_command.frame_id());
                }
                // See how many joints are at their target, move onto the next waypoint if all are (Can change how many are "all")
                int ready_joints = std::accumulate(reachPositions.begin(), reachPositions.end(), 0);
                if (ready_joints == active_joints)
                {
                    stage++;
                    std::cout << "finished stage: " << stage << std::endl
                              << std::endl;
                    for (int i = 0; i < actuator_count - 1; i++)
                    {
                        pids[i].clear_integral();
                    }
                    measured_angles.push_back(measure_joints(base_feedback, start_time));
                    reachPositions = {0, 0, 0, 0, 0, 0};
                }
                // Break out of loop if current target is the last
                if (stage == num_of_targets)
                {
                    stage = 0;
                    if (repeat == 0)
                    {
                        end_time = GetTickUs();
                        break;
                    }
                }

                // If still going, send next commands to Arm and recieve feedback in the lambda_callback function.
                try
                {
                    base_cyclic->Refresh_callback(base_command, lambda_fct_callback, 0);
                }
                catch (k_api::KDetailedException &ex)
                {
                    // Catch kinova specific errors during RT loop. Output the message and codes
                    // TODO: See if we need to return for all errors, is there a way to recover and continue mid run?
                    std::cout << "Error during RT Loop" << std::endl;
                    printException(ex);
                }
                catch (...)
                {
                    timeout++;
                }
                timer_count++;
                last = GetTickUs();
            }
        }
        cout << "amount of time it took was:" << setprecision(6) << (end_time - start_time) / 1000000.0 << 's' << endl;
    }
    catch (k_api::KDetailedException &ex)
    {
        // mylogger.Log("Kortex error: " + std::string(ex.what()), ERR);
    }
    catch (std::runtime_error &ex2)
    {
        // mylogger.Log("Runtime error: " + std::string(ex2.what()), ERR);
    }

    set_actuator_control_mode(0);

    // Set back the servoing mode to Single Level Servoing
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);

    return measured_angles;
}

vector<float> KortexRobot::measure_joints(k_api::BaseCyclic::Feedback base_feedback, int64_t start_time)
{
    vector<float> current_joint_angles;
    int64_t curr_time = GetTickUs();
    float time_diff = (curr_time - start_time) / (float)1000000;
    current_joint_angles.push_back(time_diff);

    for (int i = 0; i < actuator_count; i++)
    {
        current_joint_angles.push_back(base_feedback.actuators(i).position());
    }

    return current_joint_angles;
}

void KortexRobot::printException(k_api::KDetailedException &ex)
{
    // You can print the error informations and error codes
    auto error_info = ex.getErrorInfo().getError();
    std::cout << "KDetailedoption detected what:  " << ex.what() << std::endl;

    std::cout << "KError error_code: " << error_info.error_code() << std::endl;
    std::cout << "KError sub_code: " << error_info.error_sub_code() << std::endl;
    std::cout << "KError sub_string: " << error_info.error_sub_string() << std::endl;

    // Error codes by themselves are not very verbose if you don't see their corresponding enum value
    // You can use google::protobuf helpers to get the string enum element for every error code and sub-code
    std::cout << "Error code string equivalent: " << k_api::ErrorCodes_Name(k_api::ErrorCodes(error_info.error_code())) << std::endl;
    std::cout << "Error sub-code string equivalent: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(error_info.error_sub_code())) << std::endl;
}

std::vector<std::vector<float>> KortexRobot::convert_points_to_angles(std::vector<std::vector<float>> target_points)
{
    // Function take an array of target points in format (x,y,z,theta_x,theta_y,theta_z)
    // Feeds it into the Kinova IK function that will spit out the target joint angles
    // for each actuator at each waypoint. Will also ensure all joint angles are positive and within 360.0

    cout << "Starting to convert points to angles using IKIN";

    bool verbose = true;
    std::vector<std::vector<float>> final_joint_angles;
    k_api::Base::JointAngles input_joint_angles;
    vector<float> current_guess(6, 0.0f);
    try
    {
        input_joint_angles = base->GetMeasuredJointAngles();
    }
    catch (k_api::KDetailedException &ex)
    {
        std::cout << "Unable to get current robot pose" << std::endl;
        printException(ex);
        return final_joint_angles;
    }
    int guess_indx = 0;
    for (auto joint_angle : input_joint_angles.joint_angles())
    {
        current_guess[guess_indx] = joint_angle.value();
        guess_indx++;
    }

    cout << "Through guess joint angles, doing computation step of IKIN now!";
    float iterator = 0;
    for (std::vector<float> current_target : target_points)
    {
        cout << iterator / target_points.size() << "\n"
             << endl;
        iterator++;
        // Object containing cartesian coordinates and Angle Guess
        k_api::Base::IKData input_IkData;
        k_api::Base::JointAngle *jAngle;
        k_api::Base::JointAngles computed_joint_angles;

        // Fill the IKData Object with the cartesian coordinates that need to be converted

        input_IkData.mutable_cartesian_pose()->set_x(current_target[0]);
        input_IkData.mutable_cartesian_pose()->set_y(current_target[1]);
        input_IkData.mutable_cartesian_pose()->set_z(current_target[2]);

        input_IkData.mutable_cartesian_pose()->set_theta_x(current_target[4]);
        input_IkData.mutable_cartesian_pose()->set_theta_y(current_target[5]);
        input_IkData.mutable_cartesian_pose()->set_theta_z(current_target[6]);

        // Fill the IKData Object with the guessed joint angles
        guess_indx = 0;
        for (auto joint_angle : input_joint_angles.joint_angles())
        {
            jAngle = input_IkData.mutable_guess()->add_joint_angles();
            jAngle->set_value(current_guess[guess_indx]);
            guess_indx++;
        }

        // Computing Inverse Kinematics (cartesian -> Angle convert) from arm's current pose and joint angles guess
        try
        {
            computed_joint_angles = base->ComputeInverseKinematics(input_IkData);
        }
        catch (k_api::KDetailedException &ex)
        {
            std::cout << "Unable to compute inverse kinematics at index: " << iterator << std::endl;
            printException(ex);
            return final_joint_angles;
        }

        int joint_identifier = 0;
        std::vector<float> temp_joints(6, 0.0f);

        for (auto joint_angle : computed_joint_angles.joint_angles())
        {
            temp_joints[joint_identifier] = joint_angle.value();
            temp_joints[joint_identifier] = fmod(temp_joints[joint_identifier], 360.0);
            if (temp_joints[joint_identifier] < 0.0)
            {
                temp_joints[joint_identifier] = temp_joints[joint_identifier] + 360;
            }
            joint_identifier++;
        }
        current_guess = temp_joints;
        final_joint_angles.push_back(temp_joints);
    }

    if (verbose)
    {
        std::cout << "IK vs Premade generated Angles:" << std::endl;

        for (int i = 0; i < final_joint_angles.size(); i++)
        {
            std::cout << i << ": ";
            for (auto currAngle : final_joint_angles[i])
            {
                std::cout << currAngle << ", ";
            }
            std::cout << std::endl;
        }
    }

    return final_joint_angles;
}

// TODO: Call this function to print all the stuff had in the last example
void KortexRobot::output_arm_limits_and_mode()
{
    // k_api::ControlConfig::ControlModeInformation control_mode;
    // k_api::ActuatorConfig::ControlModeInformation past_mode;
    // k_api::ControlConfig::KinematicLimits hard_limits;
    // k_api::ControlConfig::KinematicLimits soft_limits;
    // k_api::ControlConfig::KinematicLimitsList soft_angle_limits;
    // k_api::ControlConfig::TwistLinearSoftLimit set_angle_limits;

    auto act_mode = k_api::ActuatorConfig::CommandModeInformation();
    auto control_mode = control_config->GetControlMode();
    auto hard_limits = control_config->GetKinematicHardLimits();
    auto soft_limits = control_config->GetKinematicSoftLimits(control_mode);
    auto soft_angle_limits = control_config->GetAllKinematicSoftLimits();

    std::cout << "control mode is: " << control_mode.control_mode() << std::endl;
    std::cout << "actuator control mode is: " << act_mode.command_mode() << std::endl;

    std::cout << "The HARD limits are: " << std::endl;
    std::cout << "Twist Angular: \t" << hard_limits.twist_angular() << std::endl;
    std::cout << "Twist Linear: \t" << hard_limits.twist_linear() << std::endl;
    std::cout << "[Joint 1] Speed: \t" << hard_limits.joint_speed_limits(0) << "\tAcceleration: " << hard_limits.joint_acceleration_limits(0) << std::endl;
    std::cout << "[Joint 2] Speed: \t" << hard_limits.joint_speed_limits(1) << "\tAcceleration: " << hard_limits.joint_acceleration_limits(1) << std::endl;
    std::cout << "[Joint 3] Speed: \t" << hard_limits.joint_speed_limits(2) << "\tAcceleration: " << hard_limits.joint_acceleration_limits(2) << std::endl;
    std::cout << "[Joint 4] Speed: \t" << hard_limits.joint_speed_limits(3) << "\tAcceleration: " << hard_limits.joint_acceleration_limits(3) << std::endl;
    std::cout << "[Joint 5] Speed: \t" << hard_limits.joint_speed_limits(4) << "\tAcceleration: " << hard_limits.joint_acceleration_limits(4) << std::endl;
    std::cout << "[Joint 6] Speed: \t" << hard_limits.joint_speed_limits(5) << "\tAcceleration: " << hard_limits.joint_acceleration_limits(5) << std::endl;
    std::cout << "The SOFT limits are: " << std::endl;
    std::cout << "Twist Angular: \t" << soft_limits.twist_angular() << std::endl;
    std::cout << "Twist Linear: \t" << soft_limits.twist_linear() << std::endl;
    // std::cout<< "[Joint 1] Speed: \t" << soft_limits.joint_speed_limits(0) << "\tAcceleration: " << soft_limits.joint_acceleration_limits(0) << std::endl;
    // std::cout<< "[Joint 2] Speed: \t" << soft_limits.joint_speed_limits(1) << "\tAcceleration: " << soft_limits.joint_acceleration_limits(1) << std::endl;
    // std::cout<< "[Joint 3] Speed: \t" << soft_limits.joint_speed_limits(2) << "\tAcceleration: " << soft_limits.joint_acceleration_limits(2) << std::endl;
    // std::cout<< "[Joint 4] Speed: \t" << soft_limits.joint_speed_limits(3) << "\tAcceleration: " << soft_limits.joint_acceleration_limits(3) << std::endl;
    // std::cout<< "[Joint 5] Speed: \t" << soft_limits.joint_speed_limits(4) << "\tAcceleration: " << soft_limits.joint_acceleration_limits(4) << std::endl;
    // std::cout<< "[Joint 6] Speed: \t" << soft_limits.joint_speed_limits(5) << "\tAcceleration: " << soft_limits.joint_acceleration_limits(5) << std::endl;
    // std::cout<< "Joint Speed: \t" << soft_limits.joint_speed_limits() << std::endl;
    // std::cout<< "Joint Acceleration: \t" << soft_limits.joint_acceleration_limits() << std::endl;
    // soft_angle_limits = control_config->GetAllKinematicSoftLimits();
}

void KortexRobot::output_joint_values_to_csv(std::vector<std::vector<float>> joint_angles, const std::string &filename)
{
    std::ofstream file(filename);
    vector<string> col_headers = {"seconds", "Joint1", "joint2", "joint3", "joint4", "joint5"};

    for (auto each_steps : joint_angles)
    {
        file << each_steps[0] << ',' << each_steps[1] << ',' << each_steps[2] << ',' << each_steps[3] << ',' << each_steps[4] << endl;
    }

    file.close();
}

void KortexRobot::set_origin_point()
{
    auto current_pose = base->GetMeasuredCartesianPose();
    altered_origin = {current_pose.x(), current_pose.y(), current_pose.z()};
    cout << "NEW ORIGIN AT: ()" << altered_origin[0] << ", " << altered_origin[1] << ", " << altered_origin[2] << endl;
}

void KortexRobot::execute_demo()
{
    // TODO: Enter all the correct files either here or as parameters in the function
    // Create a separate example to run this function
    // Possibly option to run on repeat
    // Add functions as parameters so we can swap them out easily
    string pos1 = "Demo_32_Bot_Right";
    string pos2 = "Demo_32_Bot_Left";
    string pos3 = "Demo_32_Top_Left";
    string pos4 = "Demo_32_Top_Right";

    string pos1_elv = "Demo_32_Bot_Right_Elv";
    string pos2_elv = "Demo_32_Bot_Left_Elv";
    string pos3_elv = "Demo_32_Top_Left_Elv";
    string pos4_elv = "Demo_32_Top_Right_Elv";
    // joaquin cursive, pen pal, hello_twice, pieter
    vector<vector<float>> expected_angles_1 = read_csv("Joaquin_Cursive_Filtered_V2__joints_bot_right.csv", 1);
    vector<vector<float>> expected_angles_2 = read_csv("Pen_Pal_with_lift_filtered_V2__joints_bot_left_new.csv", 1);
    vector<vector<float>> expected_angles_3 = read_csv("Hello_twice_Filtered_V2__joints_top_left.csv", 1);
    // vector<vector<float>> expected_angles_4 = read_csv("../coordinates/demo_data/arabic_2_Filtered__joints_top_right.csv", 1); //Confirm it executes on the page
    // Move to first starting position
    set_origin_point();

    while (true)
    {

        go_to_point(pos3_elv);
        go_to_point(pos3);
        vector<vector<float>> measured_joint_angles3 = KortexRobot::move_cartesian(expected_angles_3, false, 180.0, 0.0, 90.0, true);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        go_to_point(pos2_elv);
        go_to_point(pos2);
        vector<vector<float>> measured_joint_angles2 = KortexRobot::move_cartesian(expected_angles_2, false, 180.0, 0.0, 90.0, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        go_to_point(pos1_elv);
        go_to_point(pos1);
        // Update starting point and execute
        vector<vector<float>> measured_joint_angles = KortexRobot::move_cartesian(expected_angles_1, false, 180.0, 0.0, 90.0, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    system("pause");
    // Execute first trajectory
    // move_cartesian()
    // Dont calculate error
    cout << "DONE TRAJECTORY 1 MOVING TO SPOT 2" << endl;

    // go_to_point(pos4);
    // vector<vector<float>> measured_joint_angles4 = KortexRobot::move_cartesian(expected_angles_4, false , 180.0, 0.0, 90.0, true);

    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
}
