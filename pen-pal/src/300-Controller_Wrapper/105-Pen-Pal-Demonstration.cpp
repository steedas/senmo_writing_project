#include "KortexRobot.cpp"
#include "logger.cpp"

int main(int argc, char **argv)
{
  auto parsed_args = ParseExampleArguments(argc, argv);
  string input_coordinates_file = parsed_args.coordinates;
  string gain_file = parsed_args.gain;
  bool repeat = parsed_args.repeat;
  bool demo = parsed_args.demo;

  // std::vector<std::vector<float>> xyz;
  // xyz.push_back({0.8f, 0.0f, -0.4f});
  // xyz.push_back({0.8001f,0.0001f,-0.4001f});
  // std::vector<std::vector<float>> euler_angles;
  // euler_angles.push_back({0.0f, 90.0f, 0.0f});
  // euler_angles.push_back({0.00001f, 90.000001f, 0.000001f});
  KortexRobot pen_pal(parsed_args.ip_address, parsed_args.username, parsed_args.password, demo);
  //pen_pal.numerical_inverse_kinematics_trajectory(xyz, euler_angles,0.001);
  pen_pal.get_gain_values(gain_file);
  vector<vector<float>> expected_waypoints = pen_pal.read_csv(input_coordinates_file, 1000);
  pen_pal.move_cartesian_PControl(expected_waypoints, repeat, 3, 90, 0, 90, 0.73, 0, 0.067+0.05, false);
  cout << "Generating Log File..." << endl;
  //vector<vector<float>> measured_waypoints = pen_pal.generate_log("measured_waypoints.csv", measured_joint_angles);
  cout << "Calculating Plot:" << endl;
  //pen_pal.plot(expected_waypoints, measured_waypoints);
  //vector<float> rms = pen_pal.rms_error(expected_waypoints, measured_waypoints);
  //cout << "Spatial(X/Y Plane) RMS Error:\t" << rms[0] << endl;
  //cout << "Velocity RMS Expected:\t" << rms[1] << endl;
  //cout << "Velocity RMS Measured:\t" << rms[2] << endl;
  // cout << "Velocity RMS Error(%):\t"<< rms[3] <<endl;
  //cout << "Temporal Error (%):\t" << rms[4] << endl;

  return 0;
}
