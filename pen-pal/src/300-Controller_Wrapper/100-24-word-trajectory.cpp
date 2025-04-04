#include "KortexRobot.cpp"
#include "logger.cpp"

int main(int argc, char **argv)
{
  auto parsed_args = ParseExampleArguments(argc, argv);
  string input_coordinates_file = parsed_args.coordinates;
  string gain_file = parsed_args.gain;
  bool repeat = parsed_args.repeat;
  bool demo = parsed_args.demo;
  KortexRobot pen_pal(parsed_args.ip_address, parsed_args.username, parsed_args.password, demo);
  pen_pal.get_gain_values(gain_file);
  vector<KortexRobot::TrajectoryData> trajectory_array = pen_pal.pre_compute_q_trajectories("1,2,3,4,5,6,7,8,9,10,11",3, 90, 0, 90, 0.73, 0, 0.063);
  pen_pal.run_trajectory_control("1,2,3,4,5,6,7,8,9,10,11", trajectory_array); 
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
