#include "rclcpp/rclcpp.hpp"
class RobotParamServer : public rclcpp::Node {
    public:
        RobotParamServer() : Node("robot_param_server",
                                    rclcpp::NodeOptions()
                                        .allow_undeclared_parameters(true)
                                        .automatically_declare_parameters_from_overrides(true)) {}
    private:
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotParamServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}