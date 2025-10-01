#include "rclcpp/rclcpp.hpp"
#include "asv_acados/mpc_llc_rt.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MpcLlcRtNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}