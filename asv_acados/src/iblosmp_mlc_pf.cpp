#include "rclcpp/rclcpp.hpp"
#include "asv_acados/iblosmp_mlc_pf.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IblosmpMlcPfNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}