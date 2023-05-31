#include "rclcpp/rclcpp.hpp"

class TestNode : public rclcpp::Node 
{
public:
    TestNode() : Node("test")
    {
    	RCLCPP_INFO(this->get_logger(), "Test Node cpp has been started.");
    }

private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}