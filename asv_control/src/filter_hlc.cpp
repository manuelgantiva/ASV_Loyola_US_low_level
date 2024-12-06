#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            //Interface master reference mpc hlc x->meta y->actual z->vecino
#include "std_msgs/msg/float64.hpp"                 //Interface reference_mlc data-> u_min

#include <cmath>

#define PI 3.141592

using std::placeholders::_1;


class FilterHlcNode : public rclcpp::Node
{
public:
    FilterHlcNode() : Node("filter_hlc") 
    {
        std::string my_id; 
        this-> declare_parameter("my_id", "ASV0");
        this-> declare_parameter("Ts", 100.0);       
        this-> declare_parameter("tau_f", 0.2);

        my_id = (this->get_parameter("my_id").as_string());
        Ts = this->get_parameter("Ts").as_double()/1000.0;
        tau_f = this->get_parameter("tau_f").as_double();
        alpha = Ts / (tau_f + Ts); // Factor del filtro

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&FilterHlcNode::param_callback, this, _1));

        timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts*1000.0)),
                std::bind(&FilterHlcNode::callFiltering, this));

        subscriber_master = this-> create_subscription<geometry_msgs::msg::Vector3>("/" + my_id + "/control/ref_master",10,
                std::bind(&FilterHlcNode::callbackRefMaster, this, std::placeholders::_1));
        subscriber_mavros_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state",1,
                std::bind(&FilterHlcNode::callbackMavrosState, this, std::placeholders::_1));
        publisher_ref = this-> create_publisher<std_msgs::msg::Float64>("/" + my_id + "/control/reference_mlc",1);
    	RCLCPP_INFO(this->get_logger(), "Filter High level control Node in %s has been started.", my_id.c_str());
    }

private:
    void callFiltering()
    {
        if(armed==false){
            ref=0.5;
            out=0.0;
        }else{
            out = alpha * ref + (1.0 - alpha) * out; // Actualizar el estado
            auto msg = std_msgs::msg::Float64();
            msg.data=out;
            publisher_ref->publish(msg);

        }
    }

    void callbackRefMaster(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        ref = msg->y;
    }

    void callbackMavrosState(const mavros_msgs::msg::State::SharedPtr msg)
    {
        armed= msg->armed;
    }

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params){
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto &param: params){
            if (param.get_name() == "tau_f") {
                if (param.as_double() >= 0.0 and param.as_double() < 5.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    tau_f = param.as_double();
                    alpha = Ts / (tau_f + Ts); // Factor del filtro
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0.0-5.0");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
        }
        result.successful = true;
        result.reason = "Success";
        return result;
    }


    bool armed = false;
    float tau_f,Ts, ref = 0.5, out = 0.0, alpha;

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_master;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_mavros_state;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_ref;
    rclcpp::TimerBase::SharedPtr timer_;

    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FilterHlcNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
