#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            //Interface reference_llc x->u y->r z->psi
#include "std_msgs/msg/float64.hpp"                 //Interface ref vel mid level controller
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer

#include <cmath>
#include <thread>
#include "asv_library/curvas_loyola.h"

using namespace std;

using std::placeholders::_1;

const int PWMMAX = 1900;
const int PWMMIN = 1100;

// Declaración de contantes

class LyapHlcNode : public rclcpp::Node
{
public:
    LyapHlcNode()) : Node("wang_mlc")
    { 
        std::string my_id; 
        this-> declare_parameter("my_id", "ASV0");   

        //---------Parámetros del HLC-------------------// 

        memory_u.assign(4, 0.0);
        memory_r.assign(4, 0.0);

        a=(taud*Ts)/(taud*Ts+Ts);
        b=1/(taud*Ts+Ts);

        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group=cb_group_sensors_;

        timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts*1000.0)),
                std::bind(&LyapHlcNode::calculateHighLevelController, this), cb_group_obs_);

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&LyapHlcNode::param_callback, this, _1));

        subscriber_states_obs_ = this-> create_subscription<asv_interfaces::msg::StateObserver>(
            "/" + my_id + "/observer/state_observer",rclcpp::SensorDataQoS(), std::bind(&LyapHlcNode::callbackStates,
            this, std::placeholders::_1), options_sensors_);
        subscriber_references_ = this-> create_subscription<std_msgs::msg::Float64>(
            "/" + my_id + "/control/reference_hlc", 1, std::bind(&LyapHlcNode::callbackVelReference,
            this, std::placeholders::_1), options_sensors_);
        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state",1,
                std::bind(&LyapHlcNode::callbackStateData, this, std::placeholders::_1), options_sensors_);
        

private:
    void calculateHighLevelController()
    {

    }