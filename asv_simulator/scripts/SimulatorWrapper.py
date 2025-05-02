import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State, RCOut, OverrideRCIn, RCIn
from mavros_msgs.srv import CommandHome, ParamSetV2, CommandBool, SetMode, StreamRate

import numpy as np

class ASVAgent():

    def __init__(self, dt = 0.1, X=None, drone_id=0, init_pos_v= None):
        self.h = dt
        self.eta = np.array(init_pos_v[0:3]).reshape(3, 1)
        self.nu = np.array(init_pos_v[3:6]).reshape(3, 1)
        self.n = np.round(0.1/dt).astype(int)
        self.X = X
        self.pwm_l = 1500
        self.pwm_r = 1500
        self.deadzone = [1480, 1520]

    
    def static_2nd_ord(self, d_avg, d_D, f_r, beta):
        '''
        action = [F, T]
        '''
        u, v, r = self.nu.flat
        psi = self.eta[2][0]
        ROT = np.array([[np.cos(psi), -np.sin(psi), 0],
                    [np.sin(psi),  np.cos(psi), 0],
                    [0, 0, 1]])
        if d_avg > 0 or d_D != 0:
            d = d_avg*d_avg + 0.25*d_D*d_D
            f_rgamma = f_r*(1 - beta)
            a_2_3 = [v*abs(v), v*abs(r), r*abs(v), r*abs(r), u*v,
                    u*r, v, r, f_rgamma*d, d_avg*d_D, f_rgamma*d_avg, 0.5*d_D, 1]
            A = np.array([[u*abs(u), v*r, r*r, u, d, d_avg, 1, 0, 0, 0, 0, 0, 0],
                        a_2_3,
                        a_2_3])
            self.eta += self.h*ROT@self.nu
            self.eta[2][0] = (self.eta[2][0] + np.pi) % (2*np.pi) - np.pi
            self.nu += np.clip(self.h*np.clip(np.einsum('ij,ji->i', A, self.X).reshape(3, 1), -0.3, 0.3), [[-2.5], [-2.5], [-0.4]], [[2.5], [2.5], [0.4]])
        else: 
            self.eta += self.h*ROT@self.nu
            self.eta[2][0] = (self.eta[2][0] + np.pi) % (2*np.pi) - np.pi


    def evolve(self, action, pwm=False):
        if pwm:
            if self.deadzone[0] <= action[0] <= self.deadzone[1]:
                action[0] = 1500
            if self.deadzone[0] <= action[1] <= self.deadzone[1]:
                action[1] = 1500

            self.pwm_l = int(action[0])
            self.pwm_r = int(action[1])
            action[0] = (action[0] - 1500)/400
            action[1] = (action[1] - 1500)/400
        d_avg = (action[0] + action[1])/2
        d_D = action[0] - action[1]
        signo = np.sign(d_D)
        beta = action[0] > 0 and action[1] > 0

        # for _ in range(self.n):
        self.static_2nd_ord(d_avg, d_D, signo, beta)


class SimulatorASVWrapper(Node):

    def __init__(self):
        super().__init__('simulator_asv_wrapper')

        self.declare_parameter('Ts', 100.0)
        self.declare_parameter('my_id', 0)
        self.my_id = self.get_parameter('my_id').get_parameter_value().integer_value
        
        boat = "Gazebo" if self.my_id == '0' else "YellowFish"

        self.agent = ASVAgent(X=self.getCompletePoly(), drone_id=int(self.my_id), init_pos_v= self.getInitPos())

        self.subscription = self.create_subscription(
                OverrideRCIn,
                f'/ASV{self.my_id}/mavros/rc/override',
                self.obtainRCIn,
                10)
        self.publisher_gps_local = self.create_publisher(
                Odometry,
                f'/ASV{self.my_id}/mavros/global_position/local',
                10)
        self.publisher_gps_velocity = self.create_publisher(
                TwistStamped,
                f'/ASV{self.my_id}/mavros/local_position/velocity_body',
                10)
        self.publisher_imu = self.create_publisher(
                Imu,
                f'/ASV{self.my_id}/comunication/imu_ext/data',
                10
        )
        self.publisher_rc_out = self.create_publisher(
                RCOut,
                f'/ASV{self.my_id}/mavros/rc/out',
                10)
        self.publisher_rc_in = self.create_publisher(
                RCIn,
                f'/ASV{self.my_id}/mavros/rc/in',
                10)
        self.publisher_state = self.create_publisher(
                State,
                f'/ASV{self.my_id}/mavros/state',
                10)
        self.set_mode = self.create_service(
            SetMode,
            f'/ASV{self.my_id}/mavros/set_mode',
            self.set_mode_position_callback)
        
        self.set_param = self.create_service(
            ParamSetV2,
            f'/ASV{self.my_id}/mavros/param/set',
            self.set_param_position_callback)
        
        self.home_service = self.create_service(
            CommandHome,
            f'/ASV{self.my_id}/mavros/cmd/set_home',
            self.set_home_position_callback)

        self.arming = self.create_service(
            CommandBool,
            f'/ASV{self.my_id}/mavros/cmd/arming',
            self.set_arming_position_callback
            )

        self.arming = self.create_service(
            StreamRate,
            f'/ASV{self.my_id}/mavros/set_stream_rate',
            self.set_stream_rate_position_callback
            )


        timer_period = self.get_parameter('Ts').get_parameter_value().double_value / 1000.0
        self.timer_out = self.create_timer(timer_period, self.calculateState)
        self.timer_state = self.create_timer(timer_period, self.updateState)
        self.i = 0
        self.pwm_left = 1500
        self.pwm_right = 1500
        
        self.out_channel_vals = [1500]*18
        self.in_channel_vals = [1500]*18
        self.armed = False
        self.get_logger().info(f'Simulator ASV {boat} has been initialized')

    def obtainRCIn(self, msg):
        self.pwm_left = msg.channels[9]
        self.pwm_right = msg.channels[10]

    def updateState(self):
        
        self.agent.evolve([self.pwm_left, self.pwm_right], pwm=True)

    def calculateState(self):

        
        msg = Odometry()
        stamp_time = self.get_clock().now().to_msg()
        msg.header.stamp = stamp_time
        msg.header.frame_id = f'ASV{self.my_id}'
        x, y, psi = self.obtainENU()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = np.sin(psi/2)
        msg.pose.pose.orientation.w = np.cos(psi/2)

        msg.twist.twist.linear.x = self.agent.nu[0][0]
        msg.twist.twist.linear.y = -self.agent.nu[1][0]
        msg.twist.twist.linear.z = 0.0

        # self.get_logger().info(f'nu{self.agent.nu.T}')

        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = -self.agent.nu[2][0]
        
        msgImu = Imu()
        msgImu.header.stamp = stamp_time
        msgImu.header.frame_id = f'ASV{self.my_id}'
        # msgImu.orientation = 
        msgImu.angular_velocity.z = -self.agent.nu[2][0]
        # msgImu.linear_acceleration =

        msgVel = TwistStamped()
        msgVel.header.stamp = stamp_time
        msgVel.twist.linear.x = self.agent.nu[0][0]
        msgVel.twist.linear.y = -self.agent.nu[1][0]

        self.publisher_gps_local.publish(msg)
        self.publisher_imu.publish(msgImu)
        self.publisher_gps_velocity.publish(msgVel)
        
        msg_rc_out = RCOut()
        msg_rc_out.header.stamp = stamp_time
        
        self.out_channel_vals[0], self.out_channel_vals[2] = self.agent.pwm_l, self.agent.pwm_r
        msg_rc_out.channels = self.out_channel_vals
        self.publisher_rc_out.publish(msg_rc_out)
     
        msg_rc_in = RCIn()

        msg_rc_in.header.stamp = stamp_time
        self.in_channel_vals[1] = 1700  #  APM  < 1400 <   MPC  < 1600 < IFAC
        self.in_channel_vals[2] = 1700  #  ref vel
        
        self.in_channel_vals[3] = 1200  #  ZONO < 1400 < GUILLE < 1600 < LIU
        self.in_channel_vals[5] = 1500  #  AUTO < 1300 <   ROS  < 1700 < MANUAL
        msg_rc_in.channels = self.in_channel_vals
        self.publisher_rc_in.publish(msg_rc_in)

        msg_state = State()
        msg_state.header.stamp = self.get_clock().now().to_msg()
        msg_state.connected = True
        msg_state.armed = self.armed
        msg_state.guided = True
        msg_state.mode = "GUIDED"
        msg_state.system_status = 0
        self.publisher_state.publish(msg_state)

    def obtainENU(self):
        x, y, psi = self.agent.eta
        psi[0] += 1e-5
        dy, dx = -0.2750, 0.2625
        # self.get_logger().info(f'psi in simulator {psi}')
        y, x = x + np.cos(psi)*dx + np.sin(psi)*dy,  y - np.sin(psi)*dx + np.cos(psi)*dy
        psi = -psi + np.pi/2
        
        # qz = np.sin(psi/2)
        # qw = np.cos(psi/2)
        # psi_obs = np.arctan2(2.0 * (qw * qz), qw * qw - qz*qz)
        # psi_obs=-psi_obs+(np.pi/2)
        # self.get_logger().info(f'psi in observer neg  {psi_obs}')
        # if psi_obs<0:
        #     psi_obs = psi_obs + (2*np.pi)
        # self.get_logger().info(f'psi in observer abs  {psi_obs}')

        return x[0], y[0], psi[0]

    def set_mode_position_callback(self, request, response):
        """
           Dummy function to set the mode
        """
        response.mode_sent = True
        response.mode_id = 1
        return response


    def set_home_position_callback(self, request, response):
        """
           Dummy function to set the home position
        """
        response.success = True
        response.result = 1
        return response

    def set_param_position_callback(self, request, response):
        """
           Dummy function to set params
        """
        response.success = True
        # response.integer = 1
        return response

    def set_arming_position_callback(self, request, response):
        """
           Dummy function to set arming
        """
        response.success = True
        response.result = 1
        self.armed = True
        return response

    def set_stream_rate_position_callback(self, request, response):
        """
           Dummy function to set stream rate
        """
        return response
        
    def getCompletePoly(self):
        self.declare_parameter('Xu', [1.0]*13)
        self.declare_parameter('Xv', [1.0]*13)
        self.declare_parameter('Xr', [1.0]*13)
        X = [self.get_parameter(name).get_parameter_value().double_array_value for name in ['Xu', 'Xv', 'Xr']]
        # 
        return np.vstack(X).T

    def getInitPos(self):
        self.declare_parameter('eta_nu_0', [0.0]*6)
        eta_nu_0 = self.get_parameter('eta_nu_0').get_parameter_value().double_array_value
        # self.get_logger().info(f'{eta_nu_0=}')
        return eta_nu_0


def main(args=None):
    rclpy.init(args=args)

    simulator_asv_wrapper = SimulatorASVWrapper()

    rclpy.spin(simulator_asv_wrapper)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simulator_asv_wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()