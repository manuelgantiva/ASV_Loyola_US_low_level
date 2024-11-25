import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, RCOut, OverrideRCIn, RCIn
from mavros_msgs.srv import CommandHome, ParamSetV2, CommandBool, SetMode, StreamRate

import numpy as np

class ASVAgent():

    def __init__(self, dt = 0.1, boat="YellowFish"):
        self.h = dt
        if boat=="YellowFish":
            self.eta = np.array([-6.0, -2.0, 0.0]).reshape(3, 1)
            self.nu = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
            self.n = np.round(0.2/dt).astype(int)
            # TODO: actualizar X para cada barco
            self.X = np.vstack([[-0.104329645709408,0.0883743796944665,0.0321333749784260,0.0656967954590652,0.000763919276117807,0.152204872959416,-0.0294397667653587, 0, 0, 0, 0, 0, 0],
                            [0.0263105257157808,-0.142130215251741,-0.00473745627080288,-0.00298699677407208,-0.0712376277975977,0.00187844645250426,0.0963970787967182,0.0113458425889822,0.00310247854255387,0.00508237692346040,-0.0107755417976353,-0.0278141886494755,7.54755161435895e-05],
                            [0.135823516862569,-0.117709166852216,0.0981632226665672,-0.0262800127626117,-0.179133552130113,-0.103398345089823,0.206672730007721,-0.0837078283689221,-0.0640612469258710,-0.0330046194739326,0.0617903020664491,0.249242966912002,0.00341048495680175]]).T
            
            self.X /= 0.2
        else:
            self.eta = np.array([-6.0, -2.0, 0.0]).reshape(3, 1)
            self.nu = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
            self.n = np.round(0.1/dt).astype(int)
            self.X = np.vstack([[-0.0288945937876413,0.0551797510339522,-0.0222058165159210,-0.0404467962780852,-0.0251176364052607,0.989751630600596,0.0307429019288448, 0, 0, 0, 0, 0, 0],
                            [-0.0245466993243880,0.00132719086154355,0.00523419345112100,-0.0122289881910162,-0.00169576053984230,-0.0735862225299305,-0.0242682919964077,-0.0271904208794909,0.0492120145744149,0.00834025103815702,-0.0413011407039152,-0.124288646700212,9.87123656883116e-06],
                            [-0.00340185754404998,0.0275633965295784,0.0139496881004484,-0.0536508151997537,0.000120075906318489,-0.00988466414637409,-0.0146164016081421,-0.0893289923529696,0.114915850609243,-0.0799588832376272,-0.104004476781173,-0.434571412011773,-0.000273040545828366]]).T
            
            self.X /= 0.1
        self.pwm_l = 1500
        self.pwm_r = 1500
    
    def static_2nd_ord(self, d_avg, d_D, f_r, beta):
        '''
        action = [F, T]
        '''
        u, v, r = self.nu.flat
        d = d_avg*d_avg + 0.25*d_D*d_D
        f_rgamma = f_r*(1 - beta)
        a_2_3 = [v*abs(v), v*abs(r), r*abs(v), r*abs(r), u*v,
                 u*r, v, r, f_rgamma*d, d_avg*d_D, f_rgamma*d_avg, 0.5*d_D, 1]
        A = np.array([[u*abs(u), v*r, r*r, u, d, d_avg, 1, 0, 0, 0, 0, 0, 0],
                      a_2_3,
                      a_2_3])
        
        psi = self.eta[2][0]
        ROT = np.array([[np.cos(psi), -np.sin(psi), 0],
                        [np.sin(psi),  np.cos(psi), 0],
                        [0, 0, 1]])
        self.eta += self.h*ROT@self.nu

        self.nu += self.h*np.einsum('ij,ji->i', A, self.X).reshape(3, 1)


    def evolve(self, action, pwm=False):
        if pwm:
            self.pwm_l = int(action[0])
            self.pwm_r = int(action[1])
            action[0] = (action[0] - 1500)/400
            action[1] = (action[1] - 1500)/400
        d_avg = (action[0] + action[1])/2
        d_D = action[0] - action[1]
        signo = np.sign(d_D)
        beta = action[0] > 0 and action[1] > 0

        for _ in range(self.n):
            self.static_2nd_ord(d_avg, d_D, signo, beta)


class SimulatorASVWrapper(Node):

    def __init__(self):
        super().__init__('simulator_asv_wrapper')

        self.declare_parameter('Ts', 100.0)

        self.declare_parameter('my_id', '0')
        self.my_id = self.get_parameter('my_id').get_parameter_value().string_value
        
        boat = "Gazebo" if self.my_id == '0' else "YellowFish"
        self.agent = ASVAgent(boat=boat)

        self.subscription = self.create_subscription(
                OverrideRCIn,
                f'/ASV{self.my_id}/mavros/rc/override',
                self.obtainRCIn,
                10)

        self.publisher_gps_local = self.create_publisher(
                PoseStamped,
                f'/ASV{self.my_id}/mavros/local_position/pose',
                10)
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
        self.get_logger().info(f'/ASV{self.my_id}/mavros/state')
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

        x, y, psi = self.agent.eta
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "asv_frame" # TODO: Check this
        msg.pose.position.x = x[0]
        msg.pose.position.y = y[0]
        msg.pose.position.z = 0.0

        # self.get_logger().info(f'x: {x[0]}, y: {y[0]}, psi: {psi[0]}')
        # psi to quaternion
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = np.sin(psi[0]/2)
        msg.pose.orientation.w = np.cos(psi[0]/2)

        self.publisher_gps_local.publish(msg)
        
        msg_rc_out = RCOut()
        msg_rc_out.header.stamp = self.get_clock().now().to_msg()
        self.out_channel_vals[0], self.out_channel_vals[2] = self.agent.pwm_l, self.agent.pwm_r
        msg_rc_out.channels = self.out_channel_vals
        self.publisher_rc_out.publish(msg_rc_out)
     
        msg_rc_in = RCIn()

        msg_rc_in.header.stamp = self.get_clock().now().to_msg()
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