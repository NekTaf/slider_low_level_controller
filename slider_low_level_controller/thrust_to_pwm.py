import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt8MultiArray
from rclpy.qos import QoSPresetProfiles

import numpy as np



class ThrustToPWM(Node):
    def __init__(self):
        super().__init__('thrust_to_pwm')
        
        self.declare_parameter('frequency')
        self.declare_parameter('resolution')
        self.declare_parameter('max_force')

        self.frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().integer_value
        self.max_force = self.get_parameter('max_force').get_parameter_value().double_value
        
        self.signals = [self.create_pwm(0, self.resolution) for i in range(8)]
        self.i = 0
        
        self.create_subscription(
            Vector3,
            'thrust_cmd', 
            self.callback, 
            QoSPresetProfiles.get_from_short_key('system_default')
            )
        
        self.pub = self.create_publisher(
            UInt8MultiArray, 
            '/eight_thrust_pulse', 
            QoSPresetProfiles.get_from_short_key('system_default')
            )
        
        self.create_timer(1/(self.frequency * self.resolution), self.send_signals)
        
        
    def callback(self, msg: Vector3):
        u = np.zeros(6)

        if msg.x < 0:
            u[1] = -msg.x
        else:
            u[0] = msg.x
            
        if msg.y < 0:
            u[3] = -msg.y
        else:
            u[2] = msg.y
            
        if msg.z < 0:
            u[5] = -msg.z
        else:
            u[4] = msg.z
        
        # Thrusters can only fire in one direction (+ positive)
        A = np.array([     
        [0, 0, 1, 0, 1, 0, 0, 0 ],
        [1, 0, 0, 0, 0, 0, 1, 0 ],
        [0, 1, 0, 1, 0, 0, 0, 0 ],
        [0, 0, 0, 0, 0, 1, 0, 1 ],
        [0, 0.14, 0.14, 0, 0, 0.14, 0.14, 0],
        [0.14, 0, 0, 0.14, 0.14, 0, 0, 0.14],
        ], dtype=np.float64)
        
        # Moore–Penrose pseudoinverse
        a_pinv = np.linalg.pinv(A, rcond=1e-6)               
        f = a_pinv @ u
        T = f
        
        # create individual thrust pulses for each physical on-board thruster
        self.signals = []
        for i in range(8):
            self.signals.append(self.create_pwm(T[i], self.resolution))
                
    def send_signals(self):
        self.frequency = self.get_parameter('frequency').get_parameter_value().integer_value

        req = UInt8MultiArray()
        req.data = [int(self.signals[i][self.i]) for i in range(0, 8)]

        self.i += 1
        self.i %= self.resolution
                
        self.pub.publish(req)
        

    def create_pwm(self, thrust:np.ndarray, resolution:int):
        
        """
        :param thrust: ideal continuous thrust per on-board thruster. Size (8,)
        :param resolution: pwm quantization resolution 
        
        :return: list of individual thrust pulses 
        """
        
        value = thrust / self.max_force
        unit = self.max_force / resolution
        
        if value < unit:
            value = 0.0
        
        number_of_pulses = int(value /unit)
        
        signals = [1 for _ in  range(number_of_pulses)]
        signals+= [0 for _ in  range(resolution - number_of_pulses)] 
                    
        return signals
        
        
    
def main(args=None):
    rclpy.init(args=args)
    node = ThrustToPWM()
    rclpy.spin(node)        
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()