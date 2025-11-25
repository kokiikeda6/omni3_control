import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import Adafruit_PCA9685
from math import cos, sin, pi

class OmniDriveNode(Node):
    def __init__(self):
        super().__init__('omni_drive_node')
        self.pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
        self.pwm.set_pwm_freq(60)
        self.channels = [0, 4, 8]
        self.STOP = 375
        #self.theta = [0, 2*pi/3, -2*pi/3]
        self.theta = [pi/2, -pi/6, -5*pi/6]
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 1)

    def cmd_vel_callback(self, msg: Twist):
        vx, vy, omega = msg.linear.x, msg.linear.y, msg.angular.z
        motor_sign = [1, 1, 1]  # 各モータの回転方向の符号
        pwm_values = []
        for th, sign in zip(self.theta, motor_sign):
            #speed = vx * cos(th) + vy * sin(th) + omega
            speed = vx * cos(th) + vy * sin(th) + omega*-1
            pwm = int(self.STOP + sign * speed * 200)
            pwm = max(150, min(600, pwm))
            if pwm == self.STOP:
               pwm = 0
            pwm_values.append(pwm)

        for ch, pwm_val in zip(self.channels, pwm_values):
            self.pwm.set_pwm(ch, 0, pwm_val)

def main(args=None):
    rclpy.init(args=args)
    node = OmniDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

